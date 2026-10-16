#!/usr/bin/env python3
"""
Drone Protection Solaire — VERSION FINALE (Dual Mode)
=====================================================
Mode 1 (Automatique / GPS)       : Décollage → GPS Direct → Descente → Suivi GPS
Mode 2 (Vision Intelligente IA)  : Décollage → Scan 360° → YOLO Détection → Descente → Suivi
Commutation en temps réel via les touches clavier '1' et '2'.
"""

from controller import Supervisor
import sys, math, os

try:
    import numpy as np
except ImportError:
    sys.exit("numpy manquant")

try:
    import cv2
    from ultralytics import YOLO
    HAS_VISION = True
except ImportError:
    print("[!] Modules cv2/ultralytics introuvables. Le mode caméra ne sera pas disponible.")
    HAS_VISION = False


class DroneProtection(Supervisor):
    # ── Constantes PID de vol ──
    K_VERT_THRUST  = 68.5
    K_VERT_OFFSET  = 0.6
    K_VERT_P       = 5.0
    K_ROLL_P       = 50.0
    K_PITCH_P      = 30.0

    # ── Phases de vol (Mode Vision IA — Mode 2) ──
    PH_LIFTOFF    = 0
    PH_SCAN       = 1
    PH_INTERCEPT  = 2
    PH_LOWER      = 3
    PH_TRACK      = 4

    # ── Phases de vol (Mode GPS Auto — Mode 1) ──
    PH_AUTO_LIFT     = 10
    PH_AUTO_NAV      = 11
    PH_AUTO_LOWER    = 12
    PH_AUTO_TRACK    = 13

    # ── Modes de fonctionnement ──
    MODE_AUTO   = 1
    MODE_VISION = 2

    CRUISE_ALT    = 3.0
    HOVER_ALT     = 2.1
    TRACK_ALPHA   = 0.75
    DESCENT_RATE  = 0.07

    def __init__(self):
        Supervisor.__init__(self)
        self.dt = int(self.getBasicTimeStep())

        # ── Caméras ──
        self.cam_front = self.getDevice("camera")
        if not self.cam_front:
            print("[ERREUR CRITIQUE] Caméra frontale non trouvée !")
            for i in range(self.getNumberOfDevices()):
                print(f"   -> {self.getDeviceByIndex(i).getName()}")
            sys.exit(1)
        self.cam_front.enable(self.dt)

        self.cam_bottom = self.getDevice("track_camera")
        if self.cam_bottom:
            self.cam_bottom.enable(self.dt)
        else:
            self.cam_bottom = self.cam_front

        self.active_cam = self.cam_front

        # ── Capteurs ──
        self.imu  = self.getDevice("inertial unit"); self.imu.enable(self.dt)
        self.gps  = self.getDevice("gps");           self.gps.enable(self.dt)
        self.gyro = self.getDevice("gyro");          self.gyro.enable(self.dt)
        self.keyboard.enable(self.dt)

        # ── Moteurs ──
        self.prop_fl = self.getDevice("front left propeller")
        self.prop_fr = self.getDevice("front right propeller")
        self.prop_rl = self.getDevice("rear left propeller")
        self.prop_rr = self.getDevice("rear right propeller")
        for p in [self.prop_fl, self.prop_fr, self.prop_rl, self.prop_rr]:
            p.setPosition(float('inf'))
            p.setVelocity(1)

        # ── Gimbal ──
        for axis in ["camera pitch", "camera yaw", "camera roll"]:
            self.getDevice(axis).setPosition(0.0)

        # ── Références de scène ──
        self.drone_node      = self.getSelf()
        self.drone_pos_field = self.drone_node.getField("translation")
        self.drone_rot_field = self.drone_node.getField("rotation")
        self.ped_node        = self.getFromDef("PEDESTRIAN")

        # ── État initial ──
        self.flight_mode = self.MODE_VISION
        self.alt_target = self.CRUISE_ALT
        self.phase = self.PH_LIFTOFF
        self.confirm_count = 0
        self.tick = 0
        self.settle_count = 0
        self.scan_angle = 0.0

        if HAS_VISION:
            print("[INIT] Importation réseau neuronal YOLOv8n en cours...")
            self.yolo = YOLO("yolov8n.pt")
            print("[INIT] Réseau neuronal opérationnel.")

    # ── Signaler au piéton de commencer à marcher ──
    def signal_start(self):
        try: open('/tmp/drone_go.txt', 'w').write('GO')
        except: pass

    # ── Obtenir la position du piéton ──
    def read_ped_pos(self):
        if self.ped_node:
            return list(self.ped_node.getField("translation").getSFVec3f())
        return [0.0, 0.0, 0.0]

    # ── Détection couleur verte (vêtement du piéton) ──
    def detect_color_fast(self):
        img = self.active_cam.getImage()
        if not img: return False
        w, h = self.active_cam.getWidth(), self.active_cam.getHeight()
        raw = np.frombuffer(img, np.uint8).reshape((h, w, 4))
        bgr = raw[:, :, :3]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([35, 60, 60]), np.array([85, 255, 255]))
        return cv2.countNonZero(mask) > 180

    # ── Détection complète YOLO + couleur ──
    def detect_person(self):
        img = self.active_cam.getImage()
        if not img: return False, False
        w, h = self.active_cam.getWidth(), self.active_cam.getHeight()
        raw = np.frombuffer(img, np.uint8).reshape((h, w, 4))
        bgr = raw[:, :, :3]

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([35, 60, 60]), np.array([85, 255, 255]))
        color_detected = cv2.countNonZero(mask) > 180

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        results = self.yolo(rgb, conf=0.15, verbose=False)
        yolo_detected = False
        if results and len(results[0].boxes) > 0:
            yolo_detected = any(int(b.cls[0]) == 0 for b in results[0].boxes)

        return yolo_detected, color_detected

    # ── Déplacer le drone au-dessus du piéton ──
    def navigate_above(self, target, alt=None, smooth=None):
        if alt is None:
            alt = self.alt_target
        if smooth is None:
            smooth = self.TRACK_ALPHA
        cur = self.drone_pos_field.getSFVec3f()
        nx = cur[0] + smooth * (target[0] - cur[0])
        ny = cur[1] + smooth * (target[1] - cur[1])
        nz = cur[2] + smooth * (alt - cur[2])
        self.drone_pos_field.setSFVec3f([nx, ny, nz])
        self.drone_node.resetPhysics()
        dx = target[0] - nx
        dy = target[1] - ny
        return math.sqrt(dx * dx + dy * dy)

    # ── Stabiliser en vol stationnaire ──
    def stabilize_hover(self):
        t = self.K_VERT_THRUST
        self.prop_fl.setVelocity(t)
        self.prop_fr.setVelocity(-t)
        self.prop_rl.setVelocity(-t)
        self.prop_rr.setVelocity(t)

    # ── Gestion de la commutation de mode ──
    def switch_mode(self, key):
        if key == ord('1') and self.flight_mode != self.MODE_AUTO:
            self.flight_mode = self.MODE_AUTO
            self.phase = self.PH_AUTO_LIFT
            self.confirm_count = 0
            self.settle_count = 0
            print("+" * 50)
            print("  [SWITCH] Passage au pilotage GPS automatique")
            print("  Localisation par coordonnées satellites uniquement.")
            print("+" * 50)

        elif key == ord('2') and self.flight_mode != self.MODE_VISION:
            if not HAS_VISION:
                print("[!] Bibliothèques IA absentes, changement refusé.")
                return
            self.flight_mode = self.MODE_VISION
            self.phase = self.PH_LIFTOFF
            self.confirm_count = 0
            self.settle_count = 0
            self.scan_angle = 0.0
            print("+" * 50)
            print("  [SWITCH] Passage au pilotage par intelligence artificielle")
            print("  Analyse d'image YOLO activée pour reconnaissance humaine.")
            print("+" * 50)

    # ══════════════════════════════════════════════════════════
    #  MODE GPS AUTOMATIQUE (Mode 1)
    # ══════════════════════════════════════════════════════════
    def execute_auto_mode(self, roll, pitch, yaw, x, y, z, r_acc, p_acc):

        if self.phase == self.PH_AUTO_LIFT:
            if z > self.CRUISE_ALT - 0.5:
                self.phase = self.PH_AUTO_NAV
                print("  ~ Altitude de croisière OK. Cap vers la cible GPS...")

            alt_err = max(min(self.CRUISE_ALT - z + self.K_VERT_OFFSET, 1), -1)
            vert = self.K_VERT_P * pow(alt_err, 3.0)
            ri = self.K_ROLL_P * max(min(roll, 1), -1) + r_acc
            pi = self.K_PITCH_P * max(min(pitch, 1), -1) + p_acc
            self.prop_fl.setVelocity(  self.K_VERT_THRUST + vert + pi - ri)
            self.prop_fr.setVelocity(-(self.K_VERT_THRUST + vert + pi + ri))
            self.prop_rl.setVelocity(-(self.K_VERT_THRUST + vert - pi - ri))
            self.prop_rr.setVelocity(  self.K_VERT_THRUST + vert - pi + ri)

        elif self.phase == self.PH_AUTO_NAV:
            ped = self.read_ped_pos()
            cur = self.drone_pos_field.getSFVec3f()
            heading = math.atan2(ped[1] - cur[1], ped[0] - cur[0])
            self.drone_rot_field.setSFRotation([0, 0, 1, heading])

            dist = self.navigate_above(ped, self.CRUISE_ALT, smooth=0.05)
            self.stabilize_hover()

            if dist < 0.4:
                self.phase = self.PH_AUTO_LOWER
                self.settle_count = 0
                print("  ~ Cible atteinte, amorçage de la descente progressive...")

        elif self.phase == self.PH_AUTO_LOWER:
            ped = self.read_ped_pos()
            self.navigate_above(ped, self.HOVER_ALT, self.DESCENT_RATE)
            self.stabilize_hover()

            cur_z = self.drone_pos_field.getSFVec3f()[2]
            if abs(cur_z - self.HOVER_ALT) < 0.15:
                self.settle_count += 1
                if self.settle_count >= 10:
                    self.phase = self.PH_AUTO_TRACK
                    self.signal_start()
                    print("~" * 50)
                    print("  [GPS] Drone stabilisé. Démarrage de l'accompagnement.")
                    print("  Le piéton est en mouvement — protection active.")
                    print("~" * 50)

        elif self.phase == self.PH_AUTO_TRACK:
            ped = self.read_ped_pos()
            dist = self.navigate_above(ped, self.HOVER_ALT)
            self.stabilize_hover()

            cur = self.drone_pos_field.getSFVec3f()
            heading = math.atan2(ped[1] - cur[1], ped[0] - cur[0])
            self.drone_rot_field.setSFRotation([0, 0, 1, heading])

            if self.tick % 100 == 0:
                print(f"  [GPS] Écart latéral: {dist:.2f}m | Coordonnées piéton: x={ped[0]:.1f} y={ped[1]:.1f}")

    # ══════════════════════════════════════════════════════════
    #  MODE VISION IA (Mode 2)
    # ══════════════════════════════════════════════════════════
    def execute_vision_mode(self, roll, pitch, yaw, x, y, z, r_acc, p_acc):

        if self.phase == self.PH_LIFTOFF:
            if z > self.CRUISE_ALT - 0.5:
                self.phase = self.PH_SCAN
                print("  ~ Altitude correcte. Lancement du balayage visuel 360°...")

            alt_err = max(min(self.CRUISE_ALT - z + self.K_VERT_OFFSET, 1), -1)
            vert = self.K_VERT_P * pow(alt_err, 3.0)
            ri = self.K_ROLL_P * max(min(roll, 1), -1) + r_acc
            pi = self.K_PITCH_P * max(min(pitch, 1), -1) + p_acc
            self.prop_fl.setVelocity(  self.K_VERT_THRUST + vert + pi - ri)
            self.prop_fr.setVelocity(-(self.K_VERT_THRUST + vert + pi + ri))
            self.prop_rl.setVelocity(-(self.K_VERT_THRUST + vert - pi - ri))
            self.prop_rr.setVelocity(  self.K_VERT_THRUST + vert - pi + ri)

        elif self.phase == self.PH_SCAN:
            self.active_cam = self.cam_front
            self.getDevice("camera pitch").setPosition(-0.3)

            self.scan_angle += 0.03
            self.drone_rot_field.setSFRotation([0, 0, 1, self.scan_angle])

            cur = self.drone_pos_field.getSFVec3f()
            self.navigate_above(cur, self.CRUISE_ALT)
            self.stabilize_hover()

            # Ne commence la détection qu'après un tour complet (360° = 2*pi)
            if self.scan_angle < 6.30:
                return  # Continuer le scan sans chercher

            if self.tick % 10 == 0:
                yolo_ok, color_ok = self.detect_person()
                if yolo_ok or color_ok:
                    self.confirm_count += 1
                    if self.confirm_count >= 3:
                        self.phase = self.PH_INTERCEPT
                        print("  ~ [IA] Silhouette humaine confirmée ! Vol d'interception lancé.")
                else:
                    self.confirm_count = 0

        elif self.phase == self.PH_INTERCEPT:
            ped = self.read_ped_pos()
            cur = self.drone_pos_field.getSFVec3f()
            heading = math.atan2(ped[1] - cur[1], ped[0] - cur[0])
            self.drone_rot_field.setSFRotation([0, 0, 1, heading])

            dist = self.navigate_above(ped, self.CRUISE_ALT, smooth=0.03)
            self.stabilize_hover()

            if dist < 0.4:
                self.phase = self.PH_LOWER
                self.settle_count = 0
                print("  ~ [IA] Position verticale acquise. Réduction d'altitude...")

        elif self.phase == self.PH_LOWER:
            self.active_cam = self.cam_bottom
            ped = self.read_ped_pos()
            self.navigate_above(ped, self.HOVER_ALT, self.DESCENT_RATE)
            self.stabilize_hover()

            cur_z = self.drone_pos_field.getSFVec3f()[2]
            if abs(cur_z - self.HOVER_ALT) < 0.15:
                self.settle_count += 1
                if self.settle_count >= 10:
                    self.phase = self.PH_TRACK
                    self.signal_start()
                    print("~" * 50)
                    print("  [IA] Verrouillage terminé. Accompagnement visuel en cours.")
                    print("  Piéton détecté et suivi — protection active.")
                    print("~" * 50)

        elif self.phase == self.PH_TRACK:
            self.active_cam = self.cam_bottom
            ped = self.read_ped_pos()
            dist = self.navigate_above(ped, self.HOVER_ALT)
            self.stabilize_hover()

            if self.tick % 20 == 0:
                color = self.detect_color_fast()
                if self.tick % 100 == 0:
                    status = "confirmé" if color else "non visible"
                    print(f"  [IA] Écart: {dist:.2f}m | Marqueur visuel: {status}")

    # ══════════════════════════════════════════════════════════
    #  BOUCLE PRINCIPALE
    # ══════════════════════════════════════════════════════════
    def run(self):
        labels = {self.MODE_AUTO: "GPS Automatique", self.MODE_VISION: "Intelligence Artificielle"}
        print("_" * 50)
        print("  SYSTÈME DE PROTECTION SOLAIRE PAR DRONE")
        print(f"  Pilotage actuel : {labels[self.flight_mode]}")
        print("  Raccourcis : [1] GPS  |  [2] IA (YOLO)")
        print("_" * 50)

        while self.step(self.dt) != -1:
            key = self.keyboard.getKey()
            self.tick += 1

            if key in [ord('1'), ord('2')]:
                self.switch_mode(key)

            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x, y, z = self.gps.getValues()
            r_acc, p_acc, _ = self.gyro.getValues()

            if self.flight_mode == self.MODE_AUTO:
                self.execute_auto_mode(roll, pitch, yaw, x, y, z, r_acc, p_acc)
            else:
                self.execute_vision_mode(roll, pitch, yaw, x, y, z, r_acc, p_acc)


if __name__ == '__main__':
    DroneProtection().run()
