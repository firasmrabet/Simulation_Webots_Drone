#!/usr/bin/env python3
# Copyright 1996-2024 Cyberbotics Ltd.
# Adapted for Drone Protection Solaire project

"""Pedestrian walking controller."""
from controller import Supervisor
import optparse
import math
import os

class Pedestrian(Supervisor):
    """Control a Pedestrian PROTO."""

    def __init__(self):
        """Constructor: initialize constants."""
        Supervisor.__init__(self)
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.22
        self.speed = 0.8
        self.current_height_offset = 0
        self.joints_position_field = []
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]
        self.height_offsets = [
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
        self.angles = [
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],  # left arm
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21], # left lower arm
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],       # left hand
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],  # right arm
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38], # right lower arm
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],       # right hand
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],  # left leg
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],       # left lower leg
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],    # left foot
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],  # right leg
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],       # right lower leg
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],    # right foot
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]        # head
        ]
        
        # Idle (standing still) angles
        self.idle_angles = [
            0.0, 0.0, 0.0,  # left arm
            0.0, 0.0, 0.0,  # right arm
            0.0, 0.0, 0.0,  # left leg
            0.0, 0.0, 0.0,  # right leg
            0.0             # head
        ]

    def run(self):
        """Set the Pedestrian pose and position."""
        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--trajectory", default="0 0, 10 0", help="Specify the trajectory in the format [x1 y1, x2 y2, ...]")
        opt_parser.add_option("--speed", type=float, default=0.6, help="Specify walking speed in [m/s]")
        options, args = opt_parser.parse_args()
        
        self.speed = options.speed
        self.time_step = int(self.getBasicTimeStep())
        
        point_list = options.trajectory.split(',')
        if len(point_list) < 2:
            point_list = ["0 0", "10 0"]
            
        self.number_of_waypoints = len(point_list)
        self.waypoints = []
        for i in range(0, self.number_of_waypoints):
            self.waypoints.append([])
            self.waypoints[i].append(float(point_list[i].split()[0]))
            self.waypoints[i].append(float(point_list[i].split()[1]))
            
        self.root_node_ref = self.getSelf()
        
        # Clear signal file on startup
        if os.path.exists('/tmp/drone_go.txt'):
            try:
                os.remove('/tmp/drone_go.txt')
            except:
                pass
                
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        
        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field.append(self.root_node_ref.getField(self.joint_names[i]))

        # compute waypoints distance
        self.waypoints_distance = []
        for i in range(0, self.number_of_waypoints):
            x = self.waypoints[i][0] - self.waypoints[(i + 1) % self.number_of_waypoints][0]
            y = self.waypoints[i][1] - self.waypoints[(i + 1) % self.number_of_waypoints][1]
            if i == 0:
                self.waypoints_distance.append(math.sqrt(x * x + y * y))
            else:
                self.waypoints_distance.append(self.waypoints_distance[i - 1] + math.sqrt(x * x + y * y))
                
        is_walking = False
        start_walking_time = 0

        # Set initial standing pose immediately
        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field[i].setSFFloat(self.idle_angles[i])

        while self.step(self.time_step) != -1:
            # Check if drone has signaled to start
            if not is_walking:
                if os.path.exists('/tmp/drone_go.txt'):
                    is_walking = True
                    start_walking_time = self.getTime()
                else:
                    # Keep standing still at the first waypoint
                    root_translation = [self.waypoints[0][0], self.waypoints[0][1], self.ROOT_HEIGHT]
                    angle = math.atan2(self.waypoints[1][1] - self.waypoints[0][1],
                                       self.waypoints[1][0] - self.waypoints[0][0])
                    rotation = [0, 0, 1, angle]
                    self.root_translation_field.setSFVec3f(root_translation)
                    self.root_rotation_field.setSFRotation(rotation)
                    continue # Skip walk animation logic

            # Calculate time since started walking
            time = self.getTime() - start_walking_time

            current_sequence = int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO) % self.WALK_SEQUENCES_NUMBER)
            ratio = (time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO - \
                int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO))

            for i in range(0, self.BODY_PARTS_NUMBER):
                current_angle = self.angles[i][current_sequence] * (1 - ratio) + \
                    self.angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                self.joints_position_field[i].setSFFloat(current_angle)

            self.current_height_offset = self.height_offsets[current_sequence] * (1 - ratio) + \
                self.height_offsets[(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio

            distance = time * self.speed
            relative_distance = distance - int(distance / self.waypoints_distance[self.number_of_waypoints - 1]) * \
                self.waypoints_distance[self.number_of_waypoints - 1]

            for i in range(0, self.number_of_waypoints):
                if self.waypoints_distance[i] > relative_distance:
                    break

            distance_ratio = 0
            if i == 0:
                distance_ratio = relative_distance / self.waypoints_distance[0]
            else:
                distance_ratio = (relative_distance - self.waypoints_distance[i - 1]) / \
                    (self.waypoints_distance[i] - self.waypoints_distance[i - 1])
                    
            x = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][0] + \
                (1 - distance_ratio) * self.waypoints[i][0]
            y = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][1] + \
                (1 - distance_ratio) * self.waypoints[i][1]
                
            root_translation = [x, y, self.ROOT_HEIGHT + self.current_height_offset]
            angle = math.atan2(self.waypoints[(i + 1) % self.number_of_waypoints][1] - self.waypoints[i][1],
                               self.waypoints[(i + 1) % self.number_of_waypoints][0] - self.waypoints[i][0])
            rotation = [0, 0, 1, angle]

            self.root_translation_field.setSFVec3f(root_translation)
            self.root_rotation_field.setSFRotation(rotation)

if __name__ == '__main__':
    controller = Pedestrian()
    controller.run()
