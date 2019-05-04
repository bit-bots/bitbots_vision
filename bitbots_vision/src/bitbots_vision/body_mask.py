#!/usr/bin/env python2
import rospy
import math
import time
import numpy as np

import tf2_ros as tf2
from tf2_geometry_msgs import PointStamped


class BodyMaskObjectFinder(object):
    def __init__(self):
        self.rate = 30
        self.thickness_scalar = 100
        self.vertical_fov = 90
        self.horizontal_fov = int(90 * 1.8)
        self.x_resolution = 640
        self.y_resolution = 360
        self.frame = 'camera_optical_frame'
        self.tfBuffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.tfBuffer)

    def set_resolution(self, x, y):
        self.x_resolution = x
        self.y_resolution = y

    def start(self):
        self.main_loop()

    def main_loop(self):
        rospy.sleep(0.5)
        while not rospy.is_shutdown():
            print(self.work())
            # TODO nicer
            rospy.sleep(1/float(self.rate))

    def work(self):
        joints = ['l_wrist', 'r_wrist', 'l_lower_arm', 'r_lower_arm']

        joint_positions = map(self.transform_joint, joints)
        
        angles = map(self.calculate_arm_point_angle, joint_positions)

        relative_image_points = map(self.image_position_from_angle, angles)
        
        absolute_image_points = map(self.to_absolute, relative_image_points)

        distance = map(self.distance, joint_positions)

        thickness = map(self.distance_to_thickness, distance)

        left_arm = (absolute_image_points[0], absolute_image_points[2], thickness[0])
        right_arm = (absolute_image_points[1], absolute_image_points[3], thickness[1])

        objects = [left_arm, right_arm]

        return objects

    def distance_to_thickness(self, distance):
        return int(1/(distance+1) * self.thickness_scalar)

    def to_absolute(self, point):
        return int(point[0] + (self.x_resolution/2)), int(point[1] + (self.y_resolution/2))

    def distance(self, relative_joint_position):
        rjp = relative_joint_position
        return math.sqrt(rjp.x**2 + rjp.y**2 + rjp.z**2)

    def transform_joint(self, joint):
        #TODO try
        return self.tfBuffer.lookup_transform(self.frame, joint, rospy.Time(0)).transform.translation

    def calculate_arm_point_angle(self, relative_joint_position):
        y_rotation = math.atan2(float(relative_joint_position.x), float(relative_joint_position.z))
        x_rotation = math.atan2(float(relative_joint_position.y), float(relative_joint_position.z))
        # Finde why its off by factor 2
        return x_rotation, y_rotation

    def image_position_from_angle(self, angle):
        scalar_x = self.horizontal_fov/float(self.x_resolution)
        scalar_y = self.vertical_fov/float(self.y_resolution)
        return int(math.degrees(angle[1]) * scalar_x), int(math.degrees(angle[0]) * scalar_y)


if __name__ == "__main__":
    rospy.init_node('body_mask')
    BodyMaskObjectFinder().start()
