#!/usr/bin/env python2
# TODO: remove

import rospy
import math
import numpy as np
import cv2
from bitbots_vision.vision_modules import debug, ball, candidate
from sensor_msgs.msg import CameraInfo
import tf2_ros as tf2
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point

    # TODO: docs
    # TODO: set resolution on transformer stuff
    # TODO: publish debug image as overlay of body mask with 0.5 opacity
    # TODO: rename class to BallCandidateBodyMaskFilter

class BodyMaskBallCandidateFilter(object):
    """
    TODO
    """
    def __init__(self, debug_printer, config):
        # type: (DebugPrinter, (int, int), dict) -> None
        """
        TODO
        """
        self.max_intersection_threshold = config['vision_ball_own_body_max_intersection_threshold']
        self.finder = BodyMaskObjectFinder()
        # To accommodate termination issues
        """
        while not rospy.is_shutdown():
            self.test()
        """
        # rospy.spin()

    def get_body_parts(self, image_size):
        # type: () -> [(int, int), (int, int), int]
        """
        TODO
        """
        self.finder.set_resolution(image_size[1],image_size[0])
        
        return self.finder.work()

    def get_body_mask(self, body_parts, image_size):
        # type: ([(int, int), (int, int), int]) -> TODO
        """
        TODO
        """
        # Generate white canvas
        mask = np.ones(image_size, dtype=np.uint8) * 255
        # Draw black lines for the parts of the own body
        for body_part in body_parts:
            cv2.line(mask, body_part[0], body_part[1], (0,0,0), thickness=body_part[2])
        self.imshow(mask)
        return mask

    # TODO: remove
    def test(self):
        objects = self.get_body_parts()
        self.imshow(self.get_body_mask(objects))
    
    def get_ball_mask(self, ball_candidate, image_size):
        # type: () -> TODO
        """
        TODO
        """
        center = ball_candidate.get_center_point()
        radius = ball_candidate.get_radius()
        # Generates a black canvas
        canvas = np.zeros(image_size, dtype=np.uint8)
        # Draw white circle representing a ball
        mask = cv2.circle(canvas, center, radius, (255, 255, 255), thickness=-1)
        return mask

    # TODO: remove
    def imshow(self, image):
        cv2.imshow("image", image)
        k = cv2.waitKey(1)

    def ball_candidate_not_on_own_body(self, ball_candidate, body_mask, image_size):
        # type: () -> bool
        """
        TODO
        """
        # Area of ball candidate in number of pixels
        ball_area = math.pi * ball_candidate.get_radius() ** 2

        ball_mask = self.get_ball_mask(ball_candidate, image_size)

        # Number of pixels intersecting both masks
        intersection_area = cv2.countNonZero(
            cv2.bitwise_and(ball_mask, body_mask))

        # True, if percentile of intersecting area 
        return (intersection_area / ball_area) <= self.max_intersection_threshold

    def get_ball_candidates_not_on_own_body(self, ball_candidates, image_size):
        # type: (TODO) -> TODO
        """
        TODO
        """
        body_parts = self.get_body_parts(image_size)
        body_mask = self.get_body_mask(body_parts, image_size)
        return [ball_candidate for ball_candidate in ball_candidates if self.ball_candidate_not_on_own_body(
            ball_candidate,
            body_mask,
            image_size)]


class BodyMaskObjectFinder(object):
    def __init__(self):
        self.rate = 30
        self.arm_width = 0.05
        self.vertical_fov = 70
        self.horizontal_fov = int(70 * 1.7)
        self.x_resolution = 640
        self.y_resolution = 360
        self.frame = 'camera_optical_frame'
        self.camera_info = None
        self.tfBuffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.tfBuffer)
        rospy.Subscriber("camera_info",
                        CameraInfo,
                        self._callback_camera_info,
                        queue_size=1)

    def _callback_camera_info(self, camera_info):
        self.camera_info = camera_info
        self.calculate_fov()

    def calculate_fov(self):
        K = self.camera_info.K
        
        point = Point()
        point.x = (self.x_resolution - K[2]) / K[0]
        point.y = (self.y_resolution- K[5]) / K[4]
        point.z = 1.0

        angles = self.calculate_point_angle(point)

        self.horizontal_fov = int(math.degrees(angles[1]*2))
        self.vertical_fov = int(math.degrees(angles[0]*2))

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

        try:
            joint_positions = map(self.transform_joint, joints)
        except:
            rospy.loginfo_throttle(10, "Not able to tf arms. If vision runs in sim ignore this message.")
            return []
        
        angles = map(self.calculate_point_angle, joint_positions)

        relative_image_points = map(self.image_position_from_angle, angles)
        
        absolute_image_points = map(self.to_absolute, relative_image_points)

        distance = map(self.distance, joint_positions)

        thickness = map(self.distance_to_thickness, distance)

        left_arm = (absolute_image_points[0], absolute_image_points[2], thickness[0])
        right_arm = (absolute_image_points[1], absolute_image_points[3], thickness[1])

        objects = [left_arm, right_arm]

        return objects

    def distance_to_thickness(self, distance):
        angle_at_camera = math.atan2(float(self.arm_width), float(distance))
        thickness = self.image_position_from_angle((0,angle_at_camera))[0]
        return thickness

    def to_absolute(self, point):
        return int(point[0] + (self.x_resolution/2)), int(point[1] + (self.y_resolution/2))

    def distance(self, relative_joint_position):
        rjp = relative_joint_position
        return math.sqrt(rjp.x**2 + rjp.y**2 + rjp.z**2)

    def transform_joint(self, joint):
        return self.tfBuffer.lookup_transform(self.frame, joint, rospy.Time(0)).transform.translation

    def calculate_point_angle(self, relative_joint_position):
        y_rotation = math.atan2(float(relative_joint_position.x), float(relative_joint_position.z))
        x_rotation = math.atan2(float(relative_joint_position.y), float(relative_joint_position.z))
        # Finde why its off by factor 2
        return x_rotation, y_rotation

    def image_position_from_angle(self, angle):
        scalar_x = float(self.x_resolution)/self.horizontal_fov
        scalar_y = float(self.y_resolution)/self.vertical_fov
        return int(math.degrees(angle[1]) * scalar_x), int(math.degrees(angle[0]) * scalar_y)


if __name__ == "__main__":
    #rospy.init_node('body_mask')
    body_mask_ball_candidate_filter((360, 640))