#! /usr/bin/env python2
# TODO: remove

# TODO: docs
# TODO: set resolution on transformer stuff
# TODO: publish debug image as overlay of body mask with 0.5 opacity

import cv2
import math
import rospy
import numpy as np
from bitbots_vision.vision_modules import horizon, debug, ball, candidate

class body_mask_ball_candidate_filter(object):
    """
    TODO
    """
    def __init__(self, image_size):
        # TODO: debug printer
        # type: (DebugPrinter, (int, int), dict) -> None
        """
        TODO
        """
        self.image_size = image_size
        # TODO dyn reconf
        self.max_intersection_threshold = 0.2

    def get_body_parts(self):
        # type: () -> [(int, int), (int, int), int]
        """
        TODO
        """
        pass

    def get_body_mask(self, body_parts):
        # type: ([(int, int), (int, int), int]) -> TODO
        """
        TODO
        """
        # Generate white canvas
        mask = np.ones(self.image_size, dtype=np.uint8) * 255
        # Draw black lines for the parts of the own body
        for body_part in body_parts:
            cv2.line(mask, body_part[0], body_part[1], (0,0,0), thickness=body_part[2])
        return mask
    
    def get_ball_mask(self, ball_candidate):
        # type: () -> TODO
        """
        TODO
        """
        center = ball_candidate.get_center_point()
        radius = ball_candidate.get_radius()
        # Generates a black canvas
        canvas = np.zeros(self.image_size, dtype=np.uint8)
        # Draw white circle representing a ball
        mask = cv2.circle(canvas, center, radius, (255, 255, 255), thickness=-1)
        return mask

    def ball_candidate_not_on_own_body(self, ball_candidate, body_mask):
        # type: () -> bool
        """
        TODO
        """
        # Area of ball candidate in number of pixels
        ball_area = math.pi * ball_candidate.get_radius() ** 2

        ball_mask = self.get_ball_mask(ball_candidate)

        # Number of pixels intersecting both masks
        intersection_area = cv2.countNonZero(
            cv2.bitwise_and(ball_mask, body_mask))

        # True, if percentile of intersecting area 
        return (intersection_area / ball_area) <= self.max_intersection_threshold

    def get_ball_candidates_not_on_own_body(self, ball_candidates):
        # type: (TODO) -> TODO
        """
        TODO
        """
        body_parts = self.get_body_parts
        body_mask = self.get_body_mask(body_parts)
        return [ball_candidate for ball_candidate in ball_candidates if self.ball_candidate_not_on_own_body(
            ball_candidate,
            body_mask)]

# TODO: remove
if __name__ == '__main__':
    body_mask_ball_candidate_filter((360, 640))
