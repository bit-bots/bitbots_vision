#!/usr/bin/env python3
import rospy
from humanoid_league_msgs.msg import RegionOfInterestWithImage
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2


class Converter(object):
    def __init__(self):
        rospy.init_node("image_converter")

        rospy.Subscriber(rospy.get_param("fcnn_image_topic", "fcnn_image"),
                         RegionOfInterestWithImage,
                         self._callback_fcnn,
                         queue_size=1,
                         tcp_nodelay=True,
                         buff_size=60000000)
            # https://github.com/ros/ros_comm/issues/536

        self.image_pub = rospy.Publisher('converted_image', Image, queue_size=1)

        self.bridge = CvBridge()

        rospy.spin()

    def _callback_fcnn(self, msg):
        input_image = self.bridge.imgmsg_to_cv2(msg.image, 'bgr8')  # TODO: evaluate this!!!
        rospy.logdebug(input_image.shape, logger_name="image_converter")
        rospy.logdebug((int(msg.regionOfInterest.width) + 1, int(msg.regionOfInterest.height) + 1), logger_name="image_converter")
        input_image = cv2.resize(input_image, (int(msg.regionOfInterest.width) + 1, int(msg.regionOfInterest.height) + 1))

        output_image = np.zeros((msg.original_width, msg.original_height, 3), dtype=np.uint8)
        output_image[:,:,0] = 180  # everything is dark blue
        output_image[int(msg.regionOfInterest.y_offset):int(msg.regionOfInterest.y_offset + msg.regionOfInterest.height) + 1, int(msg.regionOfInterest.x_offset):int(msg.regionOfInterest.x_offset + msg.regionOfInterest.width) + 1] = input_image  # copying the image into the empty frame

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, 'bgr8'))

if __name__ == "__main__":
    Converter()
