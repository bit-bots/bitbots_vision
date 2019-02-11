#!/usr/bin/env python2.7
import rospy
from humanoid_league_msgs.msg import ImageWithRegionOfInterest
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import yaml
import os


class Evaluator(object):
    def __init__(self):
        rospy.init_node("vision_evaluator")

        rospy.Subscriber(rospy.get_param("fcnn_image_topic", "fcnn_image"),
                         ImageWithRegionOfInterest,
                         self._callback_fcnn,
                         queue_size=1,
                         tcp_nodelay=True,
                         buff_size=60000000)
            # https://github.com/ros/ros_comm/issues/536

        self._image_pub = rospy.Publisher('image_raw', Image, queue_size=1)

        self._image_path = '~/images'

        # read label YAML file
        self._label_filename = 'labels.yaml'
        self._images = self._read_labels(self._label_filename)

        # initialize resend timer
        self._resend_timer = rospy.Timer(rospy.Duration(2), self._resend_callback) # 2 second timer TODO: make this a variable

        self.bridge = CvBridge()

        self._image_counter = 0  # represents the current image index in the list defined by the label yaml file

        rospy.spin()

    def _callback_fcnn(self, msg):
        input_image = self.bridge.imgmsg_to_cv2(msg.image, 'bgr8')  # TODO: evaluate this!!!
        print(input_image.shape)
        print((int(msg.regionOfInterest.width) + 1, int(msg.regionOfInterest.height) + 1))
        input_image = cv2.resize(input_image, (int(msg.regionOfInterest.width) + 1, int(msg.regionOfInterest.height) + 1))

        output_image = np.zeros((self.initial_image_size[1], self.initial_image_size[0], 3), dtype=np.uint8)
        output_image[:,:,0] = 180  # everything is dark blue
        output_image[int(msg.regionOfInterest.y_offset):int(msg.regionOfInterest.y_offset + msg.regionOfInterest.height) + 1, int(msg.regionOfInterest.x_offset):int(msg.regionOfInterest.x_offset + msg.regionOfInterest.width) + 1] = input_image  # copying the image into the empty frame

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, 'bgr8'))

    def _resend_callback(self):
        pass

    def _get_current_image_name(self):
        return self._images[self._image_counter]['name']

    def _get_current_labels(self):
        return self._images[self._image_counter]['annotations']

    def _send_image(self, name):
        imgpath = os.path.join(self._image_path, name)
        image = cv2.imread(imgpath)
        if image is None:
            rospy.logwarn('Could not open image {} at path {}'.format(name, self.image_path))
            return
        msg = self.bridge.cv2_to_imgmsg(image)
        msg.header.stamp = rospy.get_rostime()
        self._image_pub.publish(msg)


    def _read_labels(self, filename):
        filepath = os.path.join(self._image_path, filename)

        with open(filepath, 'r') as stream:
            try:
                images = yaml.load(stream)['labels']
            except yaml.YAMLError as exc:
                rospy.logerr(exc)
        return images

if __name__ == "__main__":
    Evaluator()