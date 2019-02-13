#!/usr/bin/env python2.7
import rospy
from humanoid_league_msgs.msg import LineInformationInImage, ObstaclesInImage, BallsInImage
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import yaml
import os

class Evaluation(object):
    def __init__(self):
        self.received_message = False  # boolean signaling whether a message of the type was received


class ImageMeasurement(object):
    def __init__(self):
        self.time_measurements = dict()
        self.ball_evaluation = Evaluation()
        self.line_evaluation = Evaluation()
        self.obstacle_evaluation = Evaluation()

    def get_max_duration(self):
        # returns the maximal duration a measurement in the image took
        if self.time_measurements.values():
            return max(self.time_measurements.values())
        else:
            return None


class Evaluator(object):
    def __init__(self):
        rospy.init_node("vision_evaluator")

        self._ball_sub = None
        if rospy.get_param("listen_balls", False):
            rospy.loginfo('listening for balls in image...')
            self._ball_sub = rospy.Subscriber(rospy.get_param("balls_topic", "balls_in_image"),
                 BallsInImage,
                 self._balls_callback(),
                 queue_size=1,
                 tcp_nodelay=True)

        self._line_sub = None
        if rospy.get_param("listen_lines", False):
            rospy.loginfo('listening for lines in image...')
            self._line_sub = rospy.Subscriber(rospy.get_param("lines_topic", "lines_in_image"),
                 LineInformationInImage,
                 self._lines_callback(),
                 queue_size=1,
                 tcp_nodelay=True)

        self._obstacle_sub = None
        if rospy.get_param("listen_obstacle", False):
            rospy.loginfo('listening for obstacles in image...')
            self._line_sub = rospy.Subscriber(rospy.get_param("obstacles_topic", "obstacles_in_image"),
                 ObstaclesInImage,
                 self._obstacles_callback(),
                 queue_size=1,
                 tcp_nodelay=True)

        self._image_pub = rospy.Publisher('image_raw', Image, queue_size=1)

        self._image_path = '~/images'

        # read label YAML file
        self._label_filename = 'labels.yaml'
        self._images = self._read_labels(self._label_filename)

        # initialize resend timer
        self._resend_timer = rospy.Timer(rospy.Duration(2), self._resend_callback) # 2 second timer TODO: make this a variable

        self.bridge = CvBridge()

        self._send_image_counter = 0  # represents the image index of the image to be sent in the list defined by the label yaml file
        self._current_image_counter = 0  # represents the current image index in the list defined by the label yaml file

        self._measurements = dict()

        rospy.spin()

    def _resend_callback(self, event):
        self._send_image(self._get_send_image_name())
        pass

    def _get_send_image_name(self):
        return self._images[self._send_image_counter]['name']

    def _get_current_labels(self):
        return self._images[self._current_image_counter]['annotations']

    def _update_image_counter(self, seq):
        # updates the image counter to publish a new image when necessary
        # (it was not updated already by an other callback)
        # TODO: do loop stuff here!
        if self._send_image_counter <= seq:
            self._send_image_counter += 1

    def _send_image(self, name):
        imgpath = os.path.join(self._image_path, name)
        image = cv2.imread(imgpath)
        if image is None:
            rospy.logwarn('Could not open image {} at path {}'.format(name, self._image_path))
            return
        msg = self.bridge.cv2_to_imgmsg(image)
        msg.header.stamp = rospy.get_rostime()
        msg.header.seq = self._send_image_counter
        self._image_pub.publish(msg)
        self._current_image_counter = self._send_image_counter  # update the current image counter to the new current image

        # set up evaluation element in measurements list
        self._measurements[self._send_image_counter] = ImageMeasurement()

    def _read_labels(self, filename):
        # reads the labels YAML file and returns a list of image names with their labels
        filepath = os.path.join(self._image_path, filename)
        images = None
        with open(filepath, 'r') as stream:
            try:
                images = yaml.load(stream)['labels']
            except yaml.YAMLError as exc:
                rospy.logerr(exc)
        return images

    def _balls_callback(self, msg):
        # measure duration of processing
        self._measure_timing(msg.header, 'balls')

    def _obstacles_callback(self, msg):
        # measure duration of processing
        self._measure_timing(msg.header, 'obstacles')

    def _goalpost_callback(self, msg):
        # measure duration of processing
        self._measure_timing(msg.header, 'goalposts')

    def _lines_callback(self, msg):
        # measure duration of processing
        self._measure_timing(msg.header, 'lines')

    def _measure_timing(self, header, category):
        # calculating and saving the time the processing took for the category
        self._measurements[header.seq].time_measurements[category] = rospy.get_rostime() - header.stamp


    @staticmethod
    def _analyze_labels(images):
        # analyzes the label file for stuff
        # honestly, i am so sorry for this!
        # TODO: this should be done with dicts...

        no_ball_count = 0
        no_line_count = 0
        no_obstacle_count = 0
        no_goalpost_count = 0
        remove_list = list()  # image names of images which need to be removed
        for image in images:
            ball_label = False
            line_label = False
            obstacle_label = False
            goalpost_label = False
            ball_in = None
            line_in = None
            obstacle_in = None
            goalpost_in = None
            for annotation in image['annotations']:
                if annotation['type'] == 'ball':
                    ball_label = True
                    if ball_in == True:
                        if not annotation['in']:  # contradiction!
                            rospy.logwarn('Found contradicting labels in image \"{}\"! The image will be removed!'.format(image['name']))
                            remove_list.append(image['name'])
                            break
                    elif ball_in == False:
                        if annotation['in']:  # contradiction!
                            rospy.logwarn('Found contradicting labels in image \"{}\"! The image will be removed!'.format(image['name']))
                            remove_list.append(image['name'])
                            break
                    else:  # it is None and therefor not set yet
                        ball_in = annotation['in']

                elif annotation['type'] == 'line':
                    line_label = True
                    if line_in == True:
                        if not annotation['in']:  # contradiction!
                            rospy.logwarn('Found contradicting labels in image \"{}\"! The image will be removed!'.format(image['name']))
                            remove_list.append(image['name'])
                            break
                    elif line_in == False:
                        if annotation['in']:  # contradiction!
                            rospy.logwarn('Found contradicting labels in image \"{}\"! The image will be removed!'.format(image['name']))
                            remove_list.append(image['name'])
                            break
                    else:  # it is None and therefor not set yet
                        line_in = annotation['in']

                elif annotation['type'] == 'obstacle':
                    obstacle_label = True
                    if obstacle_in == True:
                        if not annotation['in']:  # contradiction!
                            rospy.logwarn('Found contradicting labels in image \"{}\"! The image will be removed!'.format(image['name']))
                            remove_list.append(image['name'])
                            break
                    elif obstacle_in == False:
                        if annotation['in']:  # contradiction!
                            rospy.logwarn('Found contradicting labels in image \"{}\"! The image will be removed!'.format(image['name']))
                            remove_list.append(image['name'])
                            break
                    else:  # it is None and therefor not set yet
                        obstacle_in = annotation['in']

                elif annotation['type'] == 'goalpost':
                    goalpost_label = True
                    if goalpost_in == True:
                        if not annotation['in']:  # contradiction!
                            rospy.logwarn('Found contradicting labels in image \"{}\"! The image will be removed!'.format(image['name']))
                            remove_list.append(image['name'])
                            break
                    elif goalpost_in == False:
                        if annotation['in']:  # contradiction!
                            rospy.logwarn('Found contradicting labels in image \"{}\"! The image will be removed!'.format(image['name']))
                            remove_list.append(image['name'])
                            break
                    else:  # it is None and therefor not set yet
                        goalpost_in = annotation['in']

                else:
                    # an unknown label type... should we do something?
                    pass

            # increase the counters when no label was found for a type
            if not ball_label:
                no_ball_count += 1
            if not line_label:
                no_line_count += 1
            if not obstacle_label:
                no_obstacle_count += 1
            if not goalpost_label:
                no_goalpost_count += 1




if __name__ == "__main__":
    Evaluator()