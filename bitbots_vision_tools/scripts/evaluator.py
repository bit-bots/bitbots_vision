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
        self.pixel_mask_rates = None
        self.duration = None


class ImageMeasurement(object):
    def __init__(self, eval_classes):
        self.evaluations = dict()
        for eval_class in eval_classes:
            self.evaluations[eval_class] = Evaluation()


    def get_max_duration(self):
        # returns the maximal duration a measurement in the image took
        max_duration = None
        for eval in self.evaluations.values():
            if eval.duration is not None and eval.duration > max_duration:
                max_duration = eval.duration
        return max_duration


class Evaluator(object):
    def __init__(self):
        rospy.init_node("vision_evaluator")


        self._evaluated_classes = list()

        self._ball_sub = None
        if rospy.get_param("listen_balls", False):
            rospy.loginfo('listening for balls in image...')
            self._evaluated_classes.append('ball')
            self._ball_sub = rospy.Subscriber(rospy.get_param("balls_topic", "balls_in_image"),
                 BallsInImage,
                 self._balls_callback(),
                 queue_size=1,
                 tcp_nodelay=True)

        self._line_sub = None
        if rospy.get_param("listen_lines", False):
            rospy.loginfo('listening for lines in image...')
            self._evaluated_classes.append('line')
            self._line_sub = rospy.Subscriber(rospy.get_param("lines_topic", "lines_in_image"),
                 LineInformationInImage,
                 self._lines_callback(),
                 queue_size=1,
                 tcp_nodelay=True)

        self._obstacle_sub = None
        if rospy.get_param("listen_obstacle", False):
            rospy.loginfo('listening for obstacles in image...')
            self._evaluated_classes.append('obstacle')
            self._evaluated_classes.append('goalpost')
            self._evaluated_classes.append('robot')
            self._line_sub = rospy.Subscriber(rospy.get_param("obstacles_topic", "obstacles_in_image"),
                 ObstaclesInImage,
                 self._obstacles_callback(),
                 queue_size=1,
                 tcp_nodelay=True)

        self._image_pub = rospy.Publisher('image_raw', Image, queue_size=1)

        self._image_path = '~/images'

        self._line_thickness = 3

        # read label YAML file
        self._label_filename = 'labels.yaml'
        self._images = self._read_labels(self._label_filename)

        # initialize resend timer
        self._resend_timer = rospy.Timer(rospy.Duration(2), self._resend_callback) # 2 second timer TODO: make this a variable

        self.bridge = CvBridge()

        self._send_image_counter = 0  # represents the image index of the image to be sent in the list defined by the label yaml file
        self._current_image_counter = 0  # represents the current image index in the list defined by the label yaml file
        self._image_size = None  # tuple (height, width)

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

        if self._image_size is None:
            self._image_size = image.shape[:-1]

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

    def _generate_polygon_mask_from_vectors(self, vectors):
        mask = np.zeros(self._image_size)

        for vector in vectors:
            cv2.fillConvexPoly(mask, vector, 1.0)
        return mask

    def _generate_rectangle_mask_from_vectors(self, vectors):
        mask = np.zeros(self._image_size)

        for vector in vectors:
            cv2.rectangle(mask, vector[0], vector[1], 1.0, thickness=-1)
        return mask

    def _generate_circle_mask_from_vectors(self, vectors):
        mask = np.zeros(self._image_size)

        for vector in vectors:
            center = (vector[0][0] + (vector[1][0] - vector[0][0]) / 2, vector[0][1] + (vector[1][1] - vector[0][1]) / 2)
            radius = ((vector[1][0] - vector[0][0]) / 2 + (vector[1][1] - vector[0][1]) / 2) / 2
            cv2.circle(mask, center, radius, 1.0, thickness=-1)
        return mask

    def _generate_line_mask_from_vectors(self, vectors):
        mask = np.zeros(self._image_size)
        for vector in vectors:
            cv2.line(mask, vector[0], vector[1], 1.0, thickness=self._line_thickness)
        return mask

    def _generate_ball_mask_from_msg(self, msg):
        mask = np.zeros(self._image_size)
        for ball in msg.candidates:
            cv2.circle(mask, (int(round(ball.center.x)), int(round(ball.center.y))), int(round(ball.diameter/2)), 1.0, thickness=-1)
        return mask

    def _generate_obstacle_mask_from_msg(self, msg):
        vectors = list()
        for obstacle in msg.obstacles:
            vector = ((obstacle.top_left.x, obstacle.top_left.y), (obstacle.top_left.x + obstacle.width, obstacle.top_left.y + obstacle.height))
            vectors.append(vector)
        return self._generate_rectangle_mask_from_vectors(vectors)

    def _generate_line_mask_from_msg(self, msg):
        mask = np.zeros(self._image_size)
        for line in msg.segments:
            cv2.line(mask, (int(round(line.start.x)), int(round(line.start.y))), (int(round(line.end.x)), int(round(line.end.y))), 1.0, thickness=self._line_thickness)
        return mask

    @staticmethod
    def _match_masks(label_mask, detected_mask):
        # WARNING: the mask has to be filled with 0 and 1 es
        # matches the masks onto each other to determine multiple measurements.
        rates = dict()
        rates['tp'] = np.mean((np.bitwise_and(label_mask, detected_mask)))
        rates['tn'] = np.mean(np.bitwise_not(np.bitwise_or(label_mask, detected_mask)))
        rates['fp'] = np.mean(np.bitwise_and(detected_mask, np.bitwise_not(label_mask)))
        rates['fn'] = np.mean(np.bitwise_and(np.bitwise_not(detected_mask), label_mask))
        rates['lp'] = np.mean(label_mask)
        rates['ln'] = 1 - rates['lp']  # because all the other pixels have to be negative
        rates['dp'] = np.mean(detected_mask)
        rates['dn'] = 1 - rates['dp']  # because all the other pixels have to be negative
        return rates

    def _recieved_all_messages_for_image(self, image_seq):
        measurement = self._measurements[image_seq]
        for eval_class in self._evaluated_classes:
            if not measurement.evaluations[eval_class].received_message:
                return False
        return True

    @staticmethod
    def _filter_type(annotations, typename):
        # returns the annotations of type TYPE
        return [annotation for annotation in annotations if annotation['type'] == typename]

    @staticmethod
    def _extract_vectors_from_annotations(annotations, typename=None):
        # returns the vectors of annotations of type TYPE
        if typename:
            return [annotation['vector'] for annotation in annotations if annotation['type'] == typename]
        return [annotation['vector'] for annotation in annotations]

        # returns the annotations of type TYPE

    def _analyze_labels(self):
        # analyzes the label file for stuff

        not_in_image_count = dict()
        for eval_class in self._evaluated_classes:
            not_in_image_count[eval_class] = 0

        remove_list = list()  # image names of images which need to be removed
        for image in self._images:
            in_image = dict()
            found_label = dict()
            for eval_class in self._evaluated_classes:
                found_label[eval_class] = False
                in_image[eval_class] = None

            for annotation in image['annotations']:
                if annotation['type'] not in self._evaluated_classes:
                    continue  # ignore other classes annotations
                # annotation type is in evaluated classes
                found_label[annotation['type']] = True
                if in_image[annotation['type']] == True:
                    if not annotation['in']:  # contradiction!
                        rospy.logwarn('Found contradicting labels of type {} in image \"{}\"! The image will be removed!'.format(annotation['type'], image['name']))
                        remove_list.append(image['name'])
                        break
                elif in_image[annotation['type']]  == False:
                    if annotation['in']:  # contradiction!
                        rospy.logwarn('Found contradicting labels of type {} in image \"{}\"! The image will be removed!'.format(annotation['type'], image['name']))
                        remove_list.append(image['name'])
                        break
                else:  # it is None and therefor not set yet
                    in_image[annotation['type']]  = annotation['in']

            # increase the counters when no label was found for a type
            for eval_class in self._evaluated_classes:
                if not found_label[eval_class]:
                    not_in_image_count[eval_class] += 1




if __name__ == "__main__":
    Evaluator()
