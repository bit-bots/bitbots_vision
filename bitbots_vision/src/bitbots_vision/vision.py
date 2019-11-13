#! /usr/bin/env python3

import os
import cv2
import rospy
import rospkg
import threading
import time
from copy import deepcopy
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from humanoid_league_msgs.msg import BallsInImage, LineInformationInImage, \
    ObstaclesInImage, ObstacleInImage, ImageWithRegionOfInterest, \
    GoalPartsInImage, FieldBoundaryInImage, Speak
from bitbots_vision.vision_modules import lines, field_boundary, color, debug, \
    fcnn_handler, live_fcnn_03, dummy_ballfinder, obstacle, yolo_handler, ros_utils
from bitbots_vision.cfg import VisionConfig
from bitbots_msgs.msg import Config, ColorSpace
from dynamic_color_space import DynamicColorSpace
try:
    from profilehooks import profile, timecall # Profilehooks profiles certain functions in you add the @profile or @timecall decorator.
except ImportError:
    rospy.loginfo("No Profiling avalabile", logger_name="vision")


class Vision:
    def __init__(self):
        # type () -> None
        """
        Vision is the main ROS-node for handling all tasks related to image processing.
        Initiating 'bitbots_vision' node.

        :return: None
        """
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path('bitbots_vision')

        rospy.init_node('bitbots_vision_auto_label')
        rospy.loginfo('Initializing vision...', logger_name="vision")

        self._cv_bridge = CvBridge()

        self._config = {}

        # Subscriber placeholder
        self._sub_image = None
        self._sub_dynamic_color_space_msg_topic = None

        # Register static publishers
        # Register publisher of 'vision_config'-messages
        # For changes of topic name: also change topic name in dynamic_color_space.py
        self._pub_config = rospy.Publisher(
            'vision_config',
            Config,
            queue_size=1,
            latch=True)

        # Needed for operations that should only be executed on the first image
        self._first_image_callback = True

        self._debug_drawer = debug.DebugImage()

        # Reconfigure dict transfer variable
        self._transfer_reconfigure_data = None
        self._transfer_reconfigure_data_read_flag = False

        # Image transfer variable
        self._transfer_image_msg = None
        self._transfer_image_msg_read_flag = False

        # Add model enums to _config
        ros_utils.add_model_enums(VisionConfig, self._package_path)
        ros_utils.add_color_space_enum(VisionConfig, self._package_path)

        self.dyn_color_space = DynamicColorSpace()

        # Register VisionConfig server (dynamic reconfigure) and set callback
        srv = Server(VisionConfig, self._dynamic_reconfigure_callback)

        # Run the vision main loop
        self._main()

    def _main(self):
        """
        Main loop that processes the images and configuration changes
        """
        # Copy _config from shared memory
        self._transfer_reconfigure_data_read_flag = True
        reconfigure_data = deepcopy(self._transfer_reconfigure_data)
        self._transfer_reconfigure_data_read_flag = False
        self._transfer_reconfigure_data = None
        # Run vision reconfiguration
        self._configure_vision(*reconfigure_data)

        folders = rospy.get_param("~folders")

        for folder in folders:
            print(folder)
            if rospy.is_shutdown():
                pass
            for image_file in sorted(os.listdir(folder)):
                if rospy.is_shutdown():
                    pass
                print(image_file)
                if image_file.endswith(".jpg") or image_file.endswith(".png"):
                    image = cv2.imread(os.path.join(folder, image_file))
                    if image is not None:
                        out_folder = folder[0:-1] + "_label"
                        debug_folder = folder[0:-1] + "_debug"
                        if not os.path.exists(out_folder):
                                os.makedirs(out_folder)
                        if not os.path.exists(debug_folder):
                                os.makedirs(debug_folder)
                        self._handle_image(image, os.path.join(out_folder, image_file), os.path.join(debug_folder, image_file))
                    else:
                        rospy.logwarn("Image not found!!!")
        #cv2.waitKey(0)

    def _dynamic_reconfigure_callback(self, config, level):
        """
        Callback for the dynamic reconfigure configuration.

        :param config: New _config
        :param level: The level is a definable int in the Vision.cfg file. All changed params are or ed together by dynamic reconfigure.
        """
        # Check flag
        while self._transfer_reconfigure_data_read_flag and not rospy.is_shutdown():
            time.sleep(0.01)
        # Set data
        self._transfer_reconfigure_data = (config, level)

        return config


    def _configure_vision(self, config, level):
        """
        Handle dynamic reconfigure configuration.

        :param config: New _config
        :param level: The level is a definable int in the Vision.cfg file. All changed params are or ed together by dynamic reconfigure.
        """

        # Set some thresholds
        self._use_dynamic_color_space = config['dynamic_color_space_active']

        # Which line type should we publish?
        self._use_line_points = config['line_detector_use_line_points']
        self._use_line_mask = config['line_detector_use_line_mask']

        # Set the white color detector
        if ros_utils.config_param_change(self._config, config, r'^white_color_detector_'):
            self._white_color_detector = color.HsvSpaceColorDetector(config, "white")

        # Set the red color detector
        if ros_utils.config_param_change(self._config, config, r'^red_color_detector_'):
            self._red_color_detector = color.HsvSpaceColorDetector(config, "red")

        # Set the blue color detector
        if ros_utils.config_param_change(self._config, config, r'^blue_color_detector_'):
            self._blue_color_detector = color.HsvSpaceColorDetector(config, "blue")

        # Check if params changed
        if ros_utils.config_param_change(self._config, config,
                r'^field_color_detector_|dynamic_color_space_'):
            # Check if the dynamic color space field color detector or the static field color detector should be used
            if self._use_dynamic_color_space:
                # Set dynamic color space field color detector
                self._field_color_detector = color.DynamicPixelListColorDetector(
                    config,
                    self._package_path)
            else:
                # Unregister old subscriber
                if self._sub_dynamic_color_space_msg_topic is not None:
                    self._sub_dynamic_color_space_msg_topic.unregister()
                # Set the static field color detector
                self._field_color_detector = color.PixelListColorDetector(
                    config,
                    self._package_path)

        # Get field boundary detector class by name from _config
        field_boundary_detector_class = field_boundary.FieldBoundaryDetector.get_by_name(
            config['field_boundary_detector_search_method'])

        # Set the field boundary detector
        self._field_boundary_detector = field_boundary_detector_class(
            config,
            self._field_color_detector)

        # Set the line detector
        self._line_detector = lines.LineDetector(
            config,
            self._white_color_detector,
            self._field_color_detector,
            self._field_boundary_detector)

        # Set the obstacle detector
        self._obstacle_detector = obstacle.ObstacleDetector(
            config,
            self._red_color_detector,
            self._blue_color_detector,
            self._white_color_detector,
            self._field_boundary_detector)

        # Set the other obstacle detectors
        self._red_obstacle_detector = obstacle.RedObstacleDetector(self._obstacle_detector)
        self._blue_obstacle_detector = obstacle.BlueObstacleDetector(self._obstacle_detector)
        self._unknown_obstacle_detector = obstacle.UnknownObstacleDetector(self._obstacle_detector)

        self._register_or_update_all_subscribers(config)

        # Define Modules that should run their calculations (modules should exist, theirfore its located here)
        self._conventional_modules = [
            self._obstacle_detector,
            self._line_detector,
        ]

        self.dyn_color_space.set_vision_config(config)

        # The old _config gets replaced with the new _config
        self._config = config

    def _register_or_update_all_subscribers(self, config):
        # type: (dict) -> None
        """
        This method registers all subscribers needed for the vision node.

        :param dict config: new, incoming _config
        :return: None
        """
        self._sub_image = ros_utils.create_or_update_subscriber(self._config, config, self._sub_image, 'ROS_img_msg_topic', Image, callback=self._image_callback, queue_size=config['ROS_img_msg_queue_size'], buff_size=60000000) # https://github.com/ros/ros_comm/issues/536

        if self._use_dynamic_color_space:
            self._sub_dynamic_color_space_msg_topic = ros_utils.create_or_update_subscriber(self._config, config, self._sub_dynamic_color_space_msg_topic, 'ROS_dynamic_color_space_msg_topic', ColorSpace, callback=self._field_color_detector.color_space_callback, queue_size=1, buff_size=2 ** 20)

    def _image_callback(self, image_msg):
        # type: (Image) -> None
        """
        This method is called by the Image-message subscriber.
        Old Image-messages were dropped.

        Sometimes the queue gets to large, even when the size is limited to 1.
        That's, why we drop old images manually.
        """
        # drops old images and cleans up queue. Still accepts very old images, that are most likely from ros bags.
        image_age = rospy.get_rostime() - image_msg.header.stamp
        if 1.0 < image_age.to_sec() < 1000.0:
            rospy.logwarn('Vision: Dropped incoming Image-message, because its too old! ({} sec)'.format(image_age.to_sec()),
                          logger_throttle=2, logger_name="")
            return

        # Check flag
        if self._transfer_image_msg_read_flag:
            return

        # Transfer the image to the main thread
        self._transfer_image_msg = image_msg

    def _handle_image(self, image, image_path, debug_path):
        """
        Runs the vision pipeline

        :param image: Image
        """
        # self.dyn_color_space.set_image(image)

        # Skip if image is None
        if image is None:
            rospy.logdebug("Image content is None :(", logger_name="vision")
            return

        # Instances that should be notified with the new image
        internal_image_subscribers =[
            self._field_color_detector,
            self._white_color_detector,
            self._red_color_detector,
            self._blue_color_detector,
            self._field_boundary_detector,
            self._obstacle_detector,
            self._red_obstacle_detector,
            self._blue_obstacle_detector,
            self._line_detector,
        ]

        # Distribute the image to the detectors
        # Iterate over subscribers
        for vision_object in internal_image_subscribers:
            # Send image
            vision_object.set_image(image)

        self._conventional_precalculation()

        if rospy.get_param("~field_boundary_mask") == "convex":
            mask = self._field_boundary_detector.get_convex_mask()
        elif rospy.get_param("~field_boundary_mask") == "normal":
            mask = self._field_boundary_detector.get_mask()

        cv2.imwrite(image_path[0:-4] + ".png", mask)

        print("Progressed image to " + os.path.join(image_path[0:-4] + ".png"))

        self._debug_drawer.set_image(image)

        self._debug_drawer.draw_mask(mask, (255,0,0), opacity=0.8)

        cv2.imwrite(debug_path[0:-4] + ".png", self._debug_drawer.get_image())

    def _conventional_precalculation(self):
        """
        Starts the conventional calculations
        """
        # Modules that should run their calculations
        # TODO: move this to DynReconf and add empty list to init
        self._conventional_modules = [
            self._field_color_detector,
            self._white_color_detector,
            self._red_color_detector,
            self._blue_color_detector,
            self._obstacle_detector,
            self._line_detector,
        ]
        # Run all modules
        for module in self._conventional_modules:
            module.compute()

if __name__ == '__main__':
    Vision()
