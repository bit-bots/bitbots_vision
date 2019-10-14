import os
import re
import rospy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from dynamic_reconfigure.encoding import Config as DynamicReconfigureConfig
from humanoid_league_msgs.msg import BallInImage, BallsInImage, LineInformationInImage, LineSegmentInImage, ObstaclesInImage, \
    ObstacleInImage, GoalPartsInImage, GoalPostInImage, GoalInImage, FieldBoundaryInImage, Speak, ImageWithRegionOfInterest
from bitbots_msgs.msg import Config

"""
This module provides some methods needed for the ros environment,
e.g. methods to convert candidates to ros msgs or methods to modify the dynamic reconfigure objects.
"""

_cv_bridge = CvBridge()

def _change_enum_items(cfg_type, parameter_name, new_items, default=None):
    """
    Take an enum-typed parameter in the given autogenerated cfg type and reset its possible values to new_items.
    You can then start a dynamic_reconfigure server which advertises the changed enum domain.
    To achieve this, you need to call this function before creating the server.
    Implementation is based on the gist: https://gist.github.com/peci1/912549b79fd6e8801023

    :param type cfg_type: One of the autogenerated config types (package.cfg.*Config).
    :param basestring parameter_name: Name of the enum parameter to change.
    :param new_items: The items that will form the new domain of the enum.
    :type new_items: array of {'name': ..., 'value': ..., 'description': ...}
    :param any default: If provided, this value is used as the default. If not, the first value in new_items is used.
    :raises RuntimeError: If there is no valid enum parameter with the given name in the given type.
    """

    # Check dynamic reconfigure type
    if not hasattr(cfg_type, 'config_description') or not hasattr(cfg_type, 'defaults'):
        raise RuntimeError('Type %s is not a valid dynamic reconfigure type.' % str(cfg_type))

    # Recursively searches all groups for specific param and adds the enum for this param
    if not _recursive_search(cfg_type, cfg_type.config_description['groups'], parameter_name, new_items, default):
        # Raise exception if param is not in config
        raise RuntimeError('Parameter "%s" not found.' % parameter_name)

def _recursive_search(cfg_type, groups, parameter_name, new_items, default):
    """
    Recursively searches all groups. If the parameter_name exists set the new_items.

    :param type cfg_type: One of the autogenerated config types (package.cfg.*Config).
    :param basestring parameter_name: Name of the enum parameter to change.
    :param new_items: The items that will form the new domain of the enum.
    :type new_items: array of {'name': ..., 'value': ..., 'description': ...}
    :param any default: If provided, this value is used as the default. If not, the first value in new_items is used.
    """
    # Iterate over all groups
    for group in groups:
        # Set param enum if it is in this group
        if _process_group(cfg_type, group, parameter_name, new_items, default):
            # Notify that item has been set
            return True
        # Search in the subgroups
        if _recursive_search(cfg_type, group['groups'], parameter_name, new_items, default):
            # Notify that item has been set
            return True

def _process_group(cfg_type, group, parameter_name, new_items, default):
    """
    Searches in the group for the parameter_name to set new_items.

    :param cfg_type: One of the autogenerated config types (package.cfg.*Config).
    :param package_path: ROS package path
    """
    # Iterate over all params
    for param_desc in group['parameters']:
        # Check if param matches
        if param_desc['name'] == parameter_name:
            # Check if enum generation method exists
            if param_desc['edit_method'] == '':
                raise RuntimeError('Type %s has empty edit_method, which means it is not a proper enum.' % str(cfg_type))

            # Execute this method
            edit_method = eval(param_desc['edit_method'])
            enum = edit_method['enum']

            # Check if a dummy enum exists
            if len(enum) == 0:
                raise RuntimeError('Type %s edit_method has empty enum, which means it is not a proper enum.' %
                                str(cfg_type))

            # we will copy all "unnecesarry" info (line number, filename etc.) from the first item
            sample_item = enum[0]

            new_enum = []
            # Add all items
            for item in new_items:
                new_item = sample_item.copy()
                new_item['name'] = item['name']
                new_item['value'] = item['value']
                new_item['description'] = item['description']
                new_enum.append(new_item)

            edit_method['enum'] = new_enum
            param_desc['edit_method'] = repr(edit_method)

            # if no default value was explicitly specified, use the first value
            if default is None:
                default = new_enum[0]['value']

            # Handle default
            param_desc['default'] = default
            cfg_type.defaults[param_desc['name']] = default

            # Return true if param has been set
            return True
    return False

def add_model_enums(cfg_type, package_path):
    """
    Add models to dynamic reconfigure enums.

    :param cfg_type: One of the autogenerated config types (package.cfg.*Config).
    :param package_path: ROS package path
    """
    # Directory in which the models are saved
    models_directory = os.path.join(package_path, "models")

    # All models
    model_names = os.listdir(models_directory)

    fcnn_paths = []
    yolo_paths = []

    # Sort models alphabetically
    model_names.sort()

    # Iterate over every model directory
    for folder in model_names:
        # Is this model an fcnn model
        if os.path.exists(os.path.join(models_directory, folder, "model_final.index")):
            # Append list with a new enum item
            fcnn_paths.append({
                'name': folder,
                'value': folder,
                'description': 'fcnn {}'.format(folder)})
        # Is this model an yolo model
        elif os.path.exists(os.path.join(models_directory, folder, "yolo_weights.weights")):
            # Append list with a new enum item
            yolo_paths.append({
                'name': folder,
                'value': folder,
                'description': 'yolo {}'.format(folder)})
        else:
            rospy.logwarn("Directory '{}' contains unknown model type. Please remove all non model directories from the 'models' directory!".format(folder), logger_name="vision_ros_utils")
    # Add enums to configuration
    _change_enum_items(cfg_type, 'fcnn_model_path', fcnn_paths)
    _change_enum_items(cfg_type, 'yolo_model_path', yolo_paths)

def add_color_space_enum(cfg_type, package_path):
    """
    Add models to dynamic reconfigure enums.

    :param cfg_type: One of the autogenerated config types (package.cfg.*Config).
    :param package_path: ROS package path
    """
    # Directory in which the color spaces are saved
    color_spaces_path = os.path.join(package_path, "config/color_spaces")
    # All color spaces
    color_spaces = os.listdir(color_spaces_path)
    # Sort color spaces alphabetically
    color_spaces.sort()
    # Get all files in this directory
    color_space_files = [file for file in color_spaces if os.path.isfile(os.path.join(color_spaces_path, file))]
    # Create list with a new enum item for each file
    field_color_space_enum = [{'name': cs_file, 'value': cs_file, 'description': 'color space {}'.format(cs_file)} for cs_file in color_space_files]

    # Add enums to configuration
    _change_enum_items(cfg_type, 'field_color_detector_path', field_color_space_enum)

def create_or_update_publisher(old_config, new_config, publisher_object, topic_key, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=1):
    """
    Creates or updates a publisher

    :param old_config: Previous config dict
    :param new_config: Current config dict
    :param publisher_object: The python object, that represents the publisher
    :param topic_key: The name of the topic variable in the config dict
    :param data_class: Data type class for ROS messages of the topic we want to subscribe
    :param subscriber_listener: Listener for subscription events
    :param tcp_nodelay: If True, this enables lower latency publishing at the cost of efficiency
    :param latch: If True, the last message, that has been published, will be sent to a new subscriber immediately
    :param headers: The ROS publisher headers
    :param queue_size: The ROS message queue size
    :return: adjusted publisher object
    """
    # Check if topic parameter has changed
    if config_param_change(old_config, new_config, topic_key):
        # Check if an publisher exists and unregister him
        if publisher_object is not None:
            publisher_object.unregister()
        # Create the new publisher
        publisher_object = rospy.Publisher(
            new_config[topic_key],
            data_class,
            subscriber_listener=subscriber_listener,
            tcp_nodelay=tcp_nodelay,
            latch=latch,
            headers=headers,
            queue_size=queue_size)
        rospy.logdebug("Registered new publisher to " + str(new_config[topic_key]), logger_name="vision_ros_utils")
    return publisher_object

def create_or_update_subscriber(old_config, new_config, subscriber_object, topic_key, data_class, callback=None, callback_args=None, queue_size=1, buff_size=65536, tcp_nodelay=False):
    """
    Creates or updates a subscriber

    :param old_config: Previous config dict
    :param new_config: Current config dict
    :param subscriber_object: The python object, that represents the subscriber
    :param topic_key: The name of the topic variable in the config dict
    :param data_class: Data type class for ROS messages of the topic we want to subscribe
    :param callback: The subscriber callback function
    :param callback_args: Additional arguments for the callback method
    :param queue_size: The ROS message queue size
    :param buff_size: The ROS message buffer size
    :param tcp_nodelay: If True, requests tcp_nodelay from publisher
    :return: adjusted subscriber object
    """
    # Check if topic parameter has changed
    if config_param_change(old_config, new_config, topic_key):
        # Check if an subsciber exists and unregister him
        if subscriber_object is not None:
            subscriber_object.unregister()
        # Create the new subscriber
        subscriber_object = rospy.Subscriber(
            new_config[topic_key],
            data_class,
            callback,
            callback_args=callback_args,
            queue_size=queue_size,
            buff_size=buff_size,
            tcp_nodelay=tcp_nodelay)
        rospy.logdebug("Registered new subscriber at " + str(new_config[topic_key]), logger_name="vision_ros_utils")
    return subscriber_object

def build_goal_parts_msg(header, goal_parts):
    """
    Builds a GoalPartsInImage message out of a list of GoalPostInImage messages

    :param header: ros header of the new message. Mostly the header of the image
    :param goal_parts: a list of goal part messages, e.g. GoalPostInImage
    :return: GoalPartsInImage message
    """
    # Create goalparts msg
    goal_parts_msg = GoalPartsInImage()
    # Add header
    goal_parts_msg.header.frame_id = header.frame_id
    goal_parts_msg.header.stamp = header.stamp
    # Add detected goal parts to the message
    goal_parts_msg.posts = goal_parts
    return goal_parts_msg

def build_goalpost_msgs(goalposts):
    """
    Builds a list of goalpost messages

    :param goalposts: goalpost candidates
    :return: list of goalposts msgs
    """
    # Create an empty list of goalposts
    message_list = []
    # Iterate over all goalpost candidates
    for goalpost in goalposts:
        # Create a empty post message
        post_msg = GoalPostInImage()
        post_msg.width = goalpost.get_width()
        if goalpost.get_rating() is not None:
            post_msg.confidence = goalpost.get_rating()
        post_msg.foot_point.x = goalpost.get_center_x()
        post_msg.foot_point.y = goalpost.get_lower_right_y()
        post_msg.top_point = post_msg.foot_point
        message_list.append(post_msg)
    return message_list

def build_goal_msg(goal_parts_msg):
    """
    Builds a goal message with a right and left post. If there is only one post in the image, the right and left post are the same.
    This should be reworked! The vision should only publish posts and e.g. the worldmodel builds a goal out of this context.

    :param top_ball_candidate: best rated ball candidate
    :return: ball msg
    """
    # Make new goal message
    goal_msg = GoalInImage()
    # Add header of the goal parts
    goal_msg.header = goal_parts_msg.header
    # Create goal posts at unrealistic high/low values
    left_post = GoalPostInImage()
    left_post.foot_point.x = 9999999999
    left_post.confidence = 1.0
    right_post = GoalPostInImage()
    right_post.foot_point.x = -9999999999
    right_post.confidence = 1.0

    # Set our posts
    for post in goal_parts_msg.posts:
        # Decide if its a left post
        if post.foot_point.x < left_post.foot_point.x:
            left_post = post
            left_post.confidence = post.confidence
        # Decide if its a right post
        if post.foot_point.x > right_post.foot_point.x:
            right_post = post
            right_post.confidence = post.confidence

    # Set posts in message
    goal_msg.left_post = left_post
    goal_msg.right_post = right_post
    goal_msg.confidence = 1.0
    # Return message if there are any posts
    if goal_parts_msg.posts:
        return goal_msg

def build_balls_msg(header, balls):
    """
    Builds a balls message out of a list of ball messages

    :param header: ros header of the new message. Mostly the header of the image
    :param balls: A list of BallInImage messages
    :return: balls msg
    """
    # create ball msg
    balls_msg = BallsInImage()
    # Set header
    balls_msg.header.frame_id = header.frame_id
    balls_msg.header.stamp = header.stamp
    # Add balls
    for ball in balls:
        balls_msg.candidates.append(ball)
    return balls_msg

def build_ball_msg(top_ball_candidate):
    """
    Builds a ball message

    :param top_ball_candidate: best rated ball candidate
    :return: ball msg
    """
    # Create a empty ball message
    ball_msg = BallInImage()
    ball_msg.center.x = top_ball_candidate.get_center_x()
    ball_msg.center.y = top_ball_candidate.get_center_y()
    ball_msg.diameter = top_ball_candidate.get_diameter()
    ball_msg.confidence = top_ball_candidate.get_rating()
    return ball_msg

def build_obstacles_msg(header, obstacles):
    """
    Builds a ObstaclesInImage message containing a list of obstacle messages

    :param header: ros header of the new message. Mostly the header of the image
    :param obstacles: a list of obstacle messages
    :return: ObstaclesInImage message
    """
    # Create obstacle msg
    obstacles_msg = ObstaclesInImage()
    # Add header
    obstacles_msg.header.frame_id = header.frame_id
    obstacles_msg.header.stamp = header.stamp
    # Add red obstacles
    obstacles_msg.obstacles = obstacles
    return obstacles_msg

def build_obstacle_msgs(obstacle_color, detections):
    """
    Builds a list of obstacles for a certain color

    :param obstacle_color: color of the obstacles
    :param detections: obstacle candidates
    :return: list of obstacle msgs
    """
    message_list = []
    for detected_obstacle in detections:
        obstacle_msg = ObstacleInImage()
        obstacle_msg.color = obstacle_color
        obstacle_msg.top_left.x = detected_obstacle.get_upper_left_x()
        obstacle_msg.top_left.y = detected_obstacle.get_upper_left_y()
        obstacle_msg.height = int(detected_obstacle.get_height())
        obstacle_msg.width = int(detected_obstacle.get_width())
        if detected_obstacle.get_rating() is not None:
            obstacle_msg.confidence = detected_obstacle.get_rating()
        obstacle_msg.playerNumber = 42
        message_list.append(obstacle_msg)
    return message_list

def build_field_boundary_msg(header, field_boundary):
    """
    Builds a FieldBoundaryInImage ROS message.

    :param header: ros header of the new message. Mostly the header of the image
    :param field_boundary: List of tuples containing the field boundary points.
    :return: FieldBoundaryInImage message
    """
    # Create message
    field_boundary_msg = FieldBoundaryInImage()
    # Add header
    field_boundary_msg.header = header
    # Add field boundary points
    for point in field_boundary:
        field_boundary_msg.field_boundary_points.append(Point(point[0], point[1], 0))
    return field_boundary_msg

def build_line_information_in_image_msg(header, line_segments):
    """
    Builds a LineInformationInImage that consists of line segments

    :param header: ros header of the new message. Mostly the header of the image
    :param line_segments: A list of LineSegmentInImage messages
    :return: Final LineInformationInImage message
    """
    # Create message
    line_msg = LineInformationInImage()
    # Set header values
    line_msg.header.frame_id = header.frame_id
    line_msg.header.stamp = header.stamp
    # Set line segments
    line_msg.segments = line_segments
    return line_msg

def build_image_msg(header, image, desired_encoding="passthrough"):
    """
    Builds a Image message

    :param header: ros header of the new message. Mostly the header of the incoming image
    :param image: A 2d NumPy UInt8 array
    :param desired_encoding: The Image type. E.g. 8UC[1-4], 8SC[1-4], 16UC[1-4], 16SC[1-4], 32SC[1-4], 32FC[1-4], 64FC[1-4]
    :return: The Image message
    """
    image_msg = _cv_bridge.cv2_to_imgmsg(image, desired_encoding)
    image_msg.header = header
    return image_msg

def convert_line_points_to_line_segment_msgs(line_points):
    """
    Converts a list of linepoints in the form [(x,y), ...] into a list of LineSegmentInImage messages

    :param line_points: A list of linepoints in the form [(x,y), ...]
    :return: A list of LineSegmentInImage messages
    """
    line_segments = []
    for line_point in line_points:
        # Create LineSegmentInImage message
        line_segment = LineSegmentInImage()
        line_segment.start.x = line_point[0]
        line_segment.start.y = line_point[1]
        line_segment.end = line_segment.start
        line_segments.append(line_segment)
    return line_segments

def speak(string, speech_publisher):
    """
    Sends a speak message and let the robot say the given string.

    :param string: Text the robot should say
    :param speech_publisher: ROS publisher for the speech message
    """
    speak_message = Speak()
    speak_message.text = string
    speech_publisher.publish(speak_message)

def config_param_change(old_config, new_config, params):
    # type: (dict, dict, [str]) -> bool
    """
    Checks whether some of the specified config params have changed.

    :param dict old_config: old config dict
    :param dict new_config: new config dict
    :param list of str or str params: regex discribing parameter name or list of parameter names
    :return bool: True if parameter has changed
    """
    # Make regex instead of list possible
    if not isinstance(params, list):
        # Build regex
        regex = re.compile(params)
        # Filter parameter names by regex
        params = list(filter(regex.search, list(new_config.keys())))
        # Check if parameters matching this regex exist
        if len(params) == 0:
            raise KeyError('Regex \'{}\' has no matches in dict.'.format(params))

    # Iterate over params
    for param in params:
        # Check if param exists in new config
        if param not in new_config:
            raise KeyError('\'{}\' not in dict.'.format(param))
        # Check if param is new or if param has changed
        elif param not in old_config or old_config[param] != new_config[param]:
            return True
    return False

def publish_vision_config(config, publisher):
    """
    Publishes the given config.

    :param config: A vision config
    :param publisher: The ROS publisher object
    """
    # Clean config dict to avoid not dumpable types
    config_cleaned = {}
    # Iterate over all config keys and values
    for key, value in config.items():
        # Check if the value is dumpable
        if not isinstance(value, DynamicReconfigureConfig):
            config_cleaned[key] = value
    # Create new config message
    msg = Config()
    # The message contains a string. So the config gets serialized and send as string
    msg.data = yaml.dump(config_cleaned)
    # Publish config
    publisher.publish(msg)

def build_fcnn_region_of_interest(fcnn_output, field_boundary_detector, header, offset):
    """
    Returns a region of interest with the fcnn heatmap under the field boundary.

    :param fcnn_output: the fcnn output
    :param field_boundary_detector: the field boundary detector
    :param header: ros header of the new message. Mostly the header of the image
    :param offset: an offset for the field boundary
    :return: fcnn heatmap under the field boundary
    """
    msg = ImageWithRegionOfInterest()
    msg.header.frame_id = header.frame_id
    msg.header.stamp = header.stamp
    field_boundary_top = field_boundary_detector.get_upper_bound(y_offset=offset)
    image_cropped = fcnn_output[field_boundary_top:]  # cut off at field_boundary
    msg.image = _cv_bridge.cv2_to_imgmsg(image_cropped, "mono8")
    msg.regionOfInterest.x_offset = 0
    msg.regionOfInterest.y_offset = field_boundary_top
    msg.regionOfInterest.height = fcnn_output.shape[0] - 1 - field_boundary_top
    msg.regionOfInterest.width = fcnn_output.shape[1] - 1
    return msg
