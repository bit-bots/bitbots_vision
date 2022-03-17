#! /usr/bin/env python3
from typing import Dict, Union, List, Tuple

import os
import cv2
import rclpy
from rclpy import logging, time
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from copy import deepcopy
from cv_bridge import CvBridge
from threading import Thread, Lock
from sensor_msgs.msg import Image
from humanoid_league_msgs.msg import LineInformationInImage, Audio
from soccer_vision_msgs.msg import BallArray, FieldBoundary, GoalpostArray, RobotArray, Robot
from bitbots_vision.vision_modules import lines, field_boundary, color, debug, \
    obstacle, yolo_handler, ros_utils, candidate

from bitbots_vision.vision_modules import yoeo_handler

from abc import ABC, abstractmethod

from .yoeo_params import gen

logger = logging.get_logger('bitbots_vision')

try:
    from profilehooks import profile, \
        timecall  # Profilehooks profiles certain functions in you add the @profile or @timecall decorator.
except ImportError:
    profile = lambda x: x
    logger.info("No Profiling avalabile")


class DebugImageColors:
    # BGR
    ball = (0, 255, 0)  # green
    team_mates = (153, 51, 255)  # magenta
    opponents = (255, 255, 102)  # cyan
    misc_obstacles = (160, 160, 160)  # grey
    goalposts = (255, 255, 255)  # white
    field_boundary = (0, 0, 255)  # red
    field_boundary_convex = (0, 255, 255)  # yellow
    lines = (255, 0, 0)  # blue


# laueft!
class DebugImageFactory:
    _debug_image: Union[None, debug.DebugImage] = None
    _config: Dict = {}

    @classmethod
    def create(cls, config: Dict):
        if cls._new_debug_image_has_to_be_created(config):
            cls._create_new_debug_image(config)
        return cls._debug_image

    @classmethod
    def _new_debug_image_has_to_be_created(cls, config: Dict) -> bool:
        return cls._debug_image is None \
               or ros_utils.config_param_change(cls._config, config, 'vision_publish_debug_image')

    @classmethod
    def _create_new_debug_image(cls, config: Dict) -> None:
        cls._debug_image = debug.DebugImage(config['vision_publish_debug_image'])
        cls._config = config


# laeuft!
class YOEOFieldBoundaryDetectorFactory:
    _field_boundary_detector: Union[None, field_boundary.FieldBoundaryDetector] = None
    _field_boundary_detector_search_method: Union[None, str] = None
    _yoeo_id: Union[None, int] = None

    @classmethod
    def create(cls, config: Dict, yoeo: yoeo_handler.IYOEOHandler):
        if cls._new_field_boundary_detector_has_to_be_created(config, yoeo):
            cls._create_new_field_boundary_detector(config, yoeo)
        return cls._field_boundary_detector

    @classmethod
    def _new_field_boundary_detector_has_to_be_created(cls, config: Dict, yoeo: yoeo_handler.IYOEOHandler) -> bool:
        return cls._field_boundary_detector is None \
               or cls._field_boundary_detector_search_method != config['field_boundary_detector_search_method'] \
               or cls._yoeo_id != id(yoeo)

    @classmethod
    def _create_new_field_boundary_detector(cls, config: Dict, yoeo: yoeo_handler.IYOEOHandler) \
            -> field_boundary.FieldBoundaryDetector:
        field_boundary_detector_class = field_boundary.FieldBoundaryDetector.get_by_name(
            config['field_boundary_detector_search_method'])
        field_detector = yoeo_handler.YOEOFieldSegmentation(yoeo)

        cls._field_boundary_detector = field_boundary_detector_class(config, field_detector)
        cls._field_boundary_detector_search_method = config['field_boundary_detector_search_method']
        cls._yoeo_id = id(yoeo)


# laueft!
class YOEOObstacleDetectorFactory:
    _config: Dict = {}
    _robot_detector = None
    _yoeo_id: Union[None, int] = None
    _red_color_detector: Union[None, color.HsvSpaceColorDetector] = None
    _blue_color_detector: Union[None, color.HsvSpaceColorDetector] = None

    @classmethod
    def create(cls, config, yoeo, color: Union[None, str] = None, subtractors=None) -> obstacle.ColorObstacleDetector:
        if cls._new_robot_detector_has_to_be_created(yoeo):
            cls._create_new_robot_detector(yoeo)

        if cls._new_red_color_detector_has_to_be_created(config):
            cls._create_new_red_color_detector(config)

        if cls._new_blue_color_detector_has_to_be_created(config):
            cls._create_new_blue_color_detector(config)

        color_detector = cls._select_color_detector_based_on(color)

        return obstacle.ColorObstacleDetector(
            cls._robot_detector,
            color_detector,
            threshold=config['obstacle_color_threshold'],
            subtractors=subtractors)

    @classmethod
    def _new_robot_detector_has_to_be_created(cls, yoeo) -> bool:
        return cls._robot_detector is None or id(yoeo) != cls._yoeo_id

    @classmethod
    def _create_new_robot_detector(cls, yoeo) -> None:
        cls._robot_detector = yoeo_handler.YOEORobotDetector(yoeo)
        cls._yoeo_id = id(yoeo)

    @classmethod
    def _new_red_color_detector_has_to_be_created(cls, config: Dict) -> bool:
        return ros_utils.config_param_change(cls._config, config, r'^red_color_detector_')

    @classmethod
    def _create_new_red_color_detector(cls, config):
        cls._red_color_detector = color.HsvSpaceColorDetector(config, "red")

    @classmethod
    def _new_blue_color_detector_has_to_be_created(cls, config: Dict) -> bool:
        return ros_utils.config_param_change(cls._config, config, r'^blue_color_detector_')

    @classmethod
    def _create_new_blue_color_detector(cls, config):
        cls._blue_color_detector = color.HsvSpaceColorDetector(config, "blue")

    @classmethod
    def _select_color_detector_based_on(cls, color: Union[None, str]):
        if color == "blue":
            color_detector = cls._blue_color_detector
        elif color == "red":
            color_detector = cls._red_color_detector
        else:
            color_detector = None
        return color_detector


class IVisionComponent(ABC):
    @abstractmethod
    def configure(self, config: Dict, yoeo: yoeo_handler.IYOEOHandler) -> None:
        ...

    @abstractmethod
    def run(self, image_msg) -> None:
        ...

    @abstractmethod
    def set_image(self, image) -> None:
        ...


class CameraCapCheckComponent(IVisionComponent):
    def __init__(self, node: Node):
        self._camera_cap_brightness_threshold: int = 0
        self._config: Dict = {}
        self._image = None
        self._first_image_callback: bool = True
        self._node: Node = node
        self._publisher: Union[None, rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, yoeo: yoeo_handler.IYOEOHandler) -> None:
        self._camera_cap_brightness_threshold = config['vision_blind_threshold']

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(self._node,
                                                               self._config,
                                                               new_config,
                                                               self._publisher,
                                                               'ROS_audio_msg_topic',
                                                               Audio,
                                                               queue_size=10)

    def run(self, image_msg) -> None:
        if self._component_has_not_run_yet():
            self._set_component_has_run()
            self._run_component()

    def _component_has_not_run_yet(self) -> bool:
        return self._first_image_callback

    def _set_component_has_run(self) -> None:
        self._first_image_callback = False

    def _run_component(self) -> None:
        if self._camera_cap_could_be_on():
            self._log_error()
            self._issue_oral_warning()

    def _camera_cap_could_be_on(self) -> bool:
        mean_image_brightness = self._calculate_mean_image_brightness()
        return mean_image_brightness < self._camera_cap_brightness_threshold

    def _calculate_mean_image_brightness(self) -> float:
        return sum(cv2.mean(self._image))

    @staticmethod
    def _log_error() -> None:
        logger.error("Image is too dark! Camera cap not removed?")

    def _issue_oral_warning(self) -> None:
        ros_utils.speak("Hey!   Remove my camera cap!", self._publisher)

    def set_image(self, image) -> None:
        logger.info(f"Image type: {image.shape}")  # numpy nd.array
        if self._component_has_not_run_yet():
            self._image = image
        else:
            self._image = None  # to allow for garbage collection of first image


# laueft!
class YOEOBallDetectionComponent(IVisionComponent):
    def __init__(self, node: Node):
        self._config: Dict = {}
        self._ball_detector: Union[None, yoeo_handler.YOEODetectorTemplate] = None
        self._debug_image: Union[None, debug.DebugImage] = None
        self._field_boundary_detector: Union[None, field_boundary.FieldBoundaryDetector] = None
        self._node: Node = node
        self._publisher: Union[None, rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, yoeo: yoeo_handler.IYOEOHandler) -> None:
        self._ball_detector = yoeo_handler.YOEOBallDetector(yoeo)
        self._debug_image = DebugImageFactory.create(config)
        self._field_boundary_detector = YOEOFieldBoundaryDetectorFactory.create(config, yoeo)

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(self._node,
                                                               self._config,
                                                               new_config,
                                                               self._publisher,
                                                               'ROS_ball_msg_topic',
                                                               BallArray)

    def run(self, image_msg) -> None:
        best_candidates = self._get_best_ball_candidates()
        balls_within_convex_field_boundary = self._get_best_candidates_within_convex_field_boundary(best_candidates)
        final_candidates = self._apply_candidate_threshold_to(balls_within_convex_field_boundary)

        ball_messages = self._create_ball_messages(final_candidates)
        balls_message = self._create_balls_message(image_msg, ball_messages)
        self._publish_balls_message(balls_message)

        self._add_best_ball_candidates_to_debug_image(best_candidates)
        self._add_best_ball_candidates_within_convex_field_boundary_to_debug_image(balls_within_convex_field_boundary)
        self._add_top_ball_candidates_to_debug_image(final_candidates)

    def _get_best_ball_candidates(self):
        return self._ball_detector.get_top_candidates(count=self._config['ball_candidate_max_count'])

    def _get_best_candidates_within_convex_field_boundary(self, best_candidates):
        return self._field_boundary_detector.candidates_under_convex_field_boundary(best_candidates,
                                                                                    self._config[
                                                                                        'ball_candidate_field_boundary_y_offset'])

    def _apply_candidate_threshold_to(self, best_candidates):
        return candidate.Candidate.rating_threshold(best_candidates, self._config['ball_candidate_rating_threshold'])

    @staticmethod
    def _create_ball_messages(ball_candidates):
        return list(map(ros_utils.build_ball_msg, ball_candidates))

    @staticmethod
    def _create_balls_message(image_msg, ball_messages):
        return ros_utils.build_balls_msg(image_msg.header, ball_messages)

    def _publish_balls_message(self, balls_message) -> None:
        if balls_message:
            self._publisher.publish(balls_message)

    def _add_best_ball_candidates_to_debug_image(self, best_candidates) -> None:
        self._debug_image.draw_ball_candidates(best_candidates, DebugImageColors.ball, thickness=1)

    def _add_best_ball_candidates_within_convex_field_boundary_to_debug_image(self, best_candidates) -> None:
        self._debug_image.draw_ball_candidates(best_candidates, DebugImageColors.ball, thickness=2)

    def _add_top_ball_candidates_to_debug_image(self, top_candidates) -> None:
        self._debug_image.draw_ball_candidates(top_candidates, DebugImageColors.ball, thickness=3)

    def set_image(self, image) -> None:
        self._field_boundary_detector.set_image(image)


# laueft!
class YOEOGoalpostDetectionComponent(IVisionComponent):
    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Union[None, debug.DebugImage] = None
        self._field_boundary_detector: Union[None, field_boundary.FieldBoundaryDetector] = None
        self._goalpost_detector: Union[None, yoeo_handler.IYOEOSegmentation] = None
        self._node: Node = node
        self._publisher: Union[None, rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, yoeo: yoeo_handler.IYOEOHandler) -> None:
        self._debug_image = DebugImageFactory.create(config)
        self._field_boundary_detector = YOEOFieldBoundaryDetectorFactory.create(config, yoeo)
        self._goalpost_detector = yoeo_handler.YOEOGoalpostDetector(yoeo)

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(self._node,
                                                               self._config,
                                                               new_config,
                                                               self._publisher,
                                                               'ROS_goal_posts_msg_topic',
                                                               GoalpostArray,
                                                               queue_size=3)

    def run(self, image_msg) -> None:
        goalposts = self._get_best_candidates_within_convex_field_boundary(self._goalpost_detector.get_candidates(),
                                                                           self._config[
                                                                               'goal_post_field_boundary_y_offset'])
        goalpost_messages = self._create_goalpost_messages(goalposts)
        goalposts_message = self._create_goalposts_message(image_msg, goalpost_messages)
        self._publish_goalposts_message(goalposts_message)

        self._draw_all_goalpost_candidates_to_debug_image(self._goalpost_detector.get_candidates())
        self._draw_goalposts_within_convex_field_boundary_to_debug_image(goalposts)

    def _get_best_candidates_within_convex_field_boundary(self, best_candidates, candidate_y_offset):
        return self._field_boundary_detector.candidates_under_convex_field_boundary(best_candidates, candidate_y_offset)

    @staticmethod
    def _create_goalpost_messages(goalposts):
        return ros_utils.build_goal_post_msgs(goalposts)

    @staticmethod
    def _create_goalposts_message(image_msg, goalpost_messages):
        return ros_utils.build_goal_post_array_msg(image_msg.header, goalpost_messages)

    def _publish_goalposts_message(self, goalposts_message) -> None:
        if goalposts_message:
            self._publisher.publish(goalposts_message)

    def _draw_all_goalpost_candidates_to_debug_image(self, goalpost_candidates) -> None:
        self._debug_image.draw_obstacle_candidates(goalpost_candidates, DebugImageColors.goalposts, thickness=1)

    def _draw_goalposts_within_convex_field_boundary_to_debug_image(self, goalpost_candidates) -> None:
        self._debug_image.draw_obstacle_candidates(goalpost_candidates, DebugImageColors.goalposts, thickness=3)

    def set_image(self, image) -> None:
        self._field_boundary_detector.set_image(image)


# laueft!
class YOEOFieldBoundaryDetectionComponent(IVisionComponent):
    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Union[None, debug.DebugImage] = None
        self._field_boundary_detector: Union[None, field_boundary.FieldBoundaryDetector] = None
        self._node: Node = node
        self._publisher: Union[None, rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, yoeo: yoeo_handler.IYOEOHandler) -> None:
        self._debug_image = DebugImageFactory.create(config)
        self._field_boundary_detector = YOEOFieldBoundaryDetectorFactory.create(config, yoeo)

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(self._node,
                                                               self._config,
                                                               new_config,
                                                               self._publisher,
                                                               'ROS_field_boundary_msg_topic',
                                                               FieldBoundary)

    def run(self, image_msg) -> None:
        convex_field_boundary = self._field_boundary_detector.get_convex_field_boundary_points()
        field_boundary_msg = self._create_field_boundary_msg(image_msg, convex_field_boundary)
        self._publish_field_boundary_msg(field_boundary_msg)

        self._add_field_boundary_to_debug_image()
        self._add_convex_field_boundary_to_debug_image(convex_field_boundary)

    def _add_field_boundary_to_debug_image(self) -> None:
        field_boundary = self._field_boundary_detector.get_field_boundary_points()
        self._debug_image.draw_field_boundary(field_boundary, DebugImageColors.field_boundary)

    def _add_convex_field_boundary_to_debug_image(self, convex_field_boundary) -> None:
        self._debug_image.draw_field_boundary(convex_field_boundary, DebugImageColors.field_boundary_convex)

    @staticmethod
    def _create_field_boundary_msg(image_msg, field_boundary):
        return ros_utils.build_field_boundary_polygon_msg(image_msg.header, field_boundary)

    def _publish_field_boundary_msg(self, field_boundary_msg) -> None:
        self._publisher.publish(field_boundary_msg)

    def set_image(self, image) -> None:
        self._field_boundary_detector.set_image(image)


# laeuft!
class YOEOLineDetectionComponent(IVisionComponent):
    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Union[None, debug.DebugImage] = None
        self._line_detector: Union[None, yoeo_handler.IYOEOSegmentation] = None
        self._node: Node = node
        self._publisher: Union[None, rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, yoeo: yoeo_handler.IYOEOHandler) -> None:
        self._debug_image = DebugImageFactory.create(config)
        self._line_detector = yoeo_handler.YOEOLineSegmentation(yoeo)

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(self._node,
                                                               self._config,
                                                               new_config,
                                                               self._publisher,
                                                               'ROS_line_mask_msg_topic', Image)

    def run(self, image_msg) -> None:
        line_mask = self._line_detector.get_mask_image()
        self._add_line_mask_to_debug_image(line_mask)

        line_mask_message = self._create_line_mask_msg(image_msg, line_mask)
        self._publish_line_mask_msg(line_mask_message)

    def _add_line_mask_to_debug_image(self, line_mask) -> None:
        self._debug_image.draw_mask(line_mask, color=DebugImageColors.lines) #, opacity=0.8)

    @staticmethod
    def _create_line_mask_msg(image_msg, line_mask):  # -> sensor_msgs.msg._image.Image:
        return ros_utils.build_image_msg(image_msg.header, line_mask, '8UC1')

    def _publish_line_mask_msg(self, line_mask_msg) -> None:
        self._publisher.publish(line_mask_msg)

    def set_image(self, image) -> None:
        pass  # Intentional


# does not crash!
class YOEOFieldDetectionComponent(IVisionComponent):
    def __init__(self, node: Node):
        self._active: bool = False
        self._config: Dict = {}
        self._field_detector: Union[None, yoeo_handler.IYOEOSegmentation] = None
        self._node: Node = node
        self._publisher: Union[None, rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, yoeo: yoeo_handler.IYOEOHandler) -> None:
        self._active = config['vision_publish_field_mask_image']
        self._log_status(config)
        self._field_detector = yoeo_handler.YOEOFieldSegmentation(yoeo)

        self._register_publisher(config)
        self._config = config

    def _log_status(self, config: Dict):
        if ros_utils.config_param_change(self._config, config, 'vision_publish_field_mask_image'):
            if self._active:
                logger.info('Field mask WILL BE published')
            else:
                logger.info('Field mask WILL NOT BE published')

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(self._node,
                                                               self._config,
                                                               new_config,
                                                               self._publisher,
                                                               'ROS_field_mask_image_msg_topic',
                                                               Image)

    def run(self, image_msg) -> None:
        if self._active:
            field_mask = self._field_detector.get_mask_image()
            field_mask_msg = self._create_field_mask_msg(image_msg, field_mask)
            self._publish_field_mask_msg(field_mask_msg)

    @staticmethod
    def _create_field_mask_msg(image_msg, field_mask):
        return ros_utils.build_image_msg(image_msg.header, field_mask, '8UC1')

    def _publish_field_mask_msg(self, field_mask_msg) -> None:
        self._publisher.publish(field_mask_msg)

    def set_image(self, image) -> None:
        pass  # Intentional


# laeuft!
class YOEOObstacleDetectionComponent(IVisionComponent):
    def __init__(self, node: Node):
        self._config: Dict = {}

        self._misc_obstacles_detector: Union[None, obstacle.ColorObstacleDetector] = None
        self._opponents_detector: Union[None, obstacle.ColorObstacleDetector] = None
        self._team_mates_detector: Union[None, obstacle.ColorObstacleDetector] = None

        self._debug_image: Union[None, debug.DebugImage] = None

        self._node: Node = node
        self._publisher: Union[None, rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, yoeo: yoeo_handler.IYOEOHandler) -> None:  # TODO IYOEODetector

        own_color, opponent_color = self._determine_team_colors(config)
        self._team_mates_detector = YOEOObstacleDetectorFactory.create(config=config,
                                                                       yoeo=yoeo,
                                                                       color=own_color,
                                                                       subtractors=None)
        self._opponents_detector = YOEOObstacleDetectorFactory.create(config=config,
                                                                      yoeo=yoeo,
                                                                      color=opponent_color,
                                                                      subtractors=[self._team_mates_detector])
        self._misc_obstacles_detector = YOEOObstacleDetectorFactory.create(config=config,
                                                                           yoeo=yoeo,
                                                                           color=None,
                                                                           subtractors=[self._team_mates_detector,
                                                                                        self._opponents_detector])
        self._debug_image = DebugImageFactory.create(config)
        self._register_publisher(config)
        self._config = config

    @staticmethod
    def _determine_team_colors(config: Dict) -> Tuple[str, str]:
        if config['obstacle_own_team_color'] == 'blue':
            own_color = "blue"
            opponent_color = "red"
        else:
            own_color = "red"
            opponent_color = "blue"
        return own_color, opponent_color

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(self._node,
                                                               self._config,
                                                               new_config,
                                                               self._publisher,
                                                               'ROS_obstacle_msg_topic',
                                                               RobotArray,
                                                               queue_size=3)

    def run(self, image_msg) -> None:
        list_of_obstacle_messages: List = []
        self._add_team_mates_to(list_of_obstacle_messages)
        self._add_opponents_to(list_of_obstacle_messages)
        self._add_remaining_obstacles_to(list_of_obstacle_messages)

        obstacles_message = self._create_obstacles_message(image_msg, list_of_obstacle_messages)
        self._publish_obstacles_message(obstacles_message)

        self._add_obstacles_to_debug_image()

    def _add_team_mates_to(self, list_of_obstacle_messages: List) -> None:  # TODO return type
        team_mate_candidates = self._get_team_mate_candidates()
        team_mate_candidate_messages = self._create_obstacle_messages(Robot.TEAM_OWN, team_mate_candidates)
        list_of_obstacle_messages.extend(team_mate_candidate_messages)

    def _get_team_mate_candidates(self):
        return self._team_mates_detector.get_candidates()

    @staticmethod
    def _create_obstacle_messages(obstacle_type: Robot, candidates):
        return ros_utils.build_obstacle_msgs(obstacle_type, candidates)

    def _add_opponents_to(self, list_of_obstacle_messages: List) -> None:
        opponent_candidates = self._get_opponent_candidates()
        opponent_candidate_messages = self._create_obstacle_messages(Robot.TEAM_OPPONENT, opponent_candidates)
        list_of_obstacle_messages.extend(opponent_candidate_messages)

    def _get_opponent_candidates(self):
        return self._opponents_detector.get_candidates()

    def _add_remaining_obstacles_to(self, list_of_obstacle_messages: List) -> None:
        remaining_candidates = self._get_remaining_candidates()
        remaining_candidate_messages = self._create_obstacle_messages(Robot.TEAM_UNKNOWN, remaining_candidates)
        list_of_obstacle_messages.extend(remaining_candidate_messages)

    def _get_remaining_candidates(self):
        return self._misc_obstacles_detector.get_candidates()

    @staticmethod
    def _create_obstacles_message(image_msg, obstacle_messages):
        return ros_utils.build_obstacle_array_msg(image_msg.header, obstacle_messages)

    def _publish_obstacles_message(self, obstacles_message) -> None:
        self._publisher.publish(obstacles_message)

    def _add_obstacles_to_debug_image(self):
        self._add_team_mates_to_debug_image()
        self._add_opponents_to_debug_image()
        self._add_remaining_objects_to_debug_image()

    def _add_team_mates_to_debug_image(self) -> None:
        team_mate_candidates = self._get_team_mate_candidates()
        self._debug_image.draw_obstacle_candidates(team_mate_candidates, DebugImageColors.team_mates, thickness=3)

    def _add_opponents_to_debug_image(self) -> None:
        opponent_candidates = self._get_opponent_candidates()
        self._debug_image.draw_obstacle_candidates(opponent_candidates, DebugImageColors.opponents, thickness=3)

    def _add_remaining_objects_to_debug_image(self) -> None:
        remaining_candidates = self._get_remaining_candidates()
        self._debug_image.draw_obstacle_candidates(remaining_candidates, DebugImageColors.misc_obstacles,
                                                   thickness=3)

    def set_image(self, image) -> None:
        self._team_mates_detector.set_image(image)
        self._opponents_detector.set_image(image)
        self._misc_obstacles_detector.set_image(image)


# laeuft!
class DebugImageComponent(IVisionComponent):
    def __init__(self, node):
        self._config: Dict = {}
        self._node: Node = node
        self._debug_image: Union[None, debug.DebugImage] = None
        self._publisher: Union[None, rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, yoeo: yoeo_handler.IYOEOHandler) -> None:
        self._debug_image = DebugImageFactory.create(config)
        self._log_status(config)

        self._register_publisher(config)
        self._config = config

    def _log_status(self, config: Dict) -> None:
        if ros_utils.config_param_change(self._config, config, 'vision_publish_debug_image'):
            if config['vision_publish_debug_image']:
                logger.info('Debug images are enabled')
            else:
                logger.info('Debug images are disabled')

    def _register_publisher(self, config) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config, config,
            self._publisher,
            'ROS_debug_image_msg_topic',
            Image)

    def run(self, image_msg) -> None:
        debug_image_message = self._create_debug_image_message(image_msg)
        self._publish_debug_image_msg(debug_image_message)

    def _create_debug_image_message(self, image_msg):
        return ros_utils.build_image_msg(image_msg.header, self._debug_image.get_image(), 'bgr8')

    def _publish_debug_image_msg(self, debug_image_msg) -> None:
        self._publisher.publish(debug_image_msg)

    def set_image(self, image) -> None:
        self._debug_image.set_image(image)


class YOEOVision(Node):
    """
    The Vision is the main ROS-node for handling all tasks related to image processing.

    This class defines the whole image processing pipeline, which uses the modules from the `vision_modules`.
    It also handles the dynamic reconfiguration of the bitbots_vision.
    """

    def __init__(self):
        super().__init__('bitbots_vision')

        self._package_path = get_package_share_directory('bitbots_vision')

        logger.info('Initializing vision...')

        self._cv_bridge = CvBridge()

        self._config: Dict = {}

        self._sub_image = None

        # Reconfigure data transfer variable
        self._transfer_reconfigure_data = None
        self._transfer_reconfigure_data_mutex = Lock()

        # Image transfer variable
        self._transfer_image_msg = None
        self._transfer_image_msg_mutex = Lock()

        self._yoeo_handler: Union[None, yoeo_handler.IYOEOHandler] = None
        self._vision_components: Union[None, List[IVisionComponent]] = None

        # Setup reconfiguration
        gen.declare_params(self)
        self.add_on_set_parameters_callback(self._dynamic_reconfigure_callback)

        # Add general params
        ros_utils.set_general_parameters(["caching"])

        self._dynamic_reconfigure_callback(self.get_parameters_by_prefix("").values())

        logger.debug("Constructor ran")

    def _dynamic_reconfigure_callback(self, params) -> SetParametersResult:
        """
        Callback for the dynamic reconfigure configuration.

        :param params: new config # TODO
        """
        logger.info(f"type params: {type(params)}")
        new_config = self._get_updated_config_with(params)
        self._configure_vision(new_config)
        self._config = new_config

        return SetParametersResult(successful=True)

    def _get_updated_config_with(self, params) -> Dict:
        new_config = deepcopy(self._config)
        for param in params:
            new_config[param.name] = param.value
        return new_config

    def _configure_vision(self, new_config: Dict) -> None:

        # TODO
        # Check if the yoeo ball/goalpost detector is activated and if the non tpu version is used
        if new_config['neural_network_type'] in ['yoeo_pytorch']:
            if ros_utils.config_param_change(self._config, new_config,
                                             ['yolo_darknet_model_path', 'neural_network_type']):
                model_path = self._get_model_path(new_config)
                if self._model_file_exists(model_path):
                    # Decide which yolo implementation should be used
                    # if new_config['neural_network_type'] == 'yolo_opencv':
                    #     # Load OpenCV implementation (uses OpenCL)
                    #     self._yoeo = yolo_handler.YoloHandlerOpenCV(new_config, absolute_model_path)
                    # elif new_config['neural_network_type'] == 'yolo_darknet':
                    #     # Load Darknet implementation (uses CUDA)
                    #     self._yoeo = yolo_handler.YoloHandlerDarknet(new_config, absolute_model_path)
                    if new_config['neural_network_type'] == 'yoeo_pytorch':
                        self._yoeo_handler = yoeo_handler.YOEOHandlerPytorch(new_config, model_path)
                    logger.info(new_config['neural_network_type'] + " vision is running now")
                else:
                    logger.error('The specified yoeo model file does not exist!')

            # For other changes only modify the config
            elif ros_utils.config_param_change(self._config, new_config, r'yoeo_'):
                self._yoeo_handler.set_config(new_config)

        # TODO
        # # Check if  tpu version of yolo ball/goalpost detector is used
        # if new_config['neural_network_type'] in ['yolo_ncs2']:
        #     if ros_utils.config_param_change(self._config, new_config, ['neural_network_type', 'yolo_openvino_model_path']):
        #         # Build absolute model path
        #         yolo_openvino_model_path = os.path.join(self._package_path, 'models', new_config['yolo_openvino_model_path'])
        #         # Check if it exists
        #         if not os.path.exists(os.path.join(yolo_openvino_model_path, "yolo.bin")) \
        #                 or not os.path.exists(os.path.join(yolo_openvino_model_path, "yolo.xml")):
        #             logger.error('The specified yolo openvino model file doesn\'t exist!')
        #         else:
        #             self._yoeo = yolo_handler.YoloHandlerNCS2(new_config, yolo_openvino_model_path)
        #             logger.info(new_config['neural_network_type'] + " vision is running now")
        #     # For other changes only modify the config
        #     elif ros_utils.config_param_change(self._config, new_config, r'yolo_'):
        #         self._yoeo.set_config(new_config)
        # until here

        self._vision_components = []

        if new_config["component_camera_cap_check_active"]:
            self._vision_components.append(CameraCapCheckComponent(self))
        if new_config["component_ball_detection_active"]:
            self._vision_components.append(YOEOBallDetectionComponent(self))
        if new_config["component_obstacle_detection_active"]:
            self._vision_components.append(YOEOObstacleDetectionComponent(self))
        if new_config["component_goalpost_detection_active"]:
            self._vision_components.append(YOEOGoalpostDetectionComponent(self))
        if new_config["component_line_detection_active"]:
            self._vision_components.append(YOEOLineDetectionComponent(self))
        if new_config["component_field_boundary_detection_active"]:
            self._vision_components.append(YOEOFieldBoundaryDetectionComponent(self))
        if new_config["component_field_detection_active"]:
            self._vision_components.append(YOEOFieldDetectionComponent(self))
        if new_config["component_debug_image_active"]:
            self._vision_components.append(DebugImageComponent(self))

        for vision_component in self._vision_components:
            vision_component.configure(new_config, self._yoeo_handler)

        self._register_subscribers(new_config)
        logger.debug("End Configure_vision")

    @staticmethod
    def _model_file_exists(model_path: str) -> bool:
        return os.path.exists(os.path.join(model_path, "yoeo.pth"))

    def _get_model_path(self, config: Dict) -> str:
        return os.path.join(self._package_path, 'models', config['yoeo_model_path'])

    def _register_subscribers(self, config: Dict) -> None:
        self._sub_image = ros_utils.create_or_update_subscriber(self, self._config, config, self._sub_image,
                                                                'ROS_img_msg_topic', Image,
                                                                callback=self._image_callback,
                                                                queue_size=config['ROS_img_msg_queue_size'])

    def _image_callback(self, image_msg) -> None:
        # TODO: type
        """
        This method is called by the Image-message subscriber.
        Too old Image-Messages are dropped.

        Sometimes the queue gets too large, even if the size is limited to 1.
        That is why we drop old images manually.
        """
        logger.debug("_image_callback called")
        if self._image_is_too_old(image_msg) or self._transfer_image_msg_mutex_is_locked():
            return

        with self._transfer_image_msg_mutex:
            self._handle_image(image_msg)

    def _image_is_too_old(self, image_msg) -> bool:
        image_age = self.get_clock().now() - time.Time.from_msg(image_msg.header.stamp)
        if 1.0 < image_age.nanoseconds / 1000000000 < 1000.0:
            logger.warning(
                f"Vision: Dropping incoming Image-Message because it is too old! ({image_age.to_sec()} sec)")
            return True
        else:
            return False

    def _transfer_image_msg_mutex_is_locked(self) -> bool:
        return self._transfer_image_msg_mutex.locked()

    @profile
    def _handle_image(self, image_msg):
        """
        Run the vision pipeline
        """

        image = self._convert_image_msg_to_cv2_image(image_msg)

        if image is None:
            logger.debug("Image content is None :(")
            return

        self._yoeo_handler.set_image(image)
        self._publish_image_to_components(image)

        self._yoeo_handler.predict()
        self._run_components(image_msg)

    def _convert_image_msg_to_cv2_image(self, image_msg):
        return self._cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')

    def _publish_image_to_components(self, image):
        for vision_component in self._vision_components:
            vision_component.set_image(image)

    def _run_components(self, image_msg) -> None:
        for vision_component in self._vision_components:
            vision_component.run(image_msg)


def main(args=None):
    rclpy.init(args=args)
    logging.set_logger_level("bitbots_vision", logging.LoggingSeverity.DEBUG)
    node = YOEOVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()