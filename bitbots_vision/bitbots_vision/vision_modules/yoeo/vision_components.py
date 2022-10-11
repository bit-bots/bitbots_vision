from __future__ import annotations

import numpy as np
import rclpy
from abc import ABC, abstractmethod
from rclpy.node import Node
from sensor_msgs.msg import Image
from typing import Dict, Optional, List, Tuple
from humanoid_league_msgs.msg import Audio, GameState
from soccer_vision_2d_msgs.msg import BallArray, FieldBoundary, GoalpostArray, RobotArray, Robot
from bitbots_vision.vision_modules import field_boundary, color, debug, \
    obstacle, ros_utils, candidate

from . import detectors, object_manager, yoeo_handlers


logger = rclpy.logging.get_logger('yoeo_vision_components')


class DebugImageFactory:
    """
    Factory class that handles the lifetime of debug.DebugImage objects used in almost all YOEO vision components.
    """
    _config: Dict = {}
    _debug_image: Optional[debug.DebugImage] = None

    @classmethod
    def get(cls, config: Dict) -> debug.DebugImage:
        """
        Get an instance of debug.DebugImage.
        """
        if cls._new_debug_image_has_to_be_created(config):
            cls._create_new_debug_image(config)
        return cls._debug_image

    @classmethod
    def _new_debug_image_has_to_be_created(cls, config: Dict) -> bool:
        return cls._debug_image is None \
               or ros_utils.config_param_change(cls._config, config, 'component_debug_image_active')

    @classmethod
    def _create_new_debug_image(cls, config: Dict) -> None:
        cls._debug_image = debug.DebugImage(config['component_debug_image_active'])
        cls._config = config


class YOEOFieldBoundaryDetectorFactory:
    """
    Factory class that handles the lifetime of field_boundary.FieldBoundaryDetector objects used in the YOEO field
    boundary detection.
    """
    _field_boundary_detector: Optional[field_boundary.FieldBoundaryDetector] = None
    _field_boundary_detector_search_method: Optional[str] = None
    _yoeo_id: Optional[int] = None

    @classmethod
    def get(cls, config: Dict) -> field_boundary.FieldBoundaryDetector:
        """
        Get an instance of field_boundary.FieldBoundaryDetector.
        """
        if cls._new_field_boundary_detector_has_to_be_created(config):
            cls._create_new_field_boundary_detector(config)
        return cls._field_boundary_detector

    @classmethod
    def _new_field_boundary_detector_has_to_be_created(cls, config: Dict) -> bool:
        return cls._field_boundary_detector is None \
               or cls._field_boundary_detector_search_method != config['field_boundary_detector_search_method'] \
               or cls._yoeo_id != object_manager.YOEOObjectManager.get_id()

    @classmethod
    def _create_new_field_boundary_detector(cls, config: Dict) -> None:
        field_boundary_detector_class = field_boundary.FieldBoundaryDetector.get_by_name(
            config['field_boundary_detector_search_method']
        )
        field_detector = detectors.YOEOFieldSegmentation(object_manager.YOEOObjectManager.get())

        cls._field_boundary_detector = field_boundary_detector_class(config, field_detector)
        cls._field_boundary_detector_search_method = config['field_boundary_detector_search_method']
        cls._yoeo_id = object_manager.YOEOObjectManager.get_id()


class YOEOObstacleDetectorFactory:
    """
    Factory class that handles the lifetime of obstacle.ColorObstacleDetector objects used in the YOEO obstacle
    detection.
    """
    _config: Dict = {}
    _blue_color_detector: Optional[color.HsvSpaceColorDetector] = None
    _red_color_detector: Optional[color.HsvSpaceColorDetector] = None
    _robot_detector = None
    _yoeo_id: Optional[int] = None

    @classmethod
    def get(cls,
            config: Dict,
            color: Optional[int] = None,
            subtractors: Optional[List[obstacle.ColorObstacleDetector]] = None) \
            -> obstacle.ColorObstacleDetector:
        """
        Get an instance of obstacle.ColorObstacleDetector with the specified parameters.
        """
        if cls._new_robot_detector_has_to_be_created():
            cls._create_new_robot_detector()

        if cls._new_red_color_detector_has_to_be_created(config):
            cls._create_new_red_color_detector(config)

        if cls._new_blue_color_detector_has_to_be_created(config):
            cls._create_new_blue_color_detector(config)

        color_detector = cls._select_detector_based_on(color)

        return obstacle.ColorObstacleDetector(
            cls._robot_detector,
            color_detector,
            threshold=config['obstacle_color_threshold'],
            subtractors=subtractors)

    @classmethod
    def _new_robot_detector_has_to_be_created(cls) -> bool:
        return cls._robot_detector is None or cls._yoeo_id != object_manager.YOEOObjectManager.get_id()

    @classmethod
    def _create_new_robot_detector(cls) -> None:
        cls._robot_detector = detectors.YOEORobotDetector(object_manager.YOEOObjectManager.get())
        cls._yoeo_id = object_manager.YOEOObjectManager.get_id()

    @classmethod
    def _new_red_color_detector_has_to_be_created(cls, config: Dict) -> bool:
        return ros_utils.config_param_change(cls._config, config, r'^red_color_detector_')

    @classmethod
    def _create_new_red_color_detector(cls, config) -> None:
        cls._red_color_detector = color.HsvSpaceColorDetector(config, "red")

    @classmethod
    def _new_blue_color_detector_has_to_be_created(cls, config: Dict) -> bool:
        return ros_utils.config_param_change(cls._config, config, r'^blue_color_detector_')

    @classmethod
    def _create_new_blue_color_detector(cls, config) -> None:
        cls._blue_color_detector = color.HsvSpaceColorDetector(config, "blue")

    @classmethod
    def _select_detector_based_on(cls, color: Optional[int]):
        if color == GameState.BLUE:
            color_detector = cls._blue_color_detector
        elif color == GameState.RED:
            color_detector = cls._red_color_detector
        else:
            color_detector = None
        return color_detector


class IVisionComponent(ABC):
    """
    Interface for the YOEO vision components.
    """
    @abstractmethod
    def configure(self, config: Dict, debug_mode: bool) -> None:
        """
        This method is to be used to (re-) configure the vision component. It should be invoked once before running the
        component for the first time.
        """
        ...

    @abstractmethod
    def run(self, image_msg: Image) -> None:
        """
        This method runs the vision component.
        """
        ...

    @abstractmethod
    def set_image(self, image: np.ndarray) -> None:
        """
        This method is to be used to notify a component about a new image.
        """
        ...


class YOEOComponent(IVisionComponent):
    """
    This component runs the YOEO network. It is MANDATORY and must be run BEFORE any of the other components.
    """
    def __init__(self):
        self._yoeo_instance: Optional[yoeo_handlers.IYOEOHandler] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._yoeo_instance = object_manager.YOEOObjectManager.get()

    def run(self, image_msg: Image) -> None:
        self._yoeo_instance.predict()

    def set_image(self, image: np.ndarray) -> None:
        self._yoeo_instance.set_image(image)


class CameraCapCheckComponent(IVisionComponent):
    """
    This component checks if the camera cap could still be attached to the camera.
    Component deactivates itself after the first image.
    """

    def __init__(self, node: Node):
        self._camera_cap_brightness_threshold: int = 0
        self._config: Dict = {}
        self._image: Optional[np.ndarray] = None
        self._is_first_image: bool = True
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._camera_cap_brightness_threshold = config['vision_blind_threshold']

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_audio_msg_topic',
            Audio
        )

    def run(self, image_msg: Image) -> None:
        if self._component_has_not_run_yet():
            self._set_component_has_run()
            self._run_component()

    def _component_has_not_run_yet(self) -> bool:
        return self._is_first_image

    def _set_component_has_run(self) -> None:
        self._is_first_image = False

    def _run_component(self) -> None:
        if self._camera_cap_could_be_on():
            self._log_error()
            self._issue_oral_warning()

    def _camera_cap_could_be_on(self) -> bool:
        mean_image_brightness = self._image.mean()
        return mean_image_brightness < self._camera_cap_brightness_threshold

    @staticmethod
    def _log_error() -> None:
        logger.error("Image is too dark! Camera cap not removed?")

    def _issue_oral_warning(self) -> None:
        ros_utils.speak("Hey!   Remove my camera cap!", self._publisher)

    def set_image(self, image: np.ndarray) -> None:
        if self._component_has_not_run_yet():
            self._image = image
        else:
            self._image = None  # to allow for garbage collection of first image


class YOEOBallDetectionComponent(IVisionComponent):
    """
    This component carries out the ball detection using YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._ball_detector: Optional[detectors.YOEOBallDetector] = None
        self._debug_image: Optional[debug.DebugImage] = None
        self._debug_mode: bool = False
        self._field_boundary_detector: Optional[field_boundary.FieldBoundaryDetector] = None
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._ball_detector = detectors.YOEOBallDetector(object_manager.YOEOObjectManager.get())
        self._debug_image = DebugImageFactory.get(config)
        self._debug_mode = debug_mode
        self._field_boundary_detector = YOEOFieldBoundaryDetectorFactory.get(config)

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_ball_msg_topic',
            BallArray
        )

    def run(self, image_msg: Image) -> None:
        candidates = self._get_best_ball_candidates()
        candidates_within_convex_fb = self._filter_for_best_candidates_within_convex_field_boundary(candidates)
        final_candidates = self._filter_by_candidate_threshold(candidates_within_convex_fb)
        self._publish_balls_message(image_msg, final_candidates)

        if self._debug_mode:
            self._add_candidates_to_debug_image(candidates)
            self._add_candidates_within_convex_fb_to_debug_image(candidates_within_convex_fb)
            self._add_final_candidates_to_debug_image(final_candidates)

    def _get_best_ball_candidates(self) -> List[candidate.Candidate]:
        return self._ball_detector.get_top_candidates(count=self._config['ball_candidate_max_count'])

    def _filter_for_best_candidates_within_convex_field_boundary(self, candidates: List[candidate.Candidate]) \
            -> List[candidate.Candidate]:
        return self._field_boundary_detector.candidates_under_convex_field_boundary(
            candidates,
            self._config['ball_candidate_field_boundary_y_offset']
        )

    def _filter_by_candidate_threshold(self, candidates: List[candidate.Candidate]) \
            -> List[candidate.Candidate]:
        return candidate.Candidate.rating_threshold(candidates, self._config['ball_candidate_rating_threshold'])

    def _publish_balls_message(self, image_msg: Image, candidates: List[candidate.Candidate]) -> None:
        ball_messages = list(map(ros_utils.build_ball_msg, candidates))
        balls_message = ros_utils.build_ball_array_msg(image_msg.header, ball_messages)
        self._publisher.publish(balls_message)

    def _add_candidates_to_debug_image(self, candidates: List[candidate.Candidate]) -> None:
        self._debug_image.draw_ball_candidates(candidates, DebugImageComponent.Colors.ball, thickness=1)

    def _add_candidates_within_convex_fb_to_debug_image(self, candidates: List[candidate.Candidate]) -> None:
        self._debug_image.draw_ball_candidates(candidates, DebugImageComponent.Colors.ball, thickness=2)

    def _add_final_candidates_to_debug_image(self, candidates: List[candidate.Candidate]) -> None:
        self._debug_image.draw_ball_candidates(candidates, DebugImageComponent.Colors.ball, thickness=3)

    def set_image(self, image: np.ndarray) -> None:
        self._field_boundary_detector.set_image(image)


class YOEOGoalpostDetectionComponent(IVisionComponent):
    """
    This component carries out the goalpost detection using YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Optional[debug.DebugImage] = None
        self._debug_mode: bool = False
        self._field_boundary_detector: Optional[field_boundary.FieldBoundaryDetector] = None
        self._goalpost_detector: Optional[detectors.YOEOGoalpostDetector] = None
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._debug_image = DebugImageFactory.get(config)
        self._debug_mode = debug_mode
        self._field_boundary_detector = YOEOFieldBoundaryDetectorFactory.get(config)
        self._goalpost_detector = detectors.YOEOGoalpostDetector(object_manager.YOEOObjectManager.get())

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_goal_posts_msg_topic',
            GoalpostArray
        )

    def run(self, image_msg: Image) -> None:
        candidates = self._get_candidates()
        final_candidates = self._get_candidates_within_convex_field_boundary(candidates)
        self._publish_goalposts_message(image_msg, final_candidates)

        if self._debug_mode:
            self._add_candidates_to_debug_image(self._goalpost_detector.get_candidates())
            self._add_final_candidates_to_debug_image(final_candidates)

    def _get_candidates(self) -> List[candidate.Candidate]:
        return self._goalpost_detector.get_candidates()

    def _get_candidates_within_convex_field_boundary(self, candidates: List[candidate.Candidate]) \
            -> List[candidate.Candidate]:
        return self._field_boundary_detector.candidates_under_convex_field_boundary(
            candidates,
            self._config['goal_post_field_boundary_y_offset']
        )

    def _publish_goalposts_message(self, image_msg: Image, candidates: List[candidate.Candidate]) -> None:
        goalpost_messages = [ros_utils.build_goal_post_msg(candidate) for candidate in candidates]
        goalposts_message = ros_utils.build_goal_post_array_msg(image_msg.header, goalpost_messages)
        self._publisher.publish(goalposts_message)

    def _add_candidates_to_debug_image(self, candidates: List[candidate.Candidate]) -> None:
        self._debug_image.draw_obstacle_candidates(candidates, DebugImageComponent.Colors.goalposts, thickness=1)

    def _add_final_candidates_to_debug_image(self, candidates: List[candidate.Candidate]) -> None:
        self._debug_image.draw_obstacle_candidates(candidates, DebugImageComponent.Colors.goalposts, thickness=3)

    def set_image(self, image: np.ndarray) -> None:
        self._field_boundary_detector.set_image(image)


class YOEOFieldBoundaryDetectionComponent(IVisionComponent):
    """
    This component carries out the field boundary detection using YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Optional[debug.DebugImage] = None
        self._debug_mode: bool = False
        self._field_boundary_detector: Optional[field_boundary.FieldBoundaryDetector] = None
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._debug_image = DebugImageFactory.get(config)
        self._debug_mode = debug_mode
        self._field_boundary_detector = YOEOFieldBoundaryDetectorFactory.get(config)

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_field_boundary_msg_topic',
            FieldBoundary
        )

    def run(self, image_msg: Image) -> None:
        convex_field_boundary_points = self._get_convex_field_boundary_points()
        self._publish_field_boundary_msg(image_msg, convex_field_boundary_points)

        if self._debug_mode:
            self._add_field_boundary_to_debug_image()
            self._add_convex_field_boundary_to_debug_image(convex_field_boundary_points)

    def _get_convex_field_boundary_points(self) -> List[Tuple[int, int]]:
        return self._field_boundary_detector.get_convex_field_boundary_points()

    def _publish_field_boundary_msg(self, image_msg: Image, field_boundary_points: List[Tuple[int, int]]) -> None:
        field_boundary_msg = ros_utils.build_field_boundary_msg(image_msg.header, field_boundary_points)
        self._publisher.publish(field_boundary_msg)

    def _add_field_boundary_to_debug_image(self) -> None:
        field_boundary = self._field_boundary_detector.get_field_boundary_points()
        self._debug_image.draw_field_boundary(field_boundary, DebugImageComponent.Colors.field_boundary)

    def _add_convex_field_boundary_to_debug_image(self, convex_field_boundary_points: List[Tuple[int, int]]) -> None:
        self._debug_image.draw_field_boundary(
            convex_field_boundary_points,
            DebugImageComponent.Colors.field_boundary_convex
        )

    def set_image(self, image: np.ndarray) -> None:
        self._field_boundary_detector.set_image(image)


class YOEOLineDetectionComponent(IVisionComponent):
    """
    This component carries out the line detection using YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Optional[debug.DebugImage] = None
        self._debug_mode: bool = False
        self._line_detector: Optional[detectors.IYOEOSegmentation] = None
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._debug_image = DebugImageFactory.get(config)
        self._debug_mode = debug_mode
        self._line_detector = detectors.YOEOLineSegmentation(object_manager.YOEOObjectManager.get())

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_line_mask_msg_topic',
            Image
        )

    def run(self, image_msg: Image) -> None:
        line_mask = self._line_detector.get_mask_image()
        self._publish_line_mask_msg(image_msg, line_mask)

        if self._debug_mode:
            self._add_line_mask_to_debug_image(line_mask)

    def _publish_line_mask_msg(self, image_msg: Image, line_mask: np.ndarray) -> None:
        line_mask_msg = ros_utils.build_image_msg(image_msg.header, line_mask, '8UC1')
        self._publisher.publish(line_mask_msg)

    def _add_line_mask_to_debug_image(self, line_mask: np.ndarray) -> None:
        self._debug_image.draw_mask(line_mask, color=DebugImageComponent.Colors.lines)

    def set_image(self, image: np.ndarray) -> None:
        pass  # Nothing should happen here


class YOEOFieldDetectionComponent(IVisionComponent):
    """
    This component carries out the field detection using YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._field_detector: Optional[detectors.IYOEOSegmentation] = None
        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._field_detector = detectors.YOEOFieldSegmentation(object_manager.YOEOObjectManager.get())
        logger.info('Field mask WILL BE published')

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_field_mask_image_msg_topic',
            Image
        )

    def run(self, image_msg: Image) -> None:
        field_mask = self._get_field_mask()
        self._publish_field_mask_msg(image_msg, field_mask)

    def _get_field_mask(self) -> np.ndarray:
        return self._field_detector.get_mask_image()

    def _publish_field_mask_msg(self, image_msg: Image, field_mask: np.ndarray) -> None:
        field_mask_msg = ros_utils.build_image_msg(image_msg.header, field_mask, '8UC1')
        self._publisher.publish(field_mask_msg)

    def set_image(self, image: np.ndarray) -> None:
        pass  # Nothing should happen here


class YOEOObstacleDetectionComponent(IVisionComponent):
    """
    This component carries out the obstacle detection using YOEO.
    """

    def __init__(self, node: Node):
        self._config: Dict = {}
        self._debug_image: Optional[debug.DebugImage] = None
        self._debug_mode: bool = False

        self._misc_obstacles_detector: Optional[obstacle.ColorObstacleDetector] = None
        self._opponents_detector: Optional[obstacle.ColorObstacleDetector] = None
        self._team_mates_detector: Optional[obstacle.ColorObstacleDetector] = None

        self._node: Node = node
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        own_color, opponent_color = self._determine_team_colors()
        self._team_mates_detector = YOEOObstacleDetectorFactory.get(
            config=config,
            color=own_color,
            subtractors=None
        )
        self._opponents_detector = YOEOObstacleDetectorFactory.get(
            config=config,
            color=opponent_color,
            subtractors=[self._team_mates_detector]
        )
        self._misc_obstacles_detector = YOEOObstacleDetectorFactory.get(
            config=config,
            color=None,
            subtractors=[self._team_mates_detector,
                         self._opponents_detector]
        )
        self._debug_image = DebugImageFactory.get(config)
        self._debug_mode = debug_mode
        
        self._register_publisher(config)
        self._config = config

    @staticmethod
    def _determine_team_colors() -> Tuple[Optional[int], Optional[int]]:
        own_color = ros_utils.get_robot_color_for_team(Robot().attributes.TEAM_OWN)
        opponent_color = ros_utils.get_robot_color_for_team(Robot().attributes.TEAM_OPPONENT)
        return own_color, opponent_color

    def _register_publisher(self, new_config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config,
            new_config,
            self._publisher,
            'ROS_obstacle_msg_topic',
            RobotArray
        )

    def run(self, image_msg: Image) -> None:
        obstacle_msgs: List[Robot] = []
        self._add_team_mates_to(obstacle_msgs)
        self._add_opponents_to(obstacle_msgs)
        self._add_remaining_obstacles_to(obstacle_msgs)

        self._publish_obstacles_message(image_msg, obstacle_msgs)

        if self._debug_mode:
            self._add_obstacles_to_debug_image()

    def _add_team_mates_to(self, obstacle_msgs: List[Robot]) -> None:
        team_mate_candidates = self._team_mates_detector.get_candidates()
        team_mate_candidate_messages = self._create_obstacle_messages(Robot().attributes.TEAM_OWN, team_mate_candidates)
        obstacle_msgs.extend(team_mate_candidate_messages)

    @staticmethod
    def _create_obstacle_messages(obstacle_type: Robot, candidates: List[candidate.Candidate]) -> List[Robot]:
        return [ros_utils.build_robot_msg(obstacle_candidate, obstacle_type) for obstacle_candidate in candidates]

    def _add_opponents_to(self, obstacle_msgs: List[Robot]) -> None:
        opponent_candidates = self._opponents_detector.get_candidates()
        opponent_candidate_messages = self._create_obstacle_messages(
            Robot().attributes.TEAM_OPPONENT,
            opponent_candidates
        )
        obstacle_msgs.extend(opponent_candidate_messages)

    def _add_remaining_obstacles_to(self, obstacle_msgs: List[Robot]) -> None:
        remaining_candidates = self._misc_obstacles_detector.get_candidates()
        remaining_candidate_messages = self._create_obstacle_messages(
            Robot().attributes.TEAM_UNKNOWN,
            remaining_candidates
        )
        obstacle_msgs.extend(remaining_candidate_messages)

    def _publish_obstacles_message(self, image_msg: Image, obstacle_msgs: List[Robot]) -> None:
        obstacles_msg = ros_utils.build_robot_array_msg(image_msg.header, obstacle_msgs)
        self._publisher.publish(obstacles_msg)

    def _add_obstacles_to_debug_image(self) -> None:
        self._add_team_mates_to_debug_image()
        self._add_opponents_to_debug_image()
        self._add_remaining_objects_to_debug_image()

    def _add_team_mates_to_debug_image(self) -> None:
        team_mate_candidates = self._team_mates_detector.get_candidates()
        self._debug_image.draw_obstacle_candidates(
            team_mate_candidates,
            DebugImageComponent.Colors.team_mates,
            thickness=3
        )

    def _add_opponents_to_debug_image(self) -> None:
        opponent_candidates = self._opponents_detector.get_candidates()
        self._debug_image.draw_obstacle_candidates(
            opponent_candidates,
            DebugImageComponent.Colors.opponents,
            thickness=3
        )

    def _add_remaining_objects_to_debug_image(self) -> None:
        remaining_candidates = self._misc_obstacles_detector.get_candidates()
        self._debug_image.draw_obstacle_candidates(
            remaining_candidates,
            DebugImageComponent.Colors.misc_obstacles,
            thickness=3
        )

    def set_image(self, image: np.ndarray) -> None:
        self._team_mates_detector.set_image(image)
        self._opponents_detector.set_image(image)
        self._misc_obstacles_detector.set_image(image)


class DebugImageComponent(IVisionComponent):
    """
    This component handles and publishes the debug image.
    """

    class Colors:
        # BGR
        ball = (0, 255, 0)  # green
        team_mates = (153, 51, 255)  # magenta
        opponents = (255, 255, 102)  # cyan
        misc_obstacles = (160, 160, 160)  # grey
        goalposts = (255, 255, 255)  # white
        field_boundary = (0, 0, 255)  # red
        field_boundary_convex = (0, 255, 255)  # yellow
        lines = (255, 0, 0)  # blue

    def __init__(self, node):
        self._config: Dict = {}
        self._node: Node = node
        self._debug_image: Optional[debug.DebugImage] = None
        self._publisher: Optional[rclpy.publisher.Publisher] = None

    def configure(self, config: Dict, debug_mode: bool) -> None:
        self._debug_image = DebugImageFactory.get(config)
        logger.info('Debug images are published')

        self._register_publisher(config)
        self._config = config

    def _register_publisher(self, config: Dict) -> None:
        self._publisher = ros_utils.create_or_update_publisher(
            self._node,
            self._config, config,
            self._publisher,
            'ROS_debug_image_msg_topic',
            Image
        )

    def run(self, image_msg: Image) -> None:
        debug_image_msg = self._create_debug_image_msg(image_msg)
        self._publish_debug_image_msg(debug_image_msg)

    def _create_debug_image_msg(self, image_msg: Image) -> Image:
        return ros_utils.build_image_msg(image_msg.header, self._debug_image.get_image(), 'bgr8')

    def _publish_debug_image_msg(self, debug_image_msg: Image) -> None:
        self._publisher.publish(debug_image_msg)

    def set_image(self, image: np.ndarray) -> None:
        self._debug_image.set_image(image)