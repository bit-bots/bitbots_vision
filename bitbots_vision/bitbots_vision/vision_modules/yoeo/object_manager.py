import os.path as osp
import rclpy
from typing import Optional, Dict


from bitbots_vision.vision_modules import ros_utils
from . import yoeo_handlers

logger = rclpy.logging.get_logger('bitbots_vision')


class YOEOObjectManager:
    """
    This class manages the creation and update of the YOEO handler instance.
    """
    _HANDLERS_BY_NAME = {
        'openvino': yoeo_handlers.YOEOHandlerOpenVino,
        'onnx': yoeo_handlers.YOEOHandlerONNX,
        'pytorch': yoeo_handlers.YOEOHandlerPytorch,
        'tvm': yoeo_handlers.YOEOHandlerTVM
    }

    _config: Dict = {}
    _framework: str = ""
    _package_directory: str = ""
    _package_directory_set: bool = False
    _yoeo_instance: Optional[yoeo_handlers.IYOEOHandler] = None

    @classmethod
    def set_package_directory(cls, package_directory: str) -> None:
        """
        Set the package directory. Required before first use in order to have the correct paths for the models.
        """
        cls._package_directory = package_directory
        cls._package_directory_set = True

    @classmethod
    def get(cls) -> yoeo_handlers.IYOEOHandler:
        """
        Get the current YOEO handler instance.

        :return: the current YOEO handler instance
        :rtype: IYOEOHandler
        """
        if cls._yoeo_instance is None:
            logger.error("No yoeo handler created yet!")
        else:
            return cls._yoeo_instance

    @classmethod
    def get_id(cls) -> int:
        """
        Get the (Python) ID of the current YOEO handler instance.

        :return: the ID of the current YOEO handler instance
        :rtype: int
        """
        if cls._yoeo_instance is None:
            logger.error("No yoeo handler created yet")
        else:
            return id(cls._yoeo_instance)

    @classmethod
    def configure(cls, config: Dict) -> None:
        if not cls._package_directory_set:
            logger.error("Package directory not set!")

        framework = config["yoeo_framework"]
        cls._verify_framework_parameter(framework)

        model_path = cls._get_full_model_path(config["yoeo_model_path"])
        cls._verify_required_neural_network_files_exist(framework, model_path)

        cls._configure_yoeo_instance(config, framework, model_path)

        cls._config = config
        cls._framework = framework

    @staticmethod
    def _verify_framework_parameter(framework: str) -> None:
        if framework not in {'openvino', 'onnx', 'pytorch', 'tvm'}:
            logger.error(f"Unknown neural network framework '{framework}'")

    @classmethod
    def _get_full_model_path(cls, model_path: str) -> str:
        return osp.join(cls._package_directory, 'models', model_path)

    @classmethod
    def _verify_required_neural_network_files_exist(cls, framework: str, model_path: str) -> None:
        if not cls._model_files_exist(framework, model_path):
            logger.error("No matching model file(s) found!")

    @classmethod
    def _model_files_exist(cls, framework: str, model_path: str) -> bool:
        return cls._HANDLERS_BY_NAME[framework].model_files_exist(model_path)

    @classmethod
    def _configure_yoeo_instance(cls, config: Dict, framework: str, model_path: str) -> None:
        if cls._new_yoeo_handler_is_needed(framework):
            cls._instantiate_new_yoeo_handler(config, framework, model_path)
        elif cls._yoeo_parameters_have_changed(config):
            cls._yoeo_instance.configure(config)

    @classmethod
    def _new_yoeo_handler_is_needed(cls, framework: str) -> bool:
        return cls._yoeo_instance is None or cls._framework != framework

    @classmethod
    def _instantiate_new_yoeo_handler(cls, config: Dict, framework: str, model_path: str) -> None:
        cls._yoeo_instance = cls._HANDLERS_BY_NAME[framework](config, model_path)
        logger.info(f"Using {cls._yoeo_instance.__class__.__name__}")

    @classmethod
    def _yoeo_parameters_have_changed(cls, new_config: Dict) -> bool:
        return ros_utils.config_param_change(cls._config, new_config, r'yoeo_')
