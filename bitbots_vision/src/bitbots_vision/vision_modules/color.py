import abc
import cv2
import yaml
import pickle
import VisionExtensions
import numpy as np
import os

try:
    from keras_segmentation.predict import predict as inference
except ModuleNotFound:
    print("Keras segmentation import failed. Neural field color detector is now unavalabile!")

class ColorDetector(object):
    """
    ColorDetector is abstract super-class of specialized sub-classes.
    ColorDetectors are used e.g. to check, if a pixel matches the defined color space
    or to create masked binary images.
    """

    def __init__(self, config, key):
        # type: (dict) -> None
        """
        Initialization of ColorDetector.

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        # Initial setup

        self._image = None
        self._mask = None

        self._config = {}
        self.update_config(config, key)

    def update_config(self, config, key):
        # type: (dict) -> None
        """
        Update (or initiate) the color detector setup with the new config.
        Always make a copy of self.config if a comparison between the old and new config is needed!

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        self._config = config

    @abc.abstractmethod
    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns, if bgr pixel is in color space

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color space or not
        """

    def set_image(self, image):
        # type: (np.array) -> None
        """
        Refreshes class variables after receiving an image

        :param image: the current frame of the video feed
        :return: None
        """
        self._image = image
        self._mask = None

    def get_mask_image(self, optional_image=None):
        # type: (np.array) -> np.array
        """
        Returns the color mask of the cached (or optional given) image
        (0 for not in color range and 255 for in color range)

        :param np.array optional_image: Optional input image
        :return np.array: masked image
        """
        if optional_image is not None:
            # Mask of optional image
            mask = self._mask_image(optional_image)
        else:
            # Mask of default cached image
            if self._mask is None:
                self._mask = self._mask_image(self._image)
            mask = self._mask

        return mask

    @abc.abstractmethod
    def _mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Returns the color mask of the image
        (0 for not in color range and 255 for in color range)

        :param np.array image: input image
        :return np.array: masked image
        """

    def mask_bitwise(self, mask):
        # type: (np.array) -> np.array
        """
        Returns bitwise-and mask with current image

        :param np.array mask: mask
        :return np.array: bitwise-and mask with current image
        """
        return cv2.bitwise_and(self.get_mask_image(), self.get_mask_image(), mask=mask)

    def match_adjacent(self, image, point, offset=1, threshold=200):
        # type: (np.array, tuple[int, int], int, float) -> bool
        """
        Returns, if an area is in color space

        :param np.array image: the full image
        :param tuple[int, int] point: a x-, y-tuple defining coordinates in the image
        :param int offset: the number of pixels to check in the surrounding of the
            point (like a radius but for a square)
        :param float threshold: the mean needed to accept the area to match (0-255)
        :return bool: whether area is in color space or not
        """
        area = image[
               max(0, point[1] - offset):
               min(image.shape[0] - 1, point[1] + offset),
               max(0, point[0] - offset):
               min(image.shape[1] - 1, point[0] + offset)
               ]
        return self.match_area(area, threshold=threshold)

    def match_area(self, area, threshold=200):
        # type: (np.array, float) -> bool
        """
        Returns if an area is in color space

        :param np.array area: the image area to check
        :param float threshold: the mean needed to accept the area to match (0-255)
        :return bool: whether area is in color space or not
        """
        return np.mean(self.get_mask_image(area)) > threshold

    @staticmethod
    def pixel_bgr2hsv(pixel):
        # type: (np.array) -> np.array
        """
        Converts bgr-pixel to hsv-pixel

        :param np.array pixel: brg-pixel
        :return np.array: hsv-pixel
        """
        pic = np.zeros((1, 1, 3), np.uint8)
        pic[0][0] = pixel
        return cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)[0][0]

    def compute(self):
        # type: () -> None
        """
        Compute image masks.

        :return: None
        """
        self.get_mask_image()


class NeuralFieldColorDetector(ColorDetector):
    """
    Color detector which masks the field using a neural network.
    """
    def __init__(self, config, model_path):
        # type: (dict, str) -> None
        self._model_path = model_path

    def _mask_image(self, image):
        return inference(checkpoints_path=self._model_path,  inp=image)


class HsvSpaceColorDetector(ColorDetector):
    """
    HsvSpaceColorDetector is a ColorDetector, that is based on the HSV-color space.
    The HSV-color space is adjustable by setting min- and max-values for hue, saturation and value.
    """
    def __init__(self, config, color_str, key):
        # type: (dict, str) -> None
        """
        Initialization of HsvSpaceColorDetector.

        :param dict config: dictionary of the vision node configuration parameters
        :param str color_str: color (described in the config) that should be detected.
        :return: None
        """
        self._detector_name = "{}_color_detector".format(color_str)

        # Initialization of parent ColorDetector.
        super(HsvSpaceColorDetector, self).__init__(config, key)

    def update_config(self, config, key):
        # type: (dict) -> None
        """
        Update (or initiate) the color detector setup with the new config.
        Always make a copy of self.config if a comparison between the old and new config is needed!

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        super(HsvSpaceColorDetector, self).update_config(config, key)

        try:
            self._min_vals = np.array([
                        config[self._detector_name + '_lower_values_h'],
                        config[self._detector_name + '_lower_values_s'],
                        config[self._detector_name + '_lower_values_v']
                ])

            self._max_vals = np.array([
                        config[self._detector_name + '_upper_values_h'],
                        config[self._detector_name + '_upper_values_s'],
                        config[self._detector_name + '_upper_values_v']
                ])
        except KeyError:
            raise

    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns if bgr pixel is in color space

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color space or not
        """
        pixel = self.pixel_bgr2hsv(pixel)
        return (self._max_vals[0] >= pixel[0] >= self._min_vals[0]) and \
               (self._max_vals[1] >= pixel[1] >= self._min_vals[1]) and \
               (self._max_vals[2] >= pixel[2] >= self._min_vals[2])

    def _mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Returns the color mask of the image
        (0 for not in color range and 255 for in color range)

        :param np.array image: input image
        :return np.array: masked image
        """
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv_image, self._min_vals, self._max_vals)


class PixelListColorDetector(ColorDetector):
    """
    PixelListColorDetector is a ColorDetector, that is based on a lookup table of color values.
    The color space is loaded from color-space-file at color_path (in config).
    The color space is represented by boolean-values for RGB-color-values.

    The following parameters of the config dict are needed:
        'field_color_detector_path'
    """

    def __init__(self, config, key):
        # type:(dict, str, str) -> None
        """
        Initialization of PixelListColorDetector.

        :param dict config: dictionary of the vision node configuration parameters
        :param str package_path: path of package
        :param str key: key of the color space file parameter in config
        :return: None
        """
        # Initialization of parent ColorDetector.
        super(PixelListColorDetector, self).__init__(config, key)

    def update_config(self, config, key):
        # type: (dict) -> None
        """
        Update (or initiate) the color detector setup with the new config.
        Always make a copy of self.config if a comparison between the old and new config is needed!

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        tmp_config = self._config.copy()

        super(PixelListColorDetector, self).update_config(config, key)

        self._color_space = self._init_color_space(config[key])

    def _init_color_space(self, color_path):
        # type: (str) -> None
        """
        Initialization of color space from .yaml or .pickle file

        :param str color_path: path to file containing the accepted colors
        :return: None
        """
        color_space = np.zeros((256, 256, 256), dtype=np.uint8)
        if color_path.endswith('.yaml'):
            with open(color_path, 'r') as stream:
                try:
                    color_values = yaml.safe_load(stream)
                except yaml.YAMLError as exc:
                    pass

        # pickle-file is stored as '.pickle'
        elif color_path.endswith('.pickle'):
            try:
                with open(color_path, 'rb') as f:
                    color_values = pickle.load(f)
            except pickle.PickleError as exc:
                pass

        # compatibility with colorpicker
        if 'color_values' in color_values.keys():
            color_values = color_values['color_values']['greenField']
        length = len(color_values['red'])
        if length == len(color_values['green']) and \
                length == len(color_values['blue']):
            # setting colors from yaml file to True in color space
            for x in range(length):
                color_space[color_values['blue'][x], color_values['green'][x], color_values['red'][x]] = 255
        return color_space

    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns, if bgr pixel is in color space

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color space or not
        """
        return self._color_space[pixel[0], pixel[1], pixel[2]]

    def _mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Returns the color mask of the image
        (0 for not in color range and 255 for in color range)

        :param np.array image: input image
        :return np.array: masked image
        """
        return VisionExtensions.maskImg(image, self._color_space)
