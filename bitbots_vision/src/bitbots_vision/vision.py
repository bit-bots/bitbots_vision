#! /usr/bin/env python3

import sys
import os
import cv2
import yaml
import glob
from bitbots_vision.vision_modules import field_boundary, color, debug


class Vision:
    def __init__(self):
        # type () -> None
        """
        Vision is the main ROS-node for handling all tasks related to image processing.
        Initiating 'bitbots_vision' node.

        :return: None
        """
        self._base_config_path = sys.argv[1]
        print(self._base_config_path)
        self._config = {}
        self._debug_drawer = debug.DebugImage()

        self._configure_vision(self._read_config(self._base_config_path))
        self._main()

    def _read_config(self, config_path):
        with open(self.config_path, "r") as config_file:
            new_config = yaml.load(config_file, Loader=yaml.SafeLoader)
        return new_config

    def _configure_vision(self, config):
        """
        Reconfigure configuration.

        :param config: New config
        """
        
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

        # The old _config gets replaced with the new _config
        self._config = config

    def _main(self):
        """
        Main loop that processes the images and configuration changes
        """
        devider = "~"*100
        print(devider)

        default_config = self._config.copy()

        dataset_roots = self._config['dataset_roots']

        for ds_root in dataset_roots:
            print(f"Loading dataset at '{ds_root}'...")

            # Generating dataset paths
            self.config_path = os.path.join(ds_root, "config.yaml")
            self.image_dir = os.path.join(ds_root, "images/")
            self.labels_dir = os.path.join(ds_root, "labels/")
            self.debug_dir = os.path.join(ds_root, "debug/")

            if not os.path.exists(self.labels_dir):
                os.makedirs(self.labels_dir)
            if not os.path.exists(self.debug_dir):
                os.makedirs(self.debug_dir)

            # Overwrite default config with dataset specific config
            if os.path.isfile(self.config_path):
                print(f"Loading config file at '{self.config_path}'...")
                with open(self.config_path, "r") as config_file:
                    new_config = yaml.load(config_file, Loader=yaml.SafeLoader)
                tmp_config = default_config.copy()
                for k in new_config.keys():
                    tmp_config[k] = new_config[k]
                self._configure_vision(tmp_config)
            else:
                print(f"No config file found at '{self.config_path}'...")

            # Load images and generate labels and debug images
            image_files = glob.glob(f"{self.image_dir}*.jpg") + glob.glob(f"{self.image_dir}*.png")
            for image_file in sorted(image_files):
                print(f"Loading image file at'{image_file}'...")
                self._handle_image(image_file)

            print(devider)

        # Restore default config
        self._configure_vision(default_config, 0)

    def _handle_image(self, image_file):
        """
        Runs the vision pipeline

        :param str image_file: Path to image file in image_dir
        """
        image_path = os.path.basename(image_file)
        image = cv2.imread(os.path.join(self.image_dir, image_path))

        # Skip if image is None
        if image is None:
            print(f"WARNING: Can not load image file at '{image_file}'...")
            return

        self._field_color_detector.set_image(image)
        self._field_boundary_detector.set_image(image)

        self._field_color_detector.compute()

        mask = None
        if rospy.get_param("~field_boundary_mask") == "convex":
            mask = self._field_boundary_detector.get_convex_mask()
        elif rospy.get_param("~field_boundary_mask") == "normal":
            mask = self._field_boundary_detector.get_mask()
        else:
            print("WARNING: Unknown field_boundary_mask parameter!")
            return

        cv2.imwrite(self.labels_dir + image_path[0:-4] + ".png", mask)

        self._debug_drawer.set_image(image)
        self._debug_drawer.draw_mask(mask, (255,0,0), opacity=0.8)
        cv2.imwrite(self.debug_dir + image_path[0:-4] + ".png", self._debug_drawer.get_image())


if __name__ == '__main__':
    Vision()
