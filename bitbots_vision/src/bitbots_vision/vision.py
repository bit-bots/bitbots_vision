#! /usr/bin/env python3

import sys
import os
import cv2
import yaml
import glob
import numpy as np
import multiprocessing
from vision_modules import field_boundary, color, debug, lines


class Vision:
    def __init__(self):
        self._base_config_path = sys.argv[1]
        print(self._base_config_path)
        self._config = {}
        self._debug_drawer = debug.DebugImage()

        self._configure_vision(self._read_config(self._base_config_path))
        self._main()

    def _read_config(self, config_path):
        with open(config_path, "r") as config_file:
            new_config = yaml.load(config_file, Loader=yaml.SafeLoader)
        return new_config

    def _configure_vision(self, config):
        self._label_drawer = debug.DebugImage()

        if config['neural_field_detection']:
            self._field_color_detector = color.NeuralFieldColorDetector(config, config['neural_field_color_detector_model_path'])
        else:
            # Set the static field color detector
            self._field_color_detector = color.PixelListColorDetector(config, "field_color_detector_path")

        # Set the static line color detector
        self._line_color_detector = color.PixelListColorDetector(config, "line_color_detector_path")

        # Set the white color detector
        self._white_color_detector = color.HsvSpaceColorDetector(config, "white", None)

        # Get field boundary detector class by name from _config
        field_boundary_detector_class = field_boundary.FieldBoundaryDetector.get_by_name(
            config['field_boundary_detector_search_method'])

        # Set the field boundary detector
        self._field_boundary_detector = field_boundary_detector_class(
            config,
            self._field_color_detector)

        # Set the line detector
        if config["lines_hsv"]:
            self._line_detector = lines.LineDetector(config, self._white_color_detector, self._field_color_detector, self._field_boundary_detector)
        else:
            self._line_detector = lines.LineDetector(config, self._line_color_detector, self._field_color_detector, self._field_boundary_detector)

        # The old _config gets replaced with the new _config
        self._config = config

    def _main(self):
        default_config = self._config.copy()

        dataset_roots = self._config['dataset_roots']

        procs = []



        if default_config['parallel']:
            for ds_root in dataset_roots:
                proc = multiprocessing.Process(target=self._handle_dataset, args=[ds_root, default_config])
                procs.append(proc)
                proc.start()

            for proc in procs:
                proc.join()
        else:
            for ds_root in dataset_roots:
                self._handle_dataset(ds_root, default_config)

    def _handle_dataset(self, ds_root, default_config):
        devider = "~"*100
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
            new_config = self._read_config(self.config_path)
            tmp_config = default_config.copy()
            for k in new_config.keys():
                tmp_config[k] = new_config[k]
            self._configure_vision(tmp_config)
        else:
            print(f"No config file found at '{self.config_path}'...")

        # Load images and generate labels and debug images
        image_files = glob.glob(f"{self.image_dir}*.jpg") + glob.glob(f"{self.image_dir}*.png")
        print(image_files)
        for image_file in sorted(image_files):
            print(f"Loading image file at'{image_file}'...")
            self._handle_image(image_file)

        print(devider)

    def _handle_image(self, image_file):
        image_path = os.path.basename(image_file)
        image = cv2.imread(os.path.join(self.image_dir, image_path))

        # Skip if image is None
        if image is None:
            print(f"WARNING: Can not load image file at '{image_file}'...")
            return

        # Create empty label image
        image_shape = image.shape
        label = np.zeros((image_shape[0], image_shape[1], 3), dtype=np.uint8)
        self._label_drawer.set_image(label)

        self._field_color_detector.set_image(image)
        self._field_boundary_detector.set_image(image)

        self._field_color_detector.compute()


        # Handle field boundary

        if self._config['masks'] or self._config['debug']:
            field_boundary_mask = None
            if self._config['field_boundary_mask'] == "convex":
                field_boundary_mask = self._field_boundary_detector.get_convex_mask()
            elif self._config['field_boundary_mask'] == "normal":
                field_boundary_mask = self._field_boundary_detector.get_mask()
            else:
                print("WARNING: Unknown field_boundary_mask parameter!")
                return

            self._label_drawer.draw_mask(field_boundary_mask, (1, 1, 1), opacity=1)
            label = self._label_drawer.get_image()


        # Handle lines
        if self._config['lines'] and self._config['masks']:
            self._white_color_detector.set_image(image)
            self._line_detector.set_image(image)
            self._white_color_detector.compute()
            self._line_detector.compute()

            line_mask = self._line_detector.get_line_mask()

            self._label_drawer.set_image(label)
            self._label_drawer.draw_mask(line_mask, (2, 2, 2), opacity=1)
            label = self._label_drawer.get_image()

        if self._config['masks']:
            cv2.imwrite(self.labels_dir + image_path[0:-4] + ".png", label)

        if self._config['debug']:
            self._debug_drawer.set_image(image)
            self._debug_drawer.draw_mask(field_boundary_mask, (255,0,0), opacity=0.5)
            if self._config['lines'] and self._config['masks']:
                self._debug_drawer.draw_mask(line_mask, (168, 50, 162), opacity=0.8)
            cv2.imwrite(self.debug_dir + image_path[0:-4] + ".png", self._debug_drawer.get_image())

        if self._config['imagetagger_annotations']:
            field_boundary_points = self._field_boundary_detector.get_convex_field_boundary_points()
            field_boundary_points_serialized = ','.join([f'x{index + 1}: {int(point[0])}, y{index + 1}: {int(point[0])}' for index, point in enumerate(field_boundary_points)])
            imagetagger_input_format_string = f"{image_path}|field edge|{{{field_boundary_points_serialized}}}"
            with open(self.labels_dir + "imagetagger_upload_annotations.dat", 'a') as f:
                f.write(f"{imagetagger_input_format_string}\n")



if __name__ == '__main__':
    Vision()
