#! /usr/bin/env python3

import os
import cv2
import numpy
import yaml
from keras_segmentation.predict import evaluate

devider = "~"*100

inp_images_dir = "/srv/ssd_nvm/deep_field/data/eval/images/"  # Evaluation dataset
annotations_dir = "/srv/ssd_nvm/deep_field/data/eval/labels/"  # Evaluation labels
models_dir = "/srv/ssd_nvm/deep_field/models/22_01_20/"  # Directory with models files

evaluation_file = os.path.join(models_dir, "eval.yaml")  # Path of evaluation file to save

evaluations = {}

# Evaluate models
#################
for file_name in [file_name for file_name in os.listdir(models_dir) if os.path.isfile(os.path.join(models_dir, file_name))]:
    if ".json" in file_name:  # Filter non model files
        continue
    print(devider)
    print("Evaluating model: {}".format(file_name))

    # Evaluate
    evaluations[file] = evaluate(
            inp_images_dir=inp_images_dir,
            annotations_dir=annotations_dir,
            checkpoints_path=file_name)

    print(evaluations[file_name])

    # Save evaluation data
    with open(evaluation_file, 'w') as file:
        yaml.dump(evaluations, file)
