#! /usr/bin/env python3

import os
import cv2
import numpy
import yaml
import json

from keras_segmentation.models import all_models
from keras_segmentation.predict import evaluate

def model_from_checkpoint_path(checkpoints_path):
    model_path = checkpoints_path.split(".")[0]
    config_path = model_path + "_config.json"
    assert (os.path.isfile(config_path), "Checkpoint not found: '{}'".format(config_path))

    with open(config_path, 'r') as file:
        model_config = json.load(file)
    model = all_models.model_from_name[model_config['model_class']](
        model_config['n_classes'], input_height=model_config['input_height'],
        input_width=model_config['input_width'])
    model.load_weights(checkpoints_path)
    print("loaded weights ", checkpoints_path)
    return model

devider = "~"*100

inp_images_dir = "/srv/ssd_nvm/deep_field/data/eval/images/"  # Evaluation dataset
annotations_dir = "/srv/ssd_nvm/deep_field/data/eval/labels/"  # Evaluation labels
models_dir = "/srv/ssd_nvm/deep_field/models/22_01_20/"  # Directory with models files

evaluation_file = os.path.join(models_dir, "eval.yaml")  # Path of evaluation file to save

evaluations = {}

if os.path.isfile(evaluation_file):  # Reload already evaluated models
    with open(evaluation_file, 'r') as file:
        evaluation = yaml.full_load(file)

# Evaluate models
#################
for file_name in [file_name for file_name in os.listdir(models_dir) if os.path.isfile(os.path.join(models_dir, file_name))]:
    if ".json" in file_name:  # Filter non model files
        continue
    if file_name in evaluations:  # Skip already evaluated model files
        continue
    print(devider)
    print("Evaluating model: {}".format(file_name))

    # Evaluate
    evaluations[file] = evaluate(
            model=model_from_checkpoint_path(os.path.join(models_dir, file_name)),
            inp_images_dir=inp_images_dir,
            annotations_dir=annotations_dir)

    print(evaluations[file_name])

    # Save evaluation data
    with open(evaluation_file, 'w') as file:
        yaml.dump(evaluations, file)
