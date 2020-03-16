#! /usr/bin/env python3

import sys
import cv2
import numpy

from keras_segmentation.models import all_models

devider = "~"*100

# Default config
################
config = {}
config['epochs'] = 25  # Number of epochs
config['n_classes'] = 2  # Number of segmentation classes
config['train_images'] = "/srv/ssd_nvm/deep_field/data/group_all/images/"  # Path to image dataset
config['train_annotations'] = "/srv/ssd_nvm/deep_field/data/group_all/labels/"  # Path to label dataset
config['checkpoints_base_path'] = "/srv/ssd_nvm/deep_field/models/13_03_20/"  # Path to store checkpoints

# Train models
##############
def train(modelname):
    print(devider)
    print("Training with model: {}".format(modelname))

    model = all_models.model_from_name[modelname](n_classes=config['n_classes'])

    model.train(
            train_images=config['train_images'],
            train_annotations=config['train_annotations'],
            checkpoints_path=config['checkpoints_base_path'] + modelname,
            epochs=config['epochs'],
            do_augment=True,
            augmentation_name="aug_all2")

if __name__ == "__main__":
    modelname = sys.argv[1]
    train(modelname)
