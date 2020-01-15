#! /usr/bin/env python3

import os
import time
import cv2
import numpy

from keras_segmentation.models import all_models

devider = "~"*100

# Arguments
###########
n_classes = 2
input_width = 224
input_height = 224

train_images = "/srv/ssd_nvm/deep_field/data/group_all/images/"
train_annotations = "/srv/ssd_nvm/deep_field/data/group_all/labels/"
checkpoints_base_path = "/srv/ssd_nvm/deep_field/models/"
epochs = 25

for modelname, model in all_models.model_from_name.items():
    print(devider)
    print("Training with model: {}".format(modelname))

    model = model(n_classes=n_classes, input_width=input_width, input_height=input_height)

    modelpath = modelname + "_" + time.strftime("%d_%m_%y_%H_%M_%S", time.localtime()) + "/" + modelname

    model.train(
        train_images = train_images,
        train_annotations = train_annotations,
        checkpoints_path = os.path.join(checkpoints_base_path, modelpath),
        epochs=epochs
    )

    del model
