#! /usr/bin/env python3

import os
import cv2
import numpy

from keras_segmentation.models import all_models

devider = "~"*100

# Default config
################
default_config = {}
default_config['epochs'] = 25  # Number of epochs
default_config['n_classes'] = 2  # Number of segmentation classes
default_config['input_width'] = 192  # Input width
default_config['input_height'] = 192  # Input height
default_config['train_images'] = "/srv/ssd_nvm/deep_field/data/group_all/images/"  # Path to image dataset
default_config['train_annotations'] = "/srv/ssd_nvm/deep_field/data/group_all/labels/"  # Path to label dataset
default_config['checkpoints_base_path'] = "/srv/ssd_nvm/deep_field/models/16_01_20/"  # Path to store checkpoints


# Model config
##############
models = {}
models['fcn_8'] = {}
models['fcn_32'] = {}
models['fcn_8_vgg'] = {}
models['fcn_32_vgg'] = {}
models['fcn_8_mobilenet'] = {}
models['fcn_32_mobilenet'] = {}
models['pspnet'] = {'input_width': 192, 'input_height': 192}
models['vgg_pspnet'] = {'input_width': 192, 'input_height': 192}
models['unet_mini'] = {}
models['unet'] = {}
models['mobilenet_unet'] = {'input_width': 224, 'input_height': 224}
models['mobilenet_segnet'] = {'input_width': 224, 'input_height': 224}
models['segnet'] = {}
models['vgg_segnet'] = {}

# Models, that run only on the CPU
cpu_models = {}
cpu_models['fcn_8_resnet50'] = {}
cpu_models['fcn_32_resnet50'] = {}
cpu_models['vgg_unet'] = {}

# Models, that do not run for some reason
blacklist_models = {}
blacklist_models['resnet50_pspnet'] = {'input_width': 192, 'input_height': 192}  # ValueError: Negative dimension size caused by subtracting 7 from 6 for 'avg_pool/AvgPool' (op: 'AvgPool'│ ) with input shapes: [?,6,6,2048].
blacklist_models['pspnet_50'] = {}  # Pooling parameters for input shape  (192, 192)  are not defined.
blacklist_models['pspnet_101'] = {}  # Pooling parameters for input shape  (192, 192)  are not defined.
blacklist_models['resnet50_unet'] = {}  # ValueError: Negative dimension size caused by subtracting 7 from 6 for 'avg_pool/AvgPool' (op: 'AvgPool') with input shapes: [?,6,6,2048].
blacklist_models['resnet50_segnet'] = {}  # ValueError: Negative dimension size caused by subtracting 7 from 6 for 'avg_pool/AvgPool' (op: 'AvgPool') with input shapes: [?,6,6,2048].


# Train models
##############
def train(models):
    for modelname, model_config in models.items():
        print(devider)
        print("Training with model: {}".format(modelname))

        # Merge configs
        config = default_config.copy()
        for key, value in model_config.items():
            config[key] = value

        model = all_models.model_from_name[modelname](
                n_classes=config['n_classes'],
                input_width=config['input_width'],
                input_height=config['input_height'])

        model.train(
                train_images=config['train_images'],
                train_annotations=config['train_annotations'],
                checkpoints_path=config['checkpoints_base_path'] + modelname,
                epochs=config['epochs'])

        del model

train(models)

# Train on CPU
os.environ['CUDA_VISIBLE_DEVICES'] = -1
train(cpu_models)
