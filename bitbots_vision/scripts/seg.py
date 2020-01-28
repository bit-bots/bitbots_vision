load = True
train = not load

from keras_segmentation.models import all_models

model_name = "pspnet"
epoch = 8
input_size = 192
model_folder = "/srv/ssd_nvm/deep_field/models/22_01_20/"

spacer = "~"*100

model = all_models.model_from_name[model_name](n_classes=2,  input_height=input_size, input_width=input_size)

if load:
    model.load_weights(model_folder + model_name + "." + str(epoch))

if train:
    model.train(
        train_images =  "/srv/ssd_nvm/deep_field/data/group_all/images/",
        train_annotations = "/srv/ssd_nvm/deep_field/data/group_all/labels/",
        checkpoints_path = "/tmp/fcn_08_05" , epochs=15
    )

import cv2
import os
import numpy as np
import time

#data_path = "/home/florian/Desktop/"
data_path = "/tmp/eval/"

inference_time_array = []

files = sorted([file_name for file_name in os.listdir(data_path) if os.path.isfile(os.path.join(data_path, file_name))])

for current_file in files:
    print("File: {}".format(current_file))

    img = cv2.imread(os.path.join(data_path, current_file))

    if img is None:
        continue

    start_time = time.time()
    out = model.predict_segmentation(inp=img)
    end_time = time.time()

    inference_time = end_time-start_time
    inference_time_array.append(inference_time)

    print("Inference time: {}".format(inference_time))

    comb = cv2.resize(out.astype('float32') * 100, dsize=(img.shape[1], img.shape[0])).astype('uint8')

    tmp = np.zeros((img.shape[0], img.shape[1], 3))
    tmp[:,:,2] = comb

    comp = tmp * 0.5 + img * 0.5
    cv2.imwrite(os.path.join("/tmp/", current_file) , comp)

    print(spacer)

inference_time_array = np.array(inference_time_array[1:])

mean_inference_time = np.mean(inference_time_array)
std_inference_time= np.std(inference_time_array)

print("Mean: {} | Std: {}".format(mean_inference_time, std_inference_time))

