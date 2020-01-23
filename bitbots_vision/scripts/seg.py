load = True
train = not load

from keras_segmentation.models import all_models 

model_name = "pspnet"
epoch = 8
input_size = 192
model = all_models.model_from_name[model_name](n_classes=2,  input_height=input_size, input_width=input_size)

if load:
    model.load_weights("/srv/ssd_nvm/deep_field/models/22_01_20/" + model_name + "." + str(epoch))

if train:
    model.train(
        train_images =  "/srv/ssd_nvm/deep_field/data/group_all/images/",
        train_annotations = "/srv/ssd_nvm/deep_field/data/group_all/labels/",
        checkpoints_path = "/tmp/fcn_08_05" , epochs=15
    )

import cv2
import os
import numpy
import time

path = "/srv/ssd_nvm/deep_field/data/eval/images/"
#path = "/tmp/"

for subdir, dirs, files in os.walk(path):
    #img = cv2.imread("/srv/ssd_nvm/deep_field/data/group1/637/images/frame{:04d}.png".format(i))  
    for file in files:
        print(file)
        img = cv2.imread(os.path.join(path, file))    
    
        if img is None:
            continue

        start_time = time.time()
        out = model.predict_segmentation(inp=img)
        end_time = time.time()
        print("Inference time: {}".format(end_time-start_time))

        comb = cv2.resize(out.astype('float32') * 100, dsize=(img.shape[1], img.shape[0])).astype('uint8') 
    
        tmp = numpy.zeros((img.shape[0], img.shape[1], 3))
        tmp[:,:,2] = comb
        
        comp = tmp * 0.5 + img * 0.5
        cv2.imwrite(os.path.join("/tmp/", file) , comp)
    
