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
epochs = 15

for modelname, model in all_models.model_from_name.items():
    model = model(n_classes=n_classes, input_width=input_width, input_height=input_height)

    print(devider)
    print("Training with model: {}".format(modelname))

    model.train(
        train_images = train_images,
        train_annotations = train_annotations,
        checkpoints_path = os.path.join(checkpoints_base_path, modelname + time.strftime("_%d_%m_%y_%H_%M_%S/", time.localtime())),
        epochs=epochs
    )

    del model


"""
for i in range(0,1):
    img = cv2.imread("/srv/ssd_nvm/deep_field/data/group1/637/images/frame{:04d}.png".format(i))  # TODO: Eval data set

    if img is None:
        continue

    out = model.predict_segmentation(inp=img)
    
    comb = cv2.resize(out.astype('float32') * 100, dsize=(img.shape[1], img.shape[0])).astype('uint8') 
    
    tmp = numpy.zeros((img.shape[0], img.shape[1], 3))
    print("Image {}".format(i))
    tmp[:,:,2] = comb
    
    comp = tmp * 0.5 + img * 0.5
    cv2.imwrite("/tmp/test{:04d}.png".format(i), comp)
"""