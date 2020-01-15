load = True
train = not load


from keras_segmentation.models.fcn import fcn_8

model = fcn_8(n_classes=3 ,  input_height=224, input_width=224)

if load:
    model.load_weights("/tmp/fcn_08_05.13")

if train:
    model.train(
        train_images =  "/srv/ssd_nvm/deep_field/data/group_all/images/",
        train_annotations = "/srv/ssd_nvm/deep_field/data/group_all/labels/",
        checkpoints_path = "/tmp/fcn_08_05" , epochs=15
    )

import cv2
import numpy

for i in range(0,1):
    #img = cv2.imread("/srv/ssd_nvm/deep_field/data/group1/637/images/frame{:04d}.png".format(i))  
    img = cv2.imread("/tmp/maxresdefault.jpg")    

    if img is None:
        continue

    out = model.predict_segmentation(inp=img)
    
    comb = cv2.resize(out.astype('float32') * 100, dsize=(img.shape[1], img.shape[0])).astype('uint8') 
    
    tmp = numpy.zeros((img.shape[0], img.shape[1], 3))
    print("Image {}".format(i))
    tmp[:,:,2] = comb
    
    comp = tmp * 0.5 + img * 0.5
    cv2.imwrite("/tmp/test{:04d}.png".format(i), comp)
    
