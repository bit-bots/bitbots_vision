import rospy
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import math

from human_pose_estimation_openvino.msg import HumanPoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Int8

NOSE = 0
NECK = 1
RShoulder = 2
RElbow = 3
RWrist = 4
LShoulder = 5
LElbow = 6
LWrist = 7
RHip = 8
RKnee = 9
RAnkle = 10
LHip = 11
LKnee = 12
LAnkle = 13
REye = 14
LEye = 15
REar = 16
LEar = 17
BACKGROUND = 18

# body labels
body_labels = ['idle', 'both-arms-up', 'arm-up-right', "arm-up-left", "show-right", "show-left", "show-up-right", "show-up-left", "clap", "cheer", 
"complain", "both-arms-right", "both-arms-left", "t-pose", "fists-together", "arm-bow-right", "arm-bow-left", "cross-arms", "time-out-low", "time-out-high"]

# color range for yellow color mask
LOWER_RANGE = np.array([20, 25, 25])
UPPER_RANGE = np.array([24, 255, 255])

# pytorch Model class
class Model(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(Model, self).__init__()
        self.i2h = nn.Linear(input_size, hidden_size)
        self.h2h = nn.Linear(hidden_size, hidden_size)
        self.h2o = nn.Linear(hidden_size, output_size)
        self.softmax = nn.LogSoftmax(dim=1)

    def forward(self, x):
        x = F.relu(self.i2h(x))
        x = self.h2h(x)
        x = self.h2o(x)
        x = self.softmax(x)
        return x

class GestureRecognition:

    def __init__(self):
        rospy.init_node("gesture_recognition")

        self.current_pose = None
        self.current_image = None
        self.new_pose = False
        self.bridge = CvBridge()
        self.pose_sub = rospy.Subscriber("/human_poses", HumanPoseArray, self.pose_cb, queue_size=1, tcp_nodelay=True)
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_cb, queue_size=1, tcp_nodelay=True)
        self.gesture_pub = rospy.Publisher("/gesture", Int8)

    def run(self):
        while not rospy.is_shutdown():
            if self.new_pose:
                poses = self.current_pose
                image = self.current_image
                # poses has following format
                # poses.poses is an list of detected poses
                # each of them consists of a list of keypoints and a score
                # example: poses.poses[0].keypoints[LKnee].x will give you the x coordinate of the LeftKnee of the first pose
                # the poses are already filtered with a score threshold so that only good poses are included here
                self.new_pose = False
                # we have new pose input
                # find out which pose (if any) belongs to the person with the yellow jacket
                trainer_pose = self.find_trainer_pose(poses, image)

                if trainer_pose is not None:
                    # use the neural network to detect which gesture is currently done
                    detected_gesture = self.predict_gesture(trainer_pose)
                    print("Detected gesture: ", detected_gesture)
                    # todo set correct pose number
                    self.gesture_pub.publish(detected_gesture)

    #---------------- POSE DETECTION CODE ----------------

    # Find the pose which belongs to the trainer
    def find_trainer_pose(poses, image):
        # create color mask for image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_RANGE, UPPER_RANGE)
        # find the pose with the most pixels that fit the mask
        trainer_pose = None
        max_mask_percentage = 0
        for pose in poses.poses:
            # get min/max x & y to estimate the upper body by a rectangle
            coords = np.array([pose[RShoulder], pose[LShoulder], pose[RHip], pose[LHip]])
            minX = int(np.min(coords[:,0:1]))
            maxX = int(np.max(coords[:,0:1]))
            minY = int(np.min(coords[:,1:2]))
            maxY = int(np.max(coords[:,1:2]))
            # calculate the percentage of white pixels in the mask within the body area
            body_area = mask[minY:maxY,minX:maxX]
            mask_percentage = np.sum(body_area) / (body_area.size * 255)
            # keep track of the most fitting pose for the trainer
            if(mask_percentage > max_mask_percentage):
                max_mask_percentage = mask_percentage
                trainer_pose = pose

        return trainer_pose

    # Normalize np vector
    def normalize(self, v1):
        v1_length = np.linalg.norm(v1)
        return v1 / v1_length

    # Calculate angle between 2 np vectors
    def angle(self, v1, v2):
        v1_norm = self.normalize(v1)
        v2_norm = self.normalize(v2)

        product = v1_norm[0] * v2_norm[0] + v1_norm[1] * v2_norm[1]
        product = max(-1, min(product, 1))

        acos = math.acos(product)
        angle = math.degrees(acos)
        return angle

    # Calculate body angles
    def get_body_angles(self, keypoints):
        shoulderR = self.angle(keypoints[NECK] - keypoints[RShoulder], keypoints[RElbow] - keypoints[RShoulder])
        shoulderL = self.angle(keypoints[NECK] - keypoints[LShoulder], keypoints[LElbow] - keypoints[LShoulder])
        ellbowR = self.angle(keypoints[RShoulder] - keypoints[RElbow], keypoints[RWrist] - keypoints[RElbow])
        ellbowL = self.angle(keypoints[LShoulder] - keypoints[LElbow], keypoints[LWrist] - keypoints[LElbow])
        return np.array([shoulderR, shoulderL, ellbowR, ellbowL]) / 180

    # Calculate boy bounding box
    def get_bounding_box(self, keypoints):

        # get important positions
        x_pos = np.array([keypoints[RElbow][0], keypoints[LElbow][0], keypoints[RWrist][0], keypoints[LWrist][0], keypoints[RShoulder][0], keypoints[LShoulder][0]])
        y_pos = np.array([keypoints[RElbow][1], keypoints[LElbow][1], keypoints[RWrist][1], keypoints[LWrist][1], keypoints[RShoulder][1], keypoints[LShoulder][1]])

        # get bounding box borders
        minValX = np.min(x_pos[np.nonzero(x_pos)])
        maxValX = np.max(x_pos[np.nonzero(x_pos)])
        minValY = np.min(y_pos[np.nonzero(y_pos)])
        maxValY = np.max(y_pos[np.nonzero(y_pos)])

        boundingBoxWidth = maxValX - minValX
        boundingBoxHeight = maxValY - minValY

        # calculate position relative to bounding box
        ellbowR = [(keypoints[RElbow][0] - minValX) / boundingBoxWidth, (keypoints[RElbow][1] - minValY) / boundingBoxHeight]
        ellbowL = [(keypoints[LElbow][0] - minValX) / boundingBoxWidth, (keypoints[LElbow][1] - minValY) / boundingBoxHeight]
        handR = [(keypoints[RWrist][0] - minValX) / boundingBoxWidth, (keypoints[RWrist][1] - minValY) / boundingBoxHeight]
        handL = [(keypoints[LWrist][0] - minValX) / boundingBoxWidth, (keypoints[LWrist][1] - minValY) / boundingBoxHeight]
        shoulderR = [(keypoints[RShoulder][0] - minValX) / boundingBoxWidth, (keypoints[RShoulder][1] - minValY) / boundingBoxHeight]
        shoulderL = [(keypoints[LShoulder][0] - minValX) / boundingBoxWidth, (keypoints[LShoulder][1] - minValY) / boundingBoxHeight]

        return np.array([ellbowR[0], ellbowR[1], ellbowL[0], ellbowL[1], handR[0], handR[1], handL[0], handL[1], shoulderR[0], shoulderR[1], shoulderL[0], shoulderL[1]])

    # Get the input tensor for the neural network for body keypoints
    def get_input_tensor(self, keypoints):
        positions = self.get_bounding_box(keypoints)
        angles = self.get_body_angles(keypoints)
        inputs = np.concatenate((angles, positions))
        inputs = np.array([(inputs * 2) - 1])
        input_tensor = torch.from_numpy(inputs)
        input_tensor = input_tensor.float()
        return input_tensor

    # Predict the gesture for given body keypoints
    def predict_gesture(self, keypoints):
        input_tensor = self.get_input_tensor(keypoints)
        model = torch.load('model.pth')
        outputs = model.forward(input_tensor)
        _, predicted = torch.max(outputs, 1)
        return body_labels[predicted]

    def pose_cb(self, msg: HumanPoseArray):
        self.current_pose = msg
        self.new_pose = True

    def image_cb(self, msg: Image):
        self.current_image = cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")


if __name__ == "__main__":
    recognition = GestureRecognition()
    recognition.run()
