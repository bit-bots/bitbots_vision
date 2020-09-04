import rospy
import numpy as np
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

# color range for yellow color mask
LOWER_RANGE = np.array([20, 25, 25])
UPPER_RANGE = np.array([24, 255, 255])

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
                trainer_pose = find_trainer_pose(poses, image)

                if trainer_pose is not None:
                    # use the neural network to detect which gesture is currently done
                    detected_gesture = -1
                    # todo set correct pose number
                    self.gesture_pub.publish(detected_gesture)

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

    def pose_cb(self, msg: HumanPoseArray):
        self.current_pose = msg
        self.new_pose = True

    def image_cb(self, msg: Image):
        self.current_image = cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")


if __name__ == "__main__":
    recognition = GestureRecognition()
    recognition.run()
