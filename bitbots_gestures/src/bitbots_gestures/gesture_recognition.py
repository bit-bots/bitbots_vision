import rospy
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
                trainer_pose = None
                # todo set correct pose to trainer_pose
                if trainer_pose is not None:
                    # use the neural network to detect which gesture is currently done
                    detected_gesture = -1
                    # todo set correct pose number
                    self.gesture_pub.publish(detected_gesture)

    def pose_cb(self, msg: HumanPoseArray):
        self.current_pose = msg
        self.new_pose = True

    def image_cb(self, msg: Image):
        self.current_image = cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")


if __name__ == "__main__":
    recognition = GestureRecognition()
    recognition.run()
