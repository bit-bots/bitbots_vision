import rospy
from human_pose_estimation_openvino.msg import HumanPoseArray
from sensor_msgs.msg import Image


class GestureRecognition:

    def __init__(self):
        rospy.init_node("gesture_recognition")

        self.current_pose = None
        self.current_image = None
        self.pose_sub = rospy.Subscriber("/human_poses", HumanPoseArray, self.pose_cb, queue_size=1, tcp_nodelay=True)
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_cb, queue_size=1, tcp_nodelay=True)

        rospy.spin()

    def pose_cb(self, msg: HumanPoseArray):
        self.current_pose = msg

    def image_cb(self, msg: Image):
        self.current_image = msg


if __name__ == "__main__":
    recognition = GestureRecognition()
