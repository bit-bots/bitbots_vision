from vision_modules import ball, classifier, live_classifier, horizon, color, debug_image
from humanoid_league_msgs.msg import BallInImage, BallsInImage, LineSegmentInImage, LineInformationInImage
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
import sys
# moving ROS to end of path to use system/venv cv2 for Python3
if "python2.7" in sys.path[1] and "python2.7" in sys.path[2]:
    sys.path.append(sys.path.pop(1))
    sys.path.append(sys.path.pop(1))

import cv2


class Vision:

    def __init__(self):
        self.field_color_detector = color.ColorDetector('../config/fieldColor.yaml')  # Todo: set right path
        self.cascade = cv2.CascadeClassifier('../classifier/cascadeNew.xml')  # Todo: set path
        self.ball_classifier = live_classifier.LiveClassifier('../models/classifier_01')  # Todo: set path
        self.debug = False

        # ROS-Stuff:

        # publisher:
        self.pub_balls = rospy.Publisher("ball_in_image", BallsInImage, queue_size=1)
        # self.pub_lines = rospy.Publisher("line_in_image", LineInformationInImage, queue_size=5)

        # subscriber:
        self.bridge = CvBridge()
        rospy.Subscriber("image_raw", Image, self._image_callback, queue_size=1)
        rospy.init_node("bitbots_soccer_vision")

        if self.debug:
            rospy.logwarn("Debug windows are enabled")
        else:
            rospy.loginfo("Debug windows are disabled")

        rospy.spin()

    def _image_callback(self, img):
        self.handle_image(img)

    def handle_image(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")  # converting the ROS image message to CV2-image
        horizon_detector = horizon.HorizonDetector(image, self.field_color_detector)
        ball_finder = ball.BallFinder(image, self.cascade)
        # Todo: filter balls under horizon
        ball_classifier = classifier.Classifier(image, self.ball_classifier, ball_finder.get_candidates())
        print(horizon_detector.get_horizon_points())
        if self.debug:
            debug_image_dings = debug_image.DebugImage(image)
            debug_image_dings.draw_horizon(horizon_detector.get_horizon_points())
            debug_image_dings.draw_ball_candidates(horizon_detector.candidates_under_horizon(ball_finder.get_candidates(), 100))
            # debug_image_dings.imshow()
        if ball_classifier.get_top_candidate()[1] > 0.5:  # Todo: set real threshold
            msg = BallsInImage()
            msg.header.frame_id = image_msg.header.frame_id
            msg.header.stamp = image_msg.header.stamp
            msg.candidates.append(ball_classifier.get_top_candidate())
            self.pub_balls.publish(msg)



