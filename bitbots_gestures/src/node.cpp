#include "node.hpp"

using namespace InferenceEngine;
using namespace human_pose_estimation;

HumanPoseEstimatorNode::HumanPoseEstimatorNode() {

  ros::NodeHandle nh;
  image_transport::ImageTransport it = image_transport::ImageTransport(nh);

  std::string package_path = ros::package::getPath("bitbots_gestures");
  HumanPoseEstimator
      estimator(package_path + "/models/human-pose-estimation-0001.xml", "MYRIAD", false);

  // pairs of connected keypoints
  const std::vector<std::pair<int, int>> pairs =
      {{1, 2}, {1, 5}, {2, 3}, {3, 4}, {5, 6}, {6, 7}, {1, 8}, {8, 9}, {9, 10}, {1, 11}, {11, 12}, {12, 13}, {1, 0},
       {0, 14}, {14, 16}, {0, 15}, {15, 17}};

  bool first_image = true;

  new_image = false;
  image_transport::Subscriber image_sub = it.subscribe("/image_raw", 1, &HumanPoseEstimatorNode::imageCb, this);
  ros::Publisher pose_pub = nh.advertise<bitbots_gestures::HumanPoseArray>("human_poses", 1);
  image_transport::Publisher debug_image_pub = it.advertise("/human_pose_debug", 1);

  bool publish_debug_image;
  nh.param<bool>("publish_debug_image", publish_debug_image, true);
  float score_threshold;
  nh.param<float>("score_threshold", score_threshold, 150);

  std::vector<HumanPose> poses;
  bitbots_gestures::HumanPoseArray pose_msg;
  pose_msg.header.frame_id = "camera_optical_frame";
  while (ros::ok()) {
    if (new_image) {
      // reshape once with data from first image. we assume that image size will not change later
      if (first_image) {
        first_image = false;
        estimator.reshape(cv_ptr_->image);
      }
      // put image into estimator
      estimator.frameToBlobCurr(cv_ptr_->image);
      new_image = false;
      estimator.startCurr();
      // get poses from estimator
      while (!estimator.readyCurr()) {
        sleep(0.1);
      }
      poses = estimator.postprocessCurr();
      ROS_WARN_STREAM(poses.size());
      std::vector<bitbots_gestures::HumanPoseEstimation>
          estimation_msgs = std::vector<bitbots_gestures::HumanPoseEstimation>();
      // convert to ROS message and publish
      for (unsigned int i = 0; i < poses.size(); i++) {
        if (poses[i].score > score_threshold) {
          bitbots_gestures::HumanPoseEstimation estimation_msg;
          std::vector<geometry_msgs::Point32> points = std::vector<geometry_msgs::Point32>();
          // iterate through all keypoints
          for (unsigned int j = 0; j < poses[i].keypoints.size(); j++) {
            geometry_msgs::Point32 point;
            point.x = poses[i].keypoints[j].x;
            point.y = poses[i].keypoints[j].y;
            points.push_back(point);
          }
          estimation_msg.keypoints = points;
          estimation_msg.score.data = poses[i].score;
          estimation_msgs.push_back(estimation_msg);
        }
      }
      pose_msg.poses = estimation_msgs;
      pose_msg.header.stamp = ros::Time::now();
      pose_pub.publish(pose_msg);
      if (publish_debug_image) {
        // draw detected poses
        for (unsigned int i = 0; i < poses.size(); i++) {
          if (poses[i].score > score_threshold) {
            // draw lines to connect keypoints
            for (unsigned int j = 0; j < pairs.size(); j++) {
              cv::Point from_point = poses[i].keypoints[pairs[j].first];
              cv::Point to_point = poses[i].keypoints[pairs[j].second];

              // dont draw it one point is 0,0, since this means it was not found
              if (from_point.x != -1 && to_point.x != -1) {
                cv::line(cv_ptr_->image,
                         from_point,
                         to_point,
                         CV_RGB(0, 0, 0),
                         3);
              }
            }
            // draw circles for all keypoints
            for (unsigned int j = 0; j < poses[i].keypoints.size(); j++) {
              cv::Point point = poses[i].keypoints[j];
              // dont take points at 0,0 since this means they were not found
              if (point.x != -1) {
                cv::circle(cv_ptr_->image, point, 3, CV_RGB(255, 0, 0), CV_FILLED);
              }
            }
            // put score as text
            cv::putText(cv_ptr_->image,
                        std::to_string(poses[i].score).substr(0, 5),
                        poses[i].keypoints[1],
                        cv::FONT_HERSHEY_DUPLEX,
                        1,
                        CV_RGB(0, 255, 0),
                        2);
          }
        }
      }
      debug_image_pub.publish(cv_ptr_->toImageMsg());
    }
    ros::spinOnce();
  }
}

void HumanPoseEstimatorNode::imageCb(const sensor_msgs::ImageConstPtr &msg) {
  cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  new_image = true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "human_pose");
  HumanPoseEstimatorNode node = HumanPoseEstimatorNode();
}
