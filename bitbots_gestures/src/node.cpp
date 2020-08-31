#include "node.hpp"

using namespace InferenceEngine;
using namespace human_pose_estimation;

HumanPoseEstimatorNode::HumanPoseEstimatorNode() : it_(nh_) {
  HumanPoseEstimator
      estimator("/home/bitbots/wolfgang_ws/src/bitbots_vision/bitbots_gestures/models/", "deeevice", true);
  cv::Size image_size = cv::Size();
  estimator.inputWidthIsChanged(image_size);

  new_image = false;
  image_sub_ =
      it_.subscribe("/image_raw", 1, &HumanPoseEstimatorNode::imageCb, this);
  pose_pub_ = nh_.advertise<bitbots_gestures::HumanPoseArray>("human_poses", 1);
  debug_image_pub_ = it_.advertise("/human_pose_debug", 1);

  // todo make parameter
  bool publish_debug_image = true;

  std::vector<HumanPose> poses;
  bitbots_gestures::HumanPoseArray pose_msg;
  pose_msg.header.frame_id = "camera_optical_frame";
  while (ros::ok()) {
    if (new_image) {
      // put image into estimator
      estimator.frameToBlobCurr(cv_ptr_->image);
      new_image = false;
      // get poses from estimator
      if (estimator.readyCurr()) {
        poses = estimator.postprocessCurr();
      }
      std::vector<bitbots_gestures::HumanPoseEstimation>
          estimation_msgs = std::vector<bitbots_gestures::HumanPoseEstimation>();
      // convert to ROS message and publish
      for (unsigned int i = 0; i < poses.size(); i++) {
        bitbots_gestures::HumanPoseEstimation estimation_msg;
        std::vector<geometry_msgs::Point32> points = std::vector<geometry_msgs::Point32>();
        // iterate through all keypoints
        for (unsigned int j = 0; j < poses[i].keypoints.size(); j++) {
          geometry_msgs::Point32 point;
          point.x = poses[i].keypoints[j].x;
          point.y = poses[i].keypoints[j].y;
          points.push_back(point);
        }
        estimation_msg.score.data = poses[i].score;
        estimation_msgs.push_back(estimation_msg);
      }
      pose_msg.poses = estimation_msgs;
      pose_msg.header.stamp = ros::Time::now();
      pose_pub_.publish(pose_msg);
      if (publish_debug_image) {
        // draw circles for all keypoints
        for (unsigned int i = 0; i < poses.size(); i++) {
          for (unsigned int j = 0; j < poses[i].keypoints.size(); j++) {
            cv::circle(cv_ptr_->image, poses[i].keypoints[j], 10, CV_RGB(255, 0, 0));
          }
        }
      }
      debug_image_pub_.publish(cv_ptr_->toImageMsg());
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
