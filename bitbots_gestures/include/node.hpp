#pragma once

#include <vector>
#include <vector>
#include <inference_engine.hpp>
#include <ocv_common.hpp>

#include "human_pose_estimator.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include "bitbots_gestures/HumanPoseEstimation.h"
#include "bitbots_gestures/HumanPoseArray.h"


namespace human_pose_estimation {
class HumanPoseEstimatorNode {
 public:
  HumanPoseEstimatorNode();

 private:
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

  image_transport::ImageTransport it_;
  ros::NodeHandle nh_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher debug_image_pub_;

  ros::Publisher pose_pub_;
  cv_bridge::CvImagePtr cv_ptr_;
  bool new_image;
};
}  // namespace human_pose_estimation
