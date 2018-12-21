#include "ORBworker.h"
#include "make_unique.h"

#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>


ORBWorker::ORBWorker(ros::NodeHandle &n) : nh_(n) {
  subImageRaw_ = nh_.subscribe("/camera/image_raw", 1, &ORBWorker::imageCallback, this);
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("/orb_odom", 5);
  mSlam_ = make_unique<ORB_SLAM2::System>(
      "Vocabulary/ORBvoc.txt", "calibration/KITTI00-02.yaml", 
      ORB_SLAM2::System::MONOCULAR, true);
}

void ORBWorker::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvShare(msg);
  if (cv_ptr == nullptr) return;
  mSlam_->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}


