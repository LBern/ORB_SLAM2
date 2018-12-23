#include "ORBworker.h"
#include "make_unique.h"

#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>

ORBWorker::ORBWorker(ros::NodeHandle &n, std::string &mode) : nh_(n) {
  if (mode == "mono")
    setupMono();
  else if (mode == "stereo")
    setupStereo();
}

void ORBWorker::setupMono() {
  subImageRaw_ = nh_.subscribe("/camera/image_raw", 1, 
      &ORBWorker::monoImageCallback, this);
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("/orb_odom", 5);
  mSlam_ = make_unique<ORB_SLAM2::System>(
      "Vocabulary/ORBvoc.txt", "calibration/KITTI00-02-MONO.yaml", 
      ORB_SLAM2::System::MONOCULAR, true);
}

void ORBWorker::setupStereo() {

  message_filters::Subscriber<sensor_msgs::Image> left_sub(
      nh_, "/camera/left/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> right_sub(
      nh_, "camera/right/image_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(
      sync_pol(10), left_sub, right_sub);
  sync.registerCallback(boost::bind(
        &ORBWorker::stereoImageCallback, this, _1, _2));

  mSlam_ = make_unique<ORB_SLAM2::System>(
      "Vocabulary/ORBvoc.txt", "calibration/KITTI00-02-STEREO.yaml", 
      ORB_SLAM2::System::STEREO, true);
}

void ORBWorker::monoImageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvShare(msg);
  if (cv_ptr == nullptr) return;
  const cv::Mat Tcw = mSlam_->TrackMonocular(
      cv_ptr->image, cv_ptr->header.stamp.toSec());
  publishOdometry(Tcw, msg->header.stamp);
}

void ORBWorker::stereoImageCallback(const sensor_msgs::ImageConstPtr& msgLeft,
    const sensor_msgs::ImageConstPtr& msgRight) {
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  cv_bridge::CvImageConstPtr cv_ptrRight;
  cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  cv_ptrRight = cv_bridge::toCvShare(msgRight);

  if (cv_ptrLeft == nullptr || cv_ptrRight == nullptr) return;
  mSlam_->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
}

void ORBWorker::publishOdometry(const cv::Mat &Tcw, const ros::Time &time) {
  if (Tcw.empty()) return;
  const cv::Mat rot = -Tcw.rowRange(0,3).colRange(0,3).t();
  const cv::Mat pos = rot * Tcw.rowRange(0,3).col(3);
  const float yaw = std::atan2(rot.at<float>(0,2), 
      std::sqrt(rot.at<float>(0,0) * rot.at<float>(0,0) 
        + rot.at<float>(0,1) * rot.at<float>(0,1)));

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = pos.at<float>(2,0);
  odom.pose.pose.position.y = -pos.at<float>(0,0);
  odom.pose.pose.position.z = pos.at<float>(1,0);
  odom.pose.pose.orientation = odom_quat;

  pubOdometry_.publish(odom);
}


