#ifndef ORB_WORKER_H
#define ORB_WORKER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "System.h"

#include <memory>

class ORBWorker {

  public:
    explicit ORBWorker(ros::NodeHandle &n, std::string &mode);

  private:
    void setupMono();
    void setupStereo();
    void monoImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void stereoImageCallback(const sensor_msgs::ImageConstPtr& msgLeft,
        const sensor_msgs::ImageConstPtr& msgRight);
    void publishOdometry(const cv::Mat &Tcw, const ros::Time &time);

    ros::Publisher pubOdometry_;
    ros::Subscriber subImageRaw_;
    ros::NodeHandle nh_;

    std::unique_ptr<ORB_SLAM2::System> mSlam_;
};

#endif
