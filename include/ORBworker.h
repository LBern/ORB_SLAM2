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
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    ros::Publisher pubOdometry_;
    ros::Subscriber subImageRaw_;
    ros::NodeHandle nh_;

    std::unique_ptr<ORB_SLAM2::System> mSlam_;
};

#endif
