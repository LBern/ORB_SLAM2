#ifndef ORB_WORKER_H
#define ORB_WORKER_H

#include <ros/ros.h>

class ORBWorker {

  public:
    explicit ORBWorker::ORBWorker(ros::NodeHandle &n);

  private:
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    ros::NodeHandle nh_;
};

#endif
