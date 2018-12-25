#ifndef ORB_WORKER_H
#define ORB_WORKER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


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

    typedef message_filters::sync_policies::ApproximateTime
       <sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    std::unique_ptr<message_filters::Synchronizer<sync_pol>> sync_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> leftSub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> rightSub_;

    ros::Publisher pubOdometry_;
    ros::Subscriber subImageRaw_;
    ros::NodeHandle nh_;

    std::unique_ptr<ORB_SLAM2::System> mSlam_;
};

#endif
