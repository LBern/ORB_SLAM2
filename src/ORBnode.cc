#include <glog/logging.h>
#include <ros/ros.h>

#include "ORBworker.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ORB");
  ros::NodeHandle nodeHandle("~");

  try {
    ORBWorker worker(nodeHandle);
    ros::spin();
  } catch (const std::exception& e) {
    LOG(ERROR) << "Exception: " << e.what();
    return 1;
  } catch (...) {
    LOG(ERROR) << "Unknown Exception";
    return 1;
  }

  return 0;
}
