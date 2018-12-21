#include <glog/logging.h>
#include <ros/ros.h>
#include <string>

#include "ORBworker.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ORB");
  ros::NodeHandle nodeHandle("~");

  try {
    std::string mode;
    if (argc <= 1) 
      mode = "mono";
    else 
      mode = argv[1];

    ORBWorker worker(nodeHandle, mode);

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
