#include <glog/logging.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ORB");
  ros::NodeHandle node_handle("~");

  std::cout << "running ORB odometry" << std::endl;

  try {
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
