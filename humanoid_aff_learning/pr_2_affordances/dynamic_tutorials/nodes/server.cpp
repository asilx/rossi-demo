#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorials/TutorialsConfig.h>

void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
ROS_INFO("In reconfigure callback");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_tutorials");

  dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
  dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
