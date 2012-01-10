#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "test_setup_model_publisher" );
  ros::NodeHandle n;

  ros::Publisher joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);
  ros::Rate loop_rate(15);
  sensor_msgs::JointState js;

	js.name.push_back(std::string("/swissranger_link"));
	js.position.push_back(0);

	while( n.ok() )
	{
    js.header.frame_id="/base_link";
    js.header.stamp = ros::Time::now();
		joint_state_publisher.publish(js);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
