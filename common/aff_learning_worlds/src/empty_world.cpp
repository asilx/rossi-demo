#include <ros/ros.h>
#include "tf/transform_broadcaster.h"

int
main (int argc, char* argv[])
{
  ros::init (argc, argv, "empty_world_tf_broadcaster");
  ros::NodeHandle n;
  
  ros::Rate loop_rate (100);
  
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(1.0, 0.0, 1.5) );
  transform.setRotation( tf::createQuaternionFromRPY(0, 1.57, 0) );
  
  while (n.ok ())
  {
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/swissranger_link"));
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
