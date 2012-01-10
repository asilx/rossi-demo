/*
 * subscriber.cpp
 *
 *  Created on: Mar 24, 2011
 *      Author: Kadir Firat Uyanik
 *      		www.kadiruyanik.wordpress.com
 */

#include "iostream"
#include "vector"
#include "test_feature_struct/FeaturePoint.h"
#include "ros/ros.h"
#include "pcl/point_types.h"
#include <pcl/ros/conversions.h>


pcl::PointCloud<pcl::Feature<SURFACE_FEATURES_HISTOGRAM> >::Ptr feature_vectors_ptr;
sensor_msgs::PointCloud2 feature_vectors_msg;

void callback(const sensor_msgs::PointCloud2::ConstPtr& pc_2 ){
	feature_vectors_msg = *pc_2;
	pcl::fromROSMsg(feature_vectors_msg,*feature_vectors_ptr);
	cout<<"*********"<<endl;
	for(unsigned int i =0;i<feature_vectors_ptr->points.size();i++)
		feature_vectors_ptr->points[i].print();
}

int main(int argc, char** argv){

	feature_vectors_ptr
	= (pcl::PointCloud<pcl::Feature<SURFACE_FEATURES_HISTOGRAM> >::Ptr)
	new pcl::PointCloud<pcl::Feature<SURFACE_FEATURES_HISTOGRAM> >();

	ros::init(argc, argv, "subscriber");
	ros::NodeHandle nh;
	ros::Subscriber sub;
	sub = nh.subscribe("feature_vector", 10, &callback);

	while(nh.ok())
		ros::spinOnce();
	return 0;
}
