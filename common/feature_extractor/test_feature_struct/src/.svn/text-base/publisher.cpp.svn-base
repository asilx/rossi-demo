/*
 * publisher.cpp
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

using namespace std;

int main(int argc, char** argv){

	ros::init(argc, argv, "publisher");
	ros::NodeHandle nh;
	ros::Publisher pub;
	pub = nh.advertise<sensor_msgs::PointCloud2>("feature_vector",1000);
	sensor_msgs::PointCloud2 feature_vectors_msg;

	pcl::PointCloud<pcl::Feature<SURFACE_FEATURES_HISTOGRAM> > feature_vectors;

	std::vector<float> data;

	for(unsigned int i =0;i<100;i++){

		for(unsigned int j=0;j<data.size();j++)
			data.pop_back();
		data.clear();

		for(unsigned int j=i*10;j<i*10+10;j++)
			data.push_back(j);

		pcl::Feature<SURFACE_FEATURES_HISTOGRAM> f;
		f.calculate(data,true,0,1000);
		//f.print();
		feature_vectors.push_back(f);
	}
	pcl::toROSMsg(feature_vectors,feature_vectors_msg);

	while(nh.ok()){
		pub.publish(feature_vectors_msg);
		ros::spinOnce();
	}
	return 0;
}
