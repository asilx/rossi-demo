/*
 * FeatureExtractor.h
 *
 *  Created on: Feb 9, 2011
 *      Author: kadir
 */

#ifndef FEATUREEXTRACTOR_H_
#define FEATUREEXTRACTOR_H_

#include "iostream"
#include "queue"
#include "vector"

// ROS include
#include "ros/ros.h"
#include <pcl/ros/conversions.h>

#include <image_transport/image_transport.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/features/normal_3d.h>

#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/radius_outlier_removal.h"

#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "feature_extractor/FeaturePoint.h"
//#include "feature_extractor/Features.h"

#define GRID_EDGE_SIZE 16

namespace feature_extractor
{

  class FeatureExtractor
  {
  private:

    ros::NodeHandle n;
    image_transport::ImageTransport n_it_img_d, n_it_img_a, n_it_img_c;
    image_transport::Subscriber sub_img_d;
    image_transport::Subscriber sub_img_a;
    image_transport::Subscriber sub_img_c;
    ros::Subscriber sub_pc2;

    sensor_msgs::Image img_d, img_a, img_c/*, img_d_16*/;
    sensor_msgs::PointCloud2 pc2;
    std::queue<sensor_msgs::Image> img_d_buffer;
    std::queue<sensor_msgs::Image> img_a_buffer;
    std::queue<sensor_msgs::Image> img_c_buffer;
    std::queue<sensor_msgs::PointCloud2> pc2_buffer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_data;
    sensor_msgs::CvBridge bridge;
    IplImage* dist_data;
    IplImage* conf_data;
    IplImage* amp_data;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr pc2_normals;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree;

    std::vector<float> grid_data;
    int grid_edge_size;

    pcl::PointCloud<feature_extractor::Feature<RANGE_FEATURES_HISTOGRAM> > distances;
    pcl::PointCloud<feature_extractor::Feature<RANGE_FEATURES_HISTOGRAM> > confidences;
    pcl::PointCloud<feature_extractor::Feature<RANGE_FEATURES_HISTOGRAM> > intensities;

    pcl::PointCloud<feature_extractor::Feature<SURFACE_FEATURES_HISTOGRAM> > normals_azi;
    pcl::PointCloud<feature_extractor::Feature<SURFACE_FEATURES_HISTOGRAM> > normals_zen;

    Features features_msg;
    sensor_msgs::PointCloud2 distances_msg;
    sensor_msgs::PointCloud2 confidences_msg;
    sensor_msgs::PointCloud2 intensities_msg;
    sensor_msgs::PointCloud2 normals_azi_msg;
    sensor_msgs::PointCloud2 normals_zen_msg;

    ros::Publisher features_pub;
    ros::Publisher surface_normals_pub;
    sensor_msgs::PointCloud2 pc_2_normals_msg;
  public:

    FeatureExtractor (const ros::NodeHandle& nh_, int const& grid_edge_size = GRID_EDGE_SIZE);

    void
    Extract ();
    void
    Publish ();
    void
    ExtractRangeFeature (IplImage* range_data,
                         pcl::PointCloud<feature_extractor::Feature<RANGE_FEATURES_HISTOGRAM> > &range_features);
    void
    ExtractAllSurfaceFeatures ();
    bool
    spin ();

    void
    pc2RcvdCallback (const sensor_msgs::PointCloud2::ConstPtr& pc_2);
    void
    distanceRcvdCallback (const sensor_msgs::ImageConstPtr& image_d);
    void
    amplitudeRcvdCallback (const sensor_msgs::ImageConstPtr& image_a);
    void
    confidenceRcvdCallback (const sensor_msgs::ImageConstPtr& image_c);
  };
}
#endif /* FEATUREEXTRACTOR_H_ */
