/*  Copyright (C) 2011, Kadir Firat Uyanik
 *    Middle East Technical University, Kovan Research Lab
 *    kadir@ceng.metu.edu.tr
 *
 *    http://kovan.ceng.metu.edu.tr/~kadir
 *
 *  FeatureFactory.h is part of feature_factory.
 *
 *  feature_factory is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  feature_factory is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with feature_factory. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FEATUREFACTORY_H_
#define FEATUREFACTORY_H_

#include "feature_factory/FeaturePoint.hpp"
#include "feature_factory/FeatureCalculation.h"

// stl includes
#include "queue"

// ROS include
#include "ros/ros.h"
#include <pcl/ros/conversions.h>
#include "pcl/io/pcd_io.h"
//#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/radius_outlier_removal.h"

//#include "sensor_msgs/fill_image.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <image_transport/image_transport.h>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
//#include "cv_bridge/CvBridge.h"
//#include <opencv/cv.h>
//#include <opencv/highgui.h>

#define RANGE_IMAGE_WIDTH 176
#define RANGE_IMAGE_HEIGHT 144

#define PI 3.141214

namespace feature_factory
{

  enum FeatureType
  {
    DEPTH, CONF, AMP, NORMAL_AZI, NORMAL_ZEN
  };

  class FeatureFactory
  {
  public:
    FeatureFactory (const ros::NodeHandle& nh_, int const& grid_edge_size = GRID_EDGE_SIZE);
    virtual
    ~FeatureFactory ();

    void
    extractRangeFeature (cv_bridge::CvImagePtr cv_ptr, pcl::PointCloud<feature_factory::FeaturePoint<
        RANGE_FEATURES_HISTOGRAM> > &range_features, FeatureType feature_type = DEPTH);

    void
    extractSurfaceFeatures (pcl::PointCloud<pcl::Normal>::Ptr pc2_normals, pcl::PointCloud<
        feature_factory::FeaturePoint<SURFACE_FEATURES_HISTOGRAM> > &surface_zen_features, pcl::PointCloud<
        feature_factory::FeaturePoint<SURFACE_FEATURES_HISTOGRAM> > &surface_azi_features);

    bool
    extractFeatures ();

    void
    saveFeatures ();

    void
    sendFeatures ();

    bool
    run ();

  private:

    uint n_features_;

    bool is_sim_;
    ros::NodeHandle n;
    image_transport::ImageTransport n_it_img_d, n_it_img_a, n_it_img_c;
    image_transport::Subscriber sub_img_d;
    image_transport::Subscriber sub_img_a;
    image_transport::Subscriber sub_img_c;
    ros::Subscriber sub_pc;
    ros::Subscriber sub_pc2;

    ros::Publisher pub_pc2;
    ros::Publisher pub_features;
    //    ros::Publisher pub_dist_features;
    //    ros::Publisher pub_conf_features;
    //    ros::Publisher pub_amp_features;
    //    ros::Publisher pub_surf_features;

    // TODO: Don't try to use img_d as mono_8 but mono_16 since depth information is important
    // and we don't want to lose precision.
    sensor_msgs::Image img_d, img_a, img_c;

    sensor_msgs::PointCloud2 pc2_;
    std::queue<sensor_msgs::Image> img_d_buffer;
    std::queue<sensor_msgs::Image> img_a_buffer;
    std::queue<sensor_msgs::Image> img_c_buffer;
    std::queue<sensor_msgs::PointCloud2> pc2_buffer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_data;

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr pc2_normals;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree;

    std::vector<float> grid_data;
    int grid_edge_size;

    pcl::PointCloud<feature_factory::FeaturePoint<RANGE_FEATURES_HISTOGRAM> > distances;
    pcl::PointCloud<feature_factory::FeaturePoint<RANGE_FEATURES_HISTOGRAM> > confidences;
    pcl::PointCloud<feature_factory::FeaturePoint<RANGE_FEATURES_HISTOGRAM> > amplitudes;

    pcl::PointCloud<feature_factory::FeaturePoint<SURFACE_FEATURES_HISTOGRAM> > normals_azi;
    pcl::PointCloud<feature_factory::FeaturePoint<SURFACE_FEATURES_HISTOGRAM> > normals_zen;

    bool
    calcFeaturesCallBack (FeatureCalculation::Request& request, FeatureCalculation::Response& response);
    void
    pc2RcvdCallBack (const sensor_msgs::PointCloud2::ConstPtr& pc_2);
    void
    pcRcvdCallBack (const sensor_msgs::PointCloud::ConstPtr& pc);
    void
    distanceRcvdCallback (const sensor_msgs::ImageConstPtr& image_d);
    void
    amplitudeRcvdCallback (const sensor_msgs::ImageConstPtr& image_a);
    void
    confidenceRcvdCallback (const sensor_msgs::ImageConstPtr& image_c);

    bool
    convertPointCloud2ToDistImg (const sensor_msgs::PointCloud2& pc_2, cv::Mat& dist_img);

    void
    cleanFeatures ();

  };

}

#endif /* FEATUREFACTORY_H_ */
