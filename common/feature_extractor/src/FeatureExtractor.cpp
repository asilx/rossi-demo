/*
 * FeatureExtractor.cpp
 *
 *  Created on: Feb 9, 2011
 *      Author: kadir
 */

#include "feature_extractor/FeatureExtractor.h"
using namespace feature_extractor;

void
nullDeleter (void*)
{
}

FeatureExtractor::FeatureExtractor (const ros::NodeHandle& nh_, int const& grid_edge_size_) :
  n (nh_), n_it_img_d (n), n_it_img_a (n), n_it_img_c (n)
{

  grid_edge_size = grid_edge_size_;
  grid_data.resize (grid_edge_size * grid_edge_size);

  pc2_data = (pcl::PointCloud<pcl::PointXYZ>::Ptr)new pcl::PointCloud<pcl::PointXYZ> ();
  pc2_normals = (pcl::PointCloud<pcl::Normal>::Ptr)new pcl::PointCloud<pcl::Normal> ();

  tree = (pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr)new pcl::KdTreeFLANN<pcl::PointXYZ> ();
  ne.setSearchMethod (tree);
  ne.setKSearch (2);

  sub_img_d = n_it_img_d.subscribe ("/swissranger/distance/image_raw", 10, &FeatureExtractor::distanceRcvdCallback,
                                    this);
  sub_img_c = n_it_img_c.subscribe ("/swissranger/confidence/image_raw", 10, &FeatureExtractor::distanceRcvdCallback,
                                    this);
  sub_img_a = n_it_img_a.subscribe ("/swissranger/intensity/image_raw", 10, &FeatureExtractor::amplitudeRcvdCallback,
                                    this);
  sub_pc2 = n.subscribe ("/swissranger/pointcloud2_raw", 10, &FeatureExtractor::confidenceRcvdCallback, this);

  features_pub = n.advertise<feature_extractor::Features> ("features", 10);
}

void
FeatureExtractor::ExtractRangeFeature (IplImage* range_data, pcl::PointCloud<feature_extractor::Feature<
    RANGE_FEATURES_HISTOGRAM> > &range_features)
{
  int x = 0;
  int y = 0;

  for (int i = 0; i < 144 / grid_edge_size; i++)
  {
    for (int j = 0; j < 176 / grid_edge_size; j++)
    {
      feature_extractor::Feature<RANGE_FEATURES_HISTOGRAM> range_feature;
      for (int k = 0; k < grid_edge_size; k++)
      {
        for (int l = 0; l < grid_edge_size; l++)
        {
          x = j * grid_edge_size + l;
          y = i * grid_edge_size + k;
          grid_data[k * grid_edge_size + l] = (float)((uchar*)(dist_data->imageData + dist_data->widthStep * y))[x
              * dist_data->nChannels];
        }
      }
      range_feature.calculate (grid_data, 0, 255);
      range_features.push_back (range_feature);
    }
  }
}

void
FeatureExtractor::ExtractAllSurfaceFeatures ()
{
  int x = 0;
  int y = 0;
  std::vector<float> grid_data_x (grid_edge_size * grid_edge_size);
  std::vector<float> grid_data_y (grid_edge_size * grid_edge_size);
  std::vector<float> grid_data_z (grid_edge_size * grid_edge_size);

  for (int i = 0; i < 144 / grid_edge_size; i++)
  {
    for (int j = 0; j < 176 / grid_edge_size; j++)
    {
      feature_extractor::Feature<SURFACE_FEATURES_HISTOGRAM> surface_feature;
      for (int k = 0; k < grid_edge_size; k++)
      {
        for (int l = 0; l < grid_edge_size; l++)
        {
          x = j * grid_edge_size + l;
          y = i * grid_edge_size + k;
          grid_data_x[k * grid_edge_size + l] = pc2_normals->points[y * 176 + 144].normal[0];
          grid_data_y[k * grid_edge_size + l] = pc2_normals->points[y * 176 + 144].normal[1];
          grid_data_z[k * grid_edge_size + l] = pc2_normals->points[y * 176 + 144].normal[2];
        }
      }
      //TODO: convert normal information to zimuth-zenith representation here
      //and add the proper data to the following structures
      surface_feature.calculate (grid_data_x, -1, 1);
      normals_azi.push_back (surface_feature);

      surface_feature.calculate (grid_data_y, -1, 1);
      normals_zen.push_back (surface_feature);
    }
  }
}

void
FeatureExtractor::Extract ()
{

  if (img_d_buffer.size () > 0 && img_a_buffer.size () > 0 && img_c_buffer.size () > 0 && pc2_buffer.size () > 0)
  {

    sensor_msgs::PointCloud2 _pc2 = pc2_buffer.front ();
    sensor_msgs::Image _img_d = img_d_buffer.front ();
    sensor_msgs::Image _img_a = img_a_buffer.front ();
    sensor_msgs::Image _img_c = img_c_buffer.front ();

    pcl::fromROSMsg (_pc2, *pc2_data);
    //pcl::fromROSMsg(pc_2, pc_2_data);
    //sensor_msgs::Image::ConstPtr img_d_ptr = &img_d;

    boost::shared_ptr<sensor_msgs::Image> d_ptr (&_img_d, nullDeleter);
    dist_data = bridge.imgMsgToCv (d_ptr, "mono8");
    //cvShowImage("distance",dist_data);

    boost::shared_ptr<sensor_msgs::Image> c_ptr (&_img_c, nullDeleter);
    conf_data = bridge.imgMsgToCv (c_ptr, "mono8");
    //cvShowImage("confidence",conf_data);

    boost::shared_ptr<sensor_msgs::Image> a_ptr (&_img_a, nullDeleter);
    amp_data = bridge.imgMsgToCv (a_ptr, "mono8");
    //cvShowImage("intensity",amp_data);

    //*pc_2_data_ptr = pc_2_data;
    ne.setInputCloud (pc2_data);
    ne.compute (*pc2_normals);

    ExtractRangeFeature (dist_data, distances);
    ExtractRangeFeature (conf_data, confidences);
    ExtractRangeFeature (amp_data, intensities);
    ExtractAllSurfaceFeatures ();

    pc2_buffer.pop ();
    img_d_buffer.pop ();
    img_a_buffer.pop ();
    img_c_buffer.pop ();
  }
}

bool
FeatureExtractor::spin ()
{
  std::cout << "going to ros::ok() while" << std::endl;

  while (n.ok ())
  {
    Extract ();
    Publish ();
    ros::spinOnce ();
  }
  return true;
}

void
FeatureExtractor::Publish ()
{

  if (features_pub.getNumSubscribers () > 0)
  {

    pcl::toROSMsg (distances, distances_msg);
    pcl::toROSMsg (confidences, confidences_msg);
    pcl::toROSMsg (intensities, intensities_msg);
    pcl::toROSMsg (normals_azi, normals_azi_msg);
    pcl::toROSMsg (normals_zen, normals_zen_msg);

    features_msg.distances = distances_msg;
    features_msg.confidences = confidences_msg;
    features_msg.intensities = intensities_msg;
    features_msg.normals_azi = normals_azi_msg;
    features_msg.normals_zen = normals_zen_msg;

    features_pub.publish (features_msg);
  }
}

void
FeatureExtractor::pc2RcvdCallback (const sensor_msgs::PointCloud2::ConstPtr& pc_2)
{
  pc2 = *pc_2;
  pcl::fromROSMsg (pc2, *pc2_data);
  pc2_buffer.push (pc2);
}
void
FeatureExtractor::distanceRcvdCallback (const sensor_msgs::ImageConstPtr& image_d)
{
  img_d = *image_d;
  img_d_buffer.push (img_d);
}
void
FeatureExtractor::amplitudeRcvdCallback (const sensor_msgs::ImageConstPtr& image_a)
{
  img_a = *image_a;
  img_a_buffer.push (img_a);
}
void
FeatureExtractor::confidenceRcvdCallback (const sensor_msgs::ImageConstPtr& image_c)
{
  img_c = *image_c;
  img_c_buffer.push (img_c);
}

