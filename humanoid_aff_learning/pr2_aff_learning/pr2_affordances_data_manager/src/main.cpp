/*
 * main.cpp
 *
 *  Created on: Aug 23, 2011
 *      Author: kadir
 */

#include "ros/ros.h"
#include "affordances_msgs/PerceiveRangeData.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  //  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber sub_pc;

  int cnt;
  pcl::PointCloud<pcl::PointXYZ> pc2_data;
  sensor_msgs::Image img;
  ros::Subscriber sub_aff;

public:
  ImageConverter () :
    it_ (nh_)
  {
    image_pub_ = it_.advertise ("/out", 1);
    //    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
    sub_pc = nh_.subscribe ("/swissranger/scan", 10, &ImageConverter::rangeCb, this);
    sub_aff = nh_.subscribe ("/affordances/get_range_data", 10, &ImageConverter::showRangeData, this);

    cvNamedWindow("before");
    cvNamedWindow("after");

//    cv::namedWindow( "before" );
//    cv::namedWindow( "after");
    cnt = 0;
  }

  ~ImageConverter ()
  {
//    cv::destroyWindow( WINDOW );
    cvDestroyAllWindows();
  }

  void
  run ()
  {
    while (nh_.ok ())
    {
      ros::spinOnce ();
    }
  }

  void
  saveRangeData ()
  {
    std::stringstream ss;
    ss << cnt;
    std::string file_name = ss.str ();
    file_name.append (".pcd");
    std::cout << "filename: " << file_name << std::endl;
    pcl::io::savePCDFileASCII (file_name, pc2_data);
    cnt++;
  }

  void
  convertPCToImg (const pcl::PointCloud<pcl::PointXYZ>& pc2_data, sensor_msgs::Image& img)
  {
    unsigned int n_points = pc2_data.points.size ();
    int n_cols = 176;
    int n_rows = n_points / n_cols;
    img.data.resize (n_points);
    img.width = n_cols;
    img.height = n_rows;
    //  for(unsigned int i=0;i<n_rows;i++)
    //    for(unsigned int j=0; j<n_cols; j++)
//    std::cout<<n_points<<std::endl;
    for (unsigned int k = 0; k < n_points; k++)
    {
      float x = pc2_data.points[n_points- k].x;
      float y = pc2_data.points[n_points- k].y;
      float z = pc2_data.points[n_points -k].z;
//      std::cout<<(sqrt (x * x + y * y + z * z)*255.0/5.0)<<"\t";
      img.data[k] = (uint8_t)(sqrt (x * x + y * y + z * z)*255.0/5.0);
//      std::cout<<img.data[k]<<"\t";
    }
//    std::cout<<std::endl;

    img.encoding.assign("mono8");

  }

  void
  showRangeData (const affordances_msgs::PerceiveRangeData::ConstPtr& aff_msg)
  {
    convertPCToImg (pc2_data, img);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy (img, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//      cv::circle (cv_ptr->image, cv::Point (50, 50), 10, CV_RGB(255,0,0));

    //cv::imwrite("asd.jpg",cv_ptr->image);
    if(cnt%2 == 0)
      cv::imshow ("before", cv_ptr->image);
//      cvShowImage("before",cv_ptr->image);
    else
      cv::imshow ("after", cv_ptr->image);
//      cvShowImage("after",cv_ptr->image);
    saveRangeData ();
    cv::waitKey (1);

//    image_pub_.publish (cv_ptr->toImageMsg ());
  }

  void
  rangeCb (const sensor_msgs::PointCloud::ConstPtr& pc)
  {
    sensor_msgs::PointCloud2 pc_2;
    sensor_msgs::convertPointCloudToPointCloud2 (*pc, pc_2);
    pcl::fromROSMsg (pc_2, pc2_data);

    std::cout << "got range data" << std::endl;
  }

  //  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  //  {
  //    cv_bridge::CvImagePtr cv_ptr;
  //    try
  //    {
  //      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  //    }
  //    catch (cv_bridge::Exception& e)
  //    {
  //      ROS_ERROR("cv_bridge exception: %s", e.what());
  //      return;
  //    }
  //
  //    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  //      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
  //
  //    cv::imshow(WINDOW, cv_ptr->image);
  //    cv::waitKey(3);
  //
  //    image_pub_.publish(cv_ptr->toImageMsg());
  //  }
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "data_manager");
  ImageConverter ic;
  ic.run();

  return 0;
}
