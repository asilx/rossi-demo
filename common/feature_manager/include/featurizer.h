#ifndef FEATURIZER_H_
#define FEATURIZER_H_

#include "string"
#include <signal.h> 

#include "cstdio" 

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//ros
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_ros/io/bag_io.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "sys/stat.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include "featurizer/NormalHistogram.hpp"
#include <pcl/visualization/histogram_visualizer.h>
#include "engine.h"
#include "featurizer/DataLogger.h"

// 47th feature is the object presence
#define featureCount 47

using namespace std;
namespace featurizer
{
  class Featurizer
  {
  public:
    string
    getRandomString (int i, bool isZenith);
    void
    run (const sensor_msgs::PointCloud2& cloud2);
    void
    getNoOfClusters (const sensor_msgs::PointCloud2ConstPtr& cloud2);
    void
    giveSurfaceHists (pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                      pcl::PointCloud<featurizer::NormalHistogram>::Ptr azimuths, pcl::PointCloud<
                          featurizer::NormalHistogram>::Ptr zeniths);
    Featurizer (ros::NodeHandle *nh3);

    // ++Onur
    Featurizer ();
    void
    extractFeatures (const sensor_msgs::PointCloud2& cloud2);
    double*
    getCurrentFeatureVector ();
    void
    setFeatureVector (pcl::PointCloud<featurizer::NormalHistogram>::Ptr &azimuths, pcl::PointCloud<
        featurizer::NormalHistogram>::Ptr &zeniths);
    double*
    extractAndGetFeatures (const sensor_msgs::PointCloud2& cloud2);

    void
    extractFeatures (const sensor_msgs::PointCloud2& cloud2, int objIndex, bool visualization);

    void
    extractFeatures(const sensor_msgs::PointCloud& cloud,int objIndex, bool visualization);
    vector<double*>
    extractAndVisualizeAll (const std::vector<sensor_msgs::PointCloud> &clusters);

    void setSizeAndPos(double posX, double posY, double posZ, double sizeX, double sizeY, double sizeZ);


	double* getEffectVector();
	bool isFinal();
	void setFinal(bool value);
	void setInitVector();

DataLogger *featureLogger;
DataLogger *effectLogger;
	//static int featureCount = 46;
    // --Onur
  private:
    //ros::NodeHandle* nh3;
    ros::Publisher *pub;
    ros::Publisher *pub2;
    ros::Publisher *pub3;
    ros::Subscriber *sub;
    ros::Subscriber *sub2;
   // pcl::visualization::PCLHistogramVisualizer *phv;
    pcl::PointCloud<featurizer::NormalHistogram>::Ptr azimuths;
    pcl::PointCloud<featurizer::NormalHistogram>::Ptr zeniths;
    int counter;
    int no_of_clusters;
	
    Engine* ep;
    // ++Onur: Storing the whole feature vector (now, only normal histograms in order: azimuth, zenith) in a double array.

    double currentFeatureVector[featureCount];
	double initialConditionFeatureVector[featureCount];
	double effectVector[featureCount];
    bool usePublications;
	bool objectInFinalCondition;

    // --Onur

  };
}
#endif 
