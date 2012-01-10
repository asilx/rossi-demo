/*
 * feature_manager subscribes to a pointcloud topic (either PointCloud
 * or PointCloud type when the environment is simulation and real
 * respectively), calls a tabletop segmentation service. then calls
 * a feature extraction service.
 */

#include "iostream"
#include "math.h"

#include "ros/ros.h"
#include <ros/console.h>
#include "tf/transform_listener.h"

#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"

#include "tabletop_object_detector/TabletopSegmentation.h"
#include "tabletop_object_detector/TabletopDetectionResult.h"
#include <visualization_msgs/MarkerArray.h>

#include "pcl/visualization/histogram_visualizer.h"
#include "pcl_ros/features/normal_3d_omp.h"
#include "pcl_ros/filters/statistical_outlier_removal.h"
#include "pcl_ros/filters/radius_outlier_removal.h"
#include "pcl_ros/filters/passthrough.h"
#include <pcl/ros/conversions.h>

//aff_learning includes
//#include "aff_msgs/WorkspaceDetection.h"
#include "aff_msgs/ModuleStates.h"
#include "aff_msgs/ExperimentState.h"
#include "aff_msgs/Features.h"
#include "feature_manager/Perception.h"

#include "featurizer.h"
#include "DataLogger.h"

const std::string WORKSPACE_SEGMENTATION_SRV_NAME = "/tabletop_segmentation";
const uint16_t RANGE_IMAGE_HEIGHT = 144;
const uint16_t RANGE_IMAGE_WIDTH = 176;
const double DIFFERENCE_FACTOR = 0.1;
const double VOLUME_FACTOR = 0.1;
const bool filter_point_cloud = true;
const bool filter_not_table_top_objects = true;

const float object_center[] = {-0.3, -0.2, 0.0};
const float object_dims[] = {0.06, 0.06, 0.06};

// ++Onur

// ++Asil: Modified by
typedef struct clusterInfo
{
  double centroid[3];
  double bb_size[3];
  double mins[3];
  double maxs[3];
  int numberOfPoints;
  int index;
  int oldIndex;
  bool matched;
} ClusterInfo;

typedef struct indexedClusters
{
  std::vector<sensor_msgs::PointCloud> clusters;
  std::vector<int> cluster_indices;
} IndexedClusters;

int clusterCount;
std::vector<ClusterInfo> clusterIdentifications;

bool firstTime = false;
bool is_service_called = false;

// --Onur

ros::NodeHandle* nh;
visualization_msgs::MarkerArray bounding_boxes;
visualization_msgs::MarkerArray bb_texts;

ros::Publisher marker_array_pub;
ros::Publisher pub_bb_texts;
//ros::Publisher pub_all_clusters;
ros::Publisher pub_tabletop_detection;
ros::Publisher pub_tabletop_3d_features_;
ros::Publisher pub_pointcloud_raw;
ros::ServiceServer srv_perception;
std::vector<ros::Publisher> pubs_clusters;
ros::Subscriber sub_pointcloud;
tf::TransformListener* tf_listener;
tf::StampedTransform tf_transform;

ros::ServiceClient object_segmentation_srv;
tabletop_object_detector::TabletopSegmentation segmentation_call;
pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
pcl::RadiusOutlierRemoval<pcl::PointXYZ> cluster_outrem;
pcl::PassThrough<pcl::PointXYZ> pass_through_x;
pcl::PassThrough<pcl::PointXYZ> pass_through_y;
pcl::PassThrough<pcl::PointXYZ> pass_through_z;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_data_;
sensor_msgs::PointCloud2 pc2_;
uint8_t exp_state_;

IndexedClusters indexed_clusters;

//ros::NodeHandle nhfeat("~");
featurizer::Featurizer *featurize;//(nhfeat);

ros::AsyncSpinner* spinner_;

bool sim = false;

bool module_activation = true;
bool pointcloud_rcvd = false;

// ++ Onur

void
getMinMax (const pcl::PointCloud<pcl::PointXYZ>& cloud, double &xMin, double &yMin, double &zMin, double &xMax,
           double &yMax, double &zMax)
{
  xMin = DBL_MAX;
  yMin = DBL_MAX;
  zMin = DBL_MAX;
  xMax = DBL_MIN;
  yMax = DBL_MIN;
  zMax = DBL_MIN;

  for (uint i = 0; i < cloud.points.size (); i++)
  {
    if (isnan (cloud.points[i].x) || isnan (cloud.points[i].y) || isnan (cloud.points[i].z))
      continue;

    xMax = max (xMax, (double)(cloud.points[i].x));
    yMax = max (yMax, (double)(cloud.points[i].y));
    zMax = max (zMax, (double)(cloud.points[i].z));

    xMin = min (xMin, (double)(cloud.points[i].x));
    yMin = min (yMin, (double)(cloud.points[i].y));
    zMin = min (zMin, (double)(cloud.points[i].z));
  }
}

void
publishClustersOneByOne (const IndexedClusters& rc)//std::vector<sensor_msgs::PointCloud>& clusters)
{
  if (rc.clusters.size () != pubs_clusters.size ())
  {
    pubs_clusters.resize (rc.clusters.size ());

    for (uint i = 0; i < pubs_clusters.size (); i++)
    {
      std::stringstream s;
      std::string cluster_name ("clusters_");

      // some of the previously identified clusters might not exist, hence the caution
      //s << clusterIdentifications[i].oldIndex;
      s << rc.cluster_indices[i];//i;
      cluster_name.append (s.str ());
      pubs_clusters[i] = ros::Publisher (nh->advertise<sensor_msgs::PointCloud> (cluster_name, 10));
    }
  }
  for (uint i = 0; i < pubs_clusters.size (); i++)
  {
    if (pubs_clusters[i].getNumSubscribers () > 0)
      //      pubs_clusters[i].publish (clusters[clusterIdentifications[i].index]);
      pubs_clusters[i].publish (rc.clusters[i]);
  }
}

int
assignClusterIndex (int new_index)
{

  bool occupied_old_indices[clusterIdentifications.size ()];

  for (int i = 0; i < clusterIdentifications.size (); i++)
  {
    occupied_old_indices[i] = false;

  }

  for (unsigned int index = 0; index < clusterIdentifications.size (); index++)
  {
    if (occupied_old_indices[clusterIdentifications[index].oldIndex] > -1)
      occupied_old_indices[clusterIdentifications[index].oldIndex] = true;

  }

  //cout << "here" << endl;

  int i = 0;
  while (0 < clusterIdentifications.size () && occupied_old_indices[i] == true && i < clusterIdentifications.size ())
    i++;

  //cout << "here2" << endl;
  return i;

  /*bool exists = false;
   int assignedIndex = new_index;

   for (unsigned int index = 0; index < clusterIdentifications.size (); index++)
   {
   if (clusterIdentifications[index].oldIndex == assignedIndex || exists) // this index was previously
   {
   exists = true;

   // Now, find the largest unoccuppied index value.
   if (assignedIndex < clusterIdentifications[index].oldIndex) // larger than this, please.
   assignedIndex = clusterIdentifications[index].oldIndex + 1;

   }
   }

   if (!exists)
   {
   return new_index;
   }
   else
   {
   return assignedIndex;
   }*/
}

void
getCentroid (const sensor_msgs::PointCloud &cloud, int cloudSize, double &xCent, double &yCent, double &zCent)
{
  double xSum = 0, ySum = 0, zSum = 0;

  for (int j = 0; j < cloudSize; j++)
  {
    xSum += cloud.points[j].x;
    ySum += cloud.points[j].y;
    zSum += cloud.points[j].z;
  }

  xCent = xSum / cloudSize;
  yCent = ySum / cloudSize;
  zCent = zSum / cloudSize;
}

bool
closeEnough (ClusterInfo* info, int curNumberOfPoints, double xCent, double yCent, double zCent)
{

  if (info->matched) // dismiss the previously matched item!
  {
    return false;
  }

  double countDifference = 1.0 * abs (info->numberOfPoints - curNumberOfPoints);
  double *centroid = info->centroid;

  if (countDifference > DIFFERENCE_FACTOR * info->numberOfPoints)
  {
    return false;
  }

  double xDiff = abs (centroid[0] - xCent), yDiff = abs (centroid[1] - yCent), zDiff = abs (centroid[2] - zCent);

  return (xDiff <= DIFFERENCE_FACTOR * centroid[0]) && (yDiff <= DIFFERENCE_FACTOR * centroid[1]) && (zDiff
      <= DIFFERENCE_FACTOR * centroid[2]);
}

//++Asil: the likeliness check for moved object
bool
likelinessCheck (ClusterInfo* info, double box_x, double box_y, double box_z)
{

  if (info->matched) // dismiss the previously matched item!
  {
    return false;
  }

  //double countDifference = 1.0 * abs (info->numberOfPoints - curNumberOfPoints);
  double *size = info->bb_size;

  /*if (countDifference > DIFFERENCE_FACTOR * info->numberOfPoints)
   {
   return false;
   }*/

  double xDiff = abs (size[0] - box_x), yDiff = abs (size[1] - box_y), zDiff = abs (size[2] - box_z);

  return (xDiff * yDiff * zDiff <= VOLUME_FACTOR * size[0] * size[1] * size[2] || VOLUME_FACTOR * xDiff * yDiff * zDiff
      <= size[0] * size[1] * size[2]);
}
//--Asil

void
reindexClusters (const std::vector<sensor_msgs::PointCloud>& clusters)
{
  uint newClusterCount = clusters.size ();
  ClusterInfo* curInfo;

  double xCent, yCent, zCent;
  uint curNumberOfPoints, i, j;

  bool matchFound;
  int matchIndex = 0;

  //  cout << "New cluster count is : " << newClusterCount << endl;

  if (newClusterCount <= 0)
  {
    //    cout << "This number of clusters is not acceptable" << endl;
    return;
  }

  if (firstTime)
  {
    // Just populate the clusters.

    //cout << "First time population" << endl;

    for (i = 0; i < newClusterCount; i++)
    {

      curInfo = new ClusterInfo ();

      double min_x = 10.0, min_y = 10.0, min_z = 10.0;
      double max_x = -10.0, max_y = -10.0, max_z = -10.0;

      for (uint j = 0; j < clusters[i].points.size (); j++)
      {
        if (isnan (clusters[i].points[j].x) || isnan (clusters[i].points[j].y) || isnan (clusters[i].points[j].z))
          continue;
        if (clusters[i].points[j].x < min_x)
          min_x = clusters[i].points[j].x;

        if (clusters[i].points[j].x > max_x)
          max_x = clusters[i].points[j].x;

        if (clusters[i].points[j].y < min_y)
          min_y = clusters[i].points[j].y;
        if (clusters[i].points[j].y > max_y)
          max_y = clusters[i].points[j].y;

        if (clusters[i].points[j].z < min_z)
          min_z = clusters[i].points[j].z;
        if (clusters[i].points[j].z > max_z)
          max_z = clusters[i].points[j].z;
      }

      curInfo->bb_size[0] = max_x - min_x;
      curInfo->bb_size[1] = max_y - min_y;
      curInfo->bb_size[2] = max_z - min_z;

      curInfo->mins[0] = min_x;
      curInfo->mins[1] = min_y;
      curInfo->mins[2] = min_z;

      curInfo->maxs[0] = max_x;
      curInfo->maxs[1] = max_y;
      curInfo->maxs[2] = max_z;

      curInfo->numberOfPoints = clusters[i].points.size ();
      curInfo->index = i;
      curInfo->oldIndex = i;
      curInfo->matched = true;

      //cout << "Do you have a problem with centroid?" << endl;
      getCentroid (clusters[i], curInfo->numberOfPoints, curInfo->centroid[0], curInfo->centroid[1],
                   curInfo->centroid[2]);

      //cout << "I thought so" << endl;
      clusterIdentifications.push_back (*curInfo);

    }

    firstTime = false;
  }
  else
  {

    for (i = 0; i < clusterCount; i++)
    {
      clusterIdentifications[i].matched = false;
    }

    /*
     Perform the matching. Greedy, iterative approach is the way to go.

     for i
     1. Find the close enough clusterInfo.
     2.1 if such info does not exist, create one,
     2.2 else, <match> them (set the matched field of the corresponding info object to true)
     3. Remove the unmatched clusters. (check the matched field.)
     4. Set up the list of clouds to be published.
     */

    //cout << "Look for matches" << endl;
    for (i = 0; i < newClusterCount; i++)
    {

      matchFound = false;
      // Get the number of points for this cluster
      curNumberOfPoints = clusters[i].points.size ();

      //cout << "Getting the centroid of new cluster" << endl;

      // Get the centroid for this cluster

      getCentroid (clusters[i], curNumberOfPoints, xCent, yCent, zCent);

      //cout << "OK with this...." << endl;

      double min_x = 10.0, min_y = 10.0, min_z = 10.0;
      double max_x = -10.0, max_y = -10.0, max_z = -10.0;

      for (uint j = 0; j < clusters[i].points.size (); j++)
      {
        if (isnan (clusters[i].points[j].x) || isnan (clusters[i].points[j].y) || isnan (clusters[i].points[j].z))
          continue;
        if (clusters[i].points[j].x < min_x)
          min_x = clusters[i].points[j].x;

        if (clusters[i].points[j].x > max_x)
          max_x = clusters[i].points[j].x;

        if (clusters[i].points[j].y < min_y)
          min_y = clusters[i].points[j].y;
        if (clusters[i].points[j].y > max_y)
          max_y = clusters[i].points[j].y;

        if (clusters[i].points[j].z < min_z)
          min_z = clusters[i].points[j].z;
        if (clusters[i].points[j].z > max_z)
          max_z = clusters[i].points[j].z;
      }

      double box_x = max_x - min_x;
      double box_y = max_y - min_y;
      double box_z = max_z - min_z;

      // Scan the current list of ClusterInfo objects
      // reminder: currently, clusterCount is the length of the old list of clusters.

      for (j = 0; j < clusterCount && !matchFound; j++)
      {
        if (likelinessCheck (&clusterIdentifications[j], box_x, box_y, box_z))
        {
          //cout << "Match found. The obstacle has been stable since the last human interruption" << endl;
          matchFound = true;
          matchIndex = j; // the jth cluster identification matches with this cluster.
        }
      }
      // ++Asil: Modified by asil so as to be able to identify the moved objects
      if (matchFound)
      {
        //cout << "Setting the identification" << endl;

        clusterIdentifications[matchIndex].bb_size[0] = max_x - min_x;
        clusterIdentifications[matchIndex].bb_size[1] = max_y - min_y;
        clusterIdentifications[matchIndex].bb_size[2] = max_z - min_z;

        clusterIdentifications[matchIndex].mins[0] = min_x;
        clusterIdentifications[matchIndex].mins[1] = min_y;
        clusterIdentifications[matchIndex].mins[2] = min_z;

        clusterIdentifications[matchIndex].maxs[0] = max_x;
        clusterIdentifications[matchIndex].maxs[1] = max_y;
        clusterIdentifications[matchIndex].maxs[2] = max_z;

        clusterIdentifications[matchIndex].numberOfPoints = clusters[i].points.size ();

        //cout << "Do you have a problem with centroid?" << endl;
        getCentroid (clusters[i], clusterIdentifications[matchIndex].numberOfPoints,
                     clusterIdentifications[matchIndex].centroid[0], clusterIdentifications[matchIndex].centroid[1],
                     clusterIdentifications[matchIndex].centroid[2]);

        //cout << "I thought so" << endl;

        clusterIdentifications[matchIndex].oldIndex = clusterIdentifications[matchIndex].oldIndex;
        clusterIdentifications[matchIndex].index = i; // the cloud previously known as matchIndex_th cloud is now the cloud in clusters[i]
        clusterIdentifications[matchIndex].matched = true;
        //cout << "ID set" << endl;
      }
      else
      {
        //cout << "This cluster is new or moved by the experimenter. Identifying!!" << endl;

        for (j = 0; j < clusterCount && !matchFound; j++)
        {
          if (closeEnough (&clusterIdentifications[j], curNumberOfPoints, xCent, yCent, zCent))
          {
            //cout << "Match found. The obstacle has been stable since the last human interruption" << endl;
            matchFound = true;
            matchIndex = j; // the jth cluster identification matches with this cluster.
          }
        }

        if (matchFound)
        {
          //cout << "Match found. The obstacle has been moved by the experimenter" << endl;

          clusterIdentifications[matchIndex].bb_size[0] = max_x - min_x;
          clusterIdentifications[matchIndex].bb_size[1] = max_y - min_y;
          clusterIdentifications[matchIndex].bb_size[2] = max_z - min_z;

          clusterIdentifications[matchIndex].mins[0] = min_x;
          clusterIdentifications[matchIndex].mins[1] = min_y;
          clusterIdentifications[matchIndex].mins[2] = min_z;

          clusterIdentifications[matchIndex].maxs[0] = max_x;
          clusterIdentifications[matchIndex].maxs[1] = max_y;
          clusterIdentifications[matchIndex].maxs[2] = max_z;

          clusterIdentifications[matchIndex].numberOfPoints = clusters[i].points.size ();

          //cout << "Do you have a problem with centroid?" << endl;
          getCentroid (clusters[i], clusterIdentifications[matchIndex].numberOfPoints,
                       clusterIdentifications[matchIndex].centroid[0], clusterIdentifications[matchIndex].centroid[1],
                       clusterIdentifications[matchIndex].centroid[2]);

          //cout << "I thought so" << endl;

          clusterIdentifications[matchIndex].oldIndex = clusterIdentifications[matchIndex].index;
          clusterIdentifications[matchIndex].index = i; // the cloud previously known as matchIndex_th cloud is now the cloud in clusters[i]
          clusterIdentifications[matchIndex].matched = true;

        }
        else
        {
          curInfo = new ClusterInfo ();

          curInfo->bb_size[0] = max_x - min_x;
          curInfo->bb_size[1] = max_y - min_y;
          curInfo->bb_size[2] = max_z - min_z;

          curInfo->mins[0] = min_x;
          curInfo->mins[1] = min_y;
          curInfo->mins[2] = min_z;

          curInfo->maxs[0] = max_x;
          curInfo->maxs[1] = max_y;
          curInfo->maxs[2] = max_z;

          curInfo->centroid[0] = xCent;
          curInfo->centroid[1] = yCent;
          curInfo->centroid[2] = zCent;

          curInfo->numberOfPoints = curNumberOfPoints;
          curInfo->index = i;
          //cout << "Started assigning a new clusters " << endl;
          curInfo->oldIndex = assignClusterIndex (i); // TODO: check if this conflicts with some other guy.
          //cout << "Assigned " << curInfo->oldIndex << endl;
          curInfo->matched = true;

          clusterIdentifications.push_back (*curInfo);
          //cout << "Pushed my new cluster identification" << endl;
        }
      }
    }
    // --Asil: Modified by asil so as to be able to identify the moved objects


    //cout << "Match Done " << endl;
    // Prune the unmarked ClusterInfo objects

    for (j = 0; j < clusterIdentifications.size (); j++)
    {
      if (!clusterIdentifications[j].matched)
      {
        //cout << "The " << j << "th identification object is to be removed" << endl;
        clusterIdentifications.erase (clusterIdentifications.begin () + j);
        j--;
        //cout << "Removed!" << endl;
      }
    }
    //cout << "Prune done" << endl;
  }

  // Update the clusterCount
  clusterCount = newClusterCount;
  //cout << "hedehodo" << endl;
}

void
getReindexedClusters (const std::vector<sensor_msgs::PointCloud>& clusters, IndexedClusters& rc)
{
  reindexClusters (clusters);
  rc.clusters.resize (clusters.size ());
  rc.cluster_indices.resize (clusters.size ());

  for (unsigned int i = 0; i < clusters.size (); i++)
  {
    rc.clusters[i] = (clusters[clusterIdentifications[i].index]);
    rc.cluster_indices[i] = (clusterIdentifications[i].oldIndex);
  }
}

//void
//publishAllClusters (const std::vector<sensor_msgs::PointCloud>& clusters)
//{
//  for (uint i = 0; i < clusters.size (); i++)
//    if (pub_all_clusters.getNumSubscribers () > 0)
//      pub_all_clusters.publish (clusters[i]);
//}

void
publishBoundingBoxes (const IndexedClusters& indexed_clusters, visualization_msgs::MarkerArray& bbs,
                      visualization_msgs::MarkerArray& bb_texts)
{
  //delete the boxes already being sent
  for (uint i = 0; i < bbs.markers.size (); i++)
    bbs.markers[i].action = visualization_msgs::Marker::DELETE;

  //delete the boxes already being sent
  for (uint i = 0; i < bb_texts.markers.size (); i++)
    bb_texts.markers[i].action = visualization_msgs::Marker::DELETE;

  //publish for deleting them
  marker_array_pub.publish (bbs);
  pub_bb_texts.publish (bb_texts);

  bbs.markers.resize (indexed_clusters.clusters.size ());
  bb_texts.markers.resize (indexed_clusters.clusters.size ());

  for (uint i = 0; i < indexed_clusters.clusters.size (); i++)
  {
    double min_x = 10.0, min_y = 10.0, min_z = 10.0;
    double max_x = -10.0, max_y = -10.0, max_z = -10.0;

    for (uint j = 0; j < indexed_clusters.clusters[i].points.size (); j++)
    {
      if (isnan (indexed_clusters.clusters[i].points[j].x) || isnan (indexed_clusters.clusters[i].points[j].y)
          || isnan (indexed_clusters.clusters[i].points[j].z))
        continue;

      if (indexed_clusters.clusters[i].points[j].x < min_x)
        min_x = indexed_clusters.clusters[i].points[j].x;

      if (indexed_clusters.clusters[i].points[j].x > max_x)
        max_x = indexed_clusters.clusters[i].points[j].x;

      if (indexed_clusters.clusters[i].points[j].y < min_y)
        min_y = indexed_clusters.clusters[i].points[j].y;
      if (indexed_clusters.clusters[i].points[j].y > max_y)
        max_y = indexed_clusters.clusters[i].points[j].y;

      if (indexed_clusters.clusters[i].points[j].z < min_z)
        min_z = indexed_clusters.clusters[i].points[j].z;
      if (indexed_clusters.clusters[i].points[j].z > max_z)
        max_z = indexed_clusters.clusters[i].points[j].z;
    }
    //    std::cout << min_x << " " << min_y << " " << min_z << " " << max_x << " " << max_y << " " << max_z << std::endl;
    visualization_msgs::Marker marker;

    //create bounding box first
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now ();
    marker.ns = "bounding_boxes";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (min_x + max_x) / 2;
    marker.pose.position.y = (min_y + max_y) / 2;
    marker.pose.position.z = (min_z + max_z) / 2;
    marker.scale.x = max_x - min_x;
    marker.scale.y = max_y - min_y;
    marker.scale.z = max_z - min_z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    if (i == 0)
    {
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5;
    }
    else if (i == 1)
    {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5;
    }
    else if (i == 2)
    {
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.5;
    }
    else
    {
      marker.color.r = ((float)(indexed_clusters.clusters.size () - (i + 1))) / indexed_clusters.clusters.size ();
      marker.color.g = ((float)(i + 1)) / indexed_clusters.clusters.size ();
      marker.color.b = 0.5;
      marker.color.a = 0.5;
    }
    marker.lifetime = ros::Duration ();
    bbs.markers[i] = marker;

    //now modify this so that it becomes the text of the box
    marker.ns = "bounding_box_texts";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = i;
    std::stringstream s;
    s << indexed_clusters.cluster_indices[i] + 1;
    //    s << i;
    marker.text = s.str ();
    marker.pose.position.z += (marker.scale.z / 2.0 + 0.2);
    marker.scale.x = 0.5;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    bb_texts.markers[i] = marker;
  }

  if (marker_array_pub.getNumSubscribers () > 0)
    marker_array_pub.publish (bbs);

  if (pub_bb_texts.getNumSubscribers () > 0)
    pub_bb_texts.publish (bb_texts);
}

void
publishTabletopDetectionResults (const IndexedClusters &rc,
                                 const tabletop_object_detector::TabletopSegmentationResponse& response)//tabletop_object_detector::TabletopSegmentationResponse& response)
{
  tabletop_object_detector::TabletopDetectionResult msg;
  msg.clusters = rc.clusters;
  msg.result = response.result;
  msg.table = response.table;

  pub_tabletop_detection.publish (msg);
}

void
experimentStateCallback (aff_msgs::ExperimentStateConstPtr exp_state)
{
  exp_state_ = exp_state->experiment_state;
}

void
moduleStateCallback (aff_msgs::ModuleStatesConstPtr module_states)
{
  module_activation = module_states->workspace_detector;
}

void
publishTabletopDetectionResults (const tabletop_object_detector::TabletopSegmentationResponse& response)
{
  tabletop_object_detector::TabletopDetectionResult msg;
  msg.clusters = response.clusters;
  msg.result = response.result;
  msg.table = response.table;

  pub_tabletop_detection.publish (msg);
}

void
getSize (const pcl::PointCloud<pcl::PointXYZ> &cloud, double &xSize, double &ySize, double &zSize)
{

  double xMin, yMin, zMin, xMax, yMax, zMax;

  getMinMax (cloud, xMin, yMin, zMin, xMax, yMax, zMax);

  xSize = xMax - xMin;
  ySize = yMax - yMin;
  zSize = zMax - zMin;
}

void
pcRcvdCallback (sensor_msgs::PointCloud::ConstPtr pc_msg)
{
  if (!pointcloud_rcvd)
    pointcloud_rcvd = true;

  pc2_data_->header = pc_msg->header;
  pc2_data_->height = RANGE_IMAGE_HEIGHT;
  pc2_data_->width = RANGE_IMAGE_WIDTH;
  pc2_data_->is_dense = true; //there are no invalid points

  pc2_data_->points.resize (pc_msg->points.size ());
  for (uint i = 0; i < pc_msg->points.size (); i++)
  {
    //      pc2_data_->points[i].x = pc_msg->points[i].x;
    //TODO: why read points backwards?
    pc2_data_->points[i].x = pc_msg->points[pc_msg->points.size () - 1 - i].x;
    pc2_data_->points[i].y = pc_msg->points[pc_msg->points.size () - 1 - i].y;
    pc2_data_->points[i].z = pc_msg->points[pc_msg->points.size () - 1 - i].z;
  }

  if (filter_point_cloud)
  {
    outrem.setInputCloud (pc2_data_);
    outrem.filter (*pc2_data_);
  }

  pcl::toROSMsg (*pc2_data_, pc2_);

  //send pointcloud to tabletop detection service
  if (pub_pointcloud_raw.getNumSubscribers () > 0)
    pub_pointcloud_raw.publish (pc2_);

  //    pub_pc2.publish (pc2_);
}

void
pc2RcvdCallback (sensor_msgs::PointCloud2::ConstPtr pc2_msg)
{
  if (!pointcloud_rcvd)
    pointcloud_rcvd = true;

  ros::Time t_now = ros::Time::now ();

  if (filter_point_cloud)
  {
    tf_listener->lookupTransform ("/base_link", "/swissranger_link", ros::Time (0), tf_transform);
    pcl_ros::transformPointCloud ("/base_link", tf_transform, *pc2_msg, pc2_);

    pcl::fromROSMsg (pc2_, *pc2_data_);

    //    pcl::fromROSMsg (*pc2_msg, *pc2_data_);

    pass_through_x.setInputCloud (pc2_data_);
    pass_through_x.filter (*pc2_data_);
    pass_through_y.setInputCloud (pc2_data_);
    pass_through_y.filter (*pc2_data_);
    pass_through_z.setInputCloud (pc2_data_);
    pass_through_z.filter (*pc2_data_);

    outrem.setInputCloud (pc2_data_);
    outrem.filter (*pc2_data_);
    pcl::toROSMsg (*pc2_data_, pc2_);
  }
  else
    pc2_ = *pc2_msg;

  //std::cout << ros::Duration (ros::Time::now () - t_now).toSec () << std::endl;
  //send pointcloud to tabletop detection service
  if (pub_pointcloud_raw.getNumSubscribers () > 0)
    pub_pointcloud_raw.publish (pc2_);
}

void
filterClusters (std::vector<sensor_msgs::PointCloud>& clusters)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  for (uint i = 0; i < clusters.size (); i++)
  {
    cloud_ptr->width = clusters[i].points.size ();
    cloud_ptr->height = 1;
    cloud_ptr->points.resize (cloud_ptr->width * cloud_ptr->height);

    for (uint j = 0; j < cloud_ptr->points.size (); j++)
    {
      cloud_ptr->points[j].x = clusters[i].points[j].x;
      cloud_ptr->points[j].y = clusters[i].points[j].y;
      cloud_ptr->points[j].z = clusters[i].points[j].z;
    }
    cluster_outrem.setInputCloud (cloud_ptr);
    cluster_outrem.filter (*cloud_ptr);

    clusters[i].points.resize (cloud_ptr->points.size ());
    for (uint j = 0; j < cloud_ptr->points.size (); j++)
    {
      clusters[i].points[j].x = cloud_ptr->points[j].x;
      clusters[i].points[j].y = cloud_ptr->points[j].y;
      clusters[i].points[j].z = cloud_ptr->points[j].z;
    }
  }

  for (uint i = 0; i < clusters.size (); i++)
  {
    if (clusters[i].points.size () < 50)
    {
      clusters.erase (clusters.begin () + i);
      --i;
    }
  }
}

void
filterOutNonTableClusters (std::vector<sensor_msgs::PointCloud>& clusters, const tabletop_object_detector::Table& table)
{
  for (uint i = 0; i < clusters.size (); i++)
  {

    double min_x = 10.0, min_y = 10.0, min_z = 10.0;
    double max_x = -10.0, max_y = -10.0, max_z = -10.0;

    for (uint j = 0; j < clusters[i].points.size (); j++)
    {
      if (isnan (clusters[i].points[j].x) || isnan (clusters[i].points[j].y) || isnan (clusters[i].points[j].z))
        continue;

      if (clusters[i].points[j].x < min_x)
        min_x = clusters[i].points[j].x;

      if (clusters[i].points[j].x > max_x)
        max_x = clusters[i].points[j].x;

      if (clusters[i].points[j].y < min_y)
        min_y = clusters[i].points[j].y;
      if (clusters[i].points[j].y > max_y)
        max_y = clusters[i].points[j].y;

      if (clusters[i].points[j].z < min_z)
        min_z = clusters[i].points[j].z;
      if (clusters[i].points[j].z > max_z)
        max_z = clusters[i].points[j].z;
    }

    // if box is outside the table and above the table by 0.03 cm, then remove that cluster
    if (min_x < table.x_min || max_x > table.x_max || min_y < table.y_min || max_y > table.y_max || min_z
        > table.pose.pose.position.z + 0.08)
    {
      clusters.erase (clusters.begin () + i);
      i--;
    }
  }
}

int
percept ()
{
  if (module_activation)//module is activated
  {
    if (pointcloud_rcvd)//
    {
      bool call_segmentation_srv = true;
      bool seg_srv_call = false;

      //make a persistant service call as far as it is applicable
      while (nh->ok () && call_segmentation_srv)
      {
        //call the tabletop detection service
        //ROS_INFO("Calling tabletop segmentator");
        seg_srv_call = object_segmentation_srv.call (segmentation_call);

        if (!seg_srv_call)//service couldn't be called, quit
        {
          ROS_ERROR("Tabletop detection service failed");
          return -1;
        }
        else//service successfully called
        {
          if (segmentation_call.response.result == segmentation_call.response.NO_CLOUD_RECEIVED)
          {
            ROS_WARN("no point cloud received by segmentation service. Service is being recalled now...");
          }
          else if (segmentation_call.response.result == segmentation_call.response.NO_TABLE)
          {
            ROS_WARN("Something wrong with the table (no-table). Service is being recalled now...");
          }
          else if (segmentation_call.response.result == segmentation_call.response.OTHER_ERROR)//re-send point cloud
          {
            ROS_WARN("Possible transformation (tf) problem. Service is being recalled now...");
          }
          else if (segmentation_call.response.result == segmentation_call.response.SUCCESS)
          {//segmentation service called successfully go on with the feature extraction services
            //ROS_INFO("Successful segmentation! Moving to feature extraction...");
            call_segmentation_srv = false;
          }
        }
        ros::spinOnce ();
      }

      //filter clusters
      filterClusters (segmentation_call.response.clusters);
      filterOutNonTableClusters (segmentation_call.response.clusters, segmentation_call.response.table);

      //std::cout << "reindexing" << std::endl;
      getReindexedClusters (segmentation_call.response.clusters, indexed_clusters);

      //std::cout << "publishing bb" << std::endl;
      publishBoundingBoxes (indexed_clusters, bounding_boxes, bb_texts);

      //std::cout << "publishing one by one" << std::endl;
      publishClustersOneByOne (indexed_clusters);

      //std::cout << "publishing detection results" << std::endl;
      publishTabletopDetectionResults (indexed_clusters, segmentation_call.response);

      //call feature extraction service
      //TODO: Asil -> feature extraction module interface
      //        extractAndVisFeatures(segmentation_call.response.clusters);

      featurize->extractAndVisualizeAll (indexed_clusters.clusters);

    }
  }
  else//module is deactivated
  {
    std::cout << "Module seems to be inactive!" << std::endl;
    //Do things that are to be done when module is deactivated
  }
  return 1;
}

bool
perceptionCallback (feature_manager::Perception::Request& request, feature_manager::Perception::Response& response)
{
  if (request.task == feature_manager::Perception::Request::DO_PERCEPT)
  {
    is_service_called = true;
    percept ();
    std::cout << "Went to do percept" << std::endl;
    response.pushable_object_center.resize (3);
    response.pushable_object_size.resize (3);
    if (request.arg >= 0)
    {
      response.pushable_object_center[0] = bounding_boxes.markers[request.arg].pose.position.x;
      response.pushable_object_center[1] = bounding_boxes.markers[request.arg].pose.position.y;
      response.pushable_object_center[2] = bounding_boxes.markers[request.arg].pose.position.z;

      response.pushable_object_size[0] = bounding_boxes.markers[request.arg].scale.x;
      response.pushable_object_size[1] = bounding_boxes.markers[request.arg].scale.y;
      response.pushable_object_size[2] = bounding_boxes.markers[request.arg].scale.z;

      std::cout << "*****************************************************************" << std::endl;
      std::cout << response.pushable_object_center[0] << " " << response.pushable_object_center[1] << " "
          << response.pushable_object_center[2] << std::endl;

      std::cout << response.pushable_object_size[0] << " " << response.pushable_object_size[1] << " "
          << response.pushable_object_size[2] << std::endl;
      std::cout << "*****************************************************************" << std::endl;
      //      response.pushable_object_center[0] = object_center[0];
      //      response.pushable_object_center[1] = object_center[1];
      //      response.pushable_object_center[2] = object_center[2];
      //
      //      response.pushable_object_size[0] = object_dims[0];
      //      response.pushable_object_size[1] = object_dims[1];
      //      response.pushable_object_size[2] = object_dims[2];
    }
    else
    {
      //if something goes nasty, do respond with a virtual object
      response.pushable_object_center[0] = object_center[0];
      response.pushable_object_center[1] = object_center[1];
      response.pushable_object_center[2] = object_center[2];

      response.pushable_object_size[0] = object_dims[0];
      response.pushable_object_size[1] = object_dims[1];
      response.pushable_object_size[2] = object_dims[2];
    }

    // ++Onur: The part where the logging happens.
    for (uint8_t i = 0; i < segmentation_call.response.clusters.size (); i++)
    {
      sensor_msgs::PointCloud2 pc2_cluster;
      sensor_msgs::convertPointCloudToPointCloud2 (segmentation_call.response.clusters[i], pc2_cluster);
      featurize->extractAndGetFeatures (pc2_cluster);
             
      featurize->setSizeAndPos (bounding_boxes.markers[i].pose.position.x, bounding_boxes.markers[i].pose.position.y,
                                bounding_boxes.markers[i].pose.position.z, bounding_boxes.markers[i].scale.x,
                                bounding_boxes.markers[i].scale.y, bounding_boxes.markers[i].scale.z);

      double* dummy = featurize->getCurrentFeatureVector ();

	dummy[featureCount -1] = 1.0; // the last, 47th feature
      for (uint8_t j = 0; j < featureCount; j++)
      {
        std::cout << dummy[j] << " ";
      }

     featurize->featureLogger->logSingleData (dummy, i, request.arg); // Hence the ith cluster is stored as the request.arg_th object

      /*if (featurize->isFinal ())
      {
        dummy = featurize->getEffectVector ();
        featurize->effectLogger->logSingleData (dummy, i, request.arg); // hence effect for the ith object is stored!
        featurize->setInitVector (); // just for future use, save the final features as the next initial features
        featurize->setFinal (false);
      }
      else*/
      //{
        featurize->setInitVector (); // store the current feature vector as the initial condition vector!
        featurize->setFinal (true);
      //}
      std::cout << std::endl;
    }
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    // --Onur
  }
  else if (request.task == feature_manager::Perception::Request::EXTRACT_EFFECT)
  {
  
  	std::cout<< "The effect label is received as " << (int)request.arg_effect << std::endl;
    // ++Onur: The implementation of effect feature storage with voice command as the label of the effect
    is_service_called = true;
    percept ();

    response.pushable_object_center.resize (3);
    if(request.arg>=0)
    {
      response.pushable_object_center[0] = bounding_boxes.markers[request.arg].pose.position.x;
      response.pushable_object_center[1] = bounding_boxes.markers[request.arg].pose.position.y;
      response.pushable_object_center[2] = bounding_boxes.markers[request.arg].pose.position.z;

      std::cout<<"object ****** : "<<std::endl;
      std::cout << response.pushable_object_center[0] << " " << response.pushable_object_center[1] << " "
          << response.pushable_object_center[2] << std::endl;
    }

    std::cout << "Went to extract effect" << std::endl;

    if (featurize->isFinal ())
    {

      if(segmentation_call.response.clusters.size() < 1)
      {
        std::cout<< "No objects found on the table top. So, will now send an empty vector." << std::endl;
        double *dummy = new double[featureCount];
        for(int i = 0; i < featureCount-1; i++)
        {
          dummy[i] = 0;
        }
        
        dummy[featureCount-1] = -1.0;
        featurize->featureLogger->logSingleData(dummy,1000);
        featurize->effectLogger->logSingleData(dummy,1000,1000);
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;

        featurize->setFinal (false);

      }
      for (uint8_t i = 0; i < segmentation_call.response.clusters.size (); i++)
      {
        sensor_msgs::PointCloud2 pc2_cluster;
        sensor_msgs::convertPointCloudToPointCloud2 (segmentation_call.response.clusters[i], pc2_cluster);
        featurize->extractAndGetFeatures (pc2_cluster);

        featurize->setSizeAndPos (bounding_boxes.markers[i].pose.position.x, bounding_boxes.markers[i].pose.position.y,
                                  bounding_boxes.markers[i].pose.position.z, bounding_boxes.markers[i].scale.x,
                                  bounding_boxes.markers[i].scale.y, bounding_boxes.markers[i].scale.z);

        double* dummy = featurize->getCurrentFeatureVector ();

        std::cout << "The final vector: " << endl;
        for (uint8_t j = 0; j < featureCount; j++)
        {
          std::cout << dummy[j] << " ";
        }
	
        featurize->featureLogger->logSingleData (dummy, i); // store the final vector just in case!
        dummy = featurize->getEffectVector ();

	dummy[featureCount -1 ] = 0.0; // It should be guaranteed that the object presence did not change!
        std::cout << "The corresponding effect vector: " << endl;
        for (uint8_t j = 0; j < featureCount; j++)
        {
          std::cout << dummy[j] << " ";
        }
//
        featurize->effectLogger->logSingleData (dummy, i, request.arg_effect); // hence effect for the ith object with label request.arg!
        featurize->setInitVector (); // just for future use, save the final features as the next initial features
        featurize->setFinal (false);
////
//        std::cout << std::endl;
      }
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;

    }
    else
    {
      std::cout
          << "It seems the initial vector was not set at this instance. Would you please check your implementation & execution ?"
          << std::endl;
    }

    // --Onur

  }
  else
  {
//    std::cout << "Did not go to any of the callback options!" << std::endl;
  }

  //  else if (request.task == feature_manager::Perception::Request::EXTRACT_EFFECT)
  //  {
  //
  //  }

  response.feedback = feature_manager::Perception::Response::DONE;
  //  for (uint8_t i = 0; i < segmentation_call.response.clusters.size (); i++)
  //  {
  //    //    double x = 0, y = 0, z = 0;
  //    //    getCentroid (segmentation_call.response.clusters[i], segmentation_call.response.clusters[i].points.size (), x, y, z);
  //    //    double x_size = 0, y_size = 0, z_size = 0;
  //    //    getSize (segmentation_call.response.clusters[i], x_size,
  //    //             y_size, z_size);
  //    double x_min_limit = -0.40, x_max_limit = -0.20, y_min_limit = -0.30, y_max_limit = -0.10, z_min_limit = -0.10,
  //           z_max_limit = 0;
  //
  //    //    if( x_min_limit < bounding_boxes.markers[i].pose.position.x && bounding_boxes.markers[i].pose.position.x < x_max_limit &&
  //    //        y_min_limit < bounding_boxes.markers[i].pose.position.y-bounding_boxes.markers[i].scale.y/2 &&
  //    //        bounding_boxes.markers[i].pose.position.y-bounding_boxes.markers[i].scale.y/2 < y_max_limit  )
  //    //    {
  //    response.pushable_object_center.resize (3);
  //    response.pushable_object_center[0] = bounding_boxes.markers[i].pose.position.x;
  //    response.pushable_object_center[1] = bounding_boxes.markers[i].pose.position.y;
  //    response.pushable_object_center[2] = bounding_boxes.markers[i].pose.position.z;
  //
  //    response.pushable_object_size.resize (3);
  //    response.pushable_object_size[0] = bounding_boxes.markers[i].scale.x;
  //    response.pushable_object_size[1] = bounding_boxes.markers[i].scale.y;
  //    response.pushable_object_size[2] = bounding_boxes.markers[i].scale.z;
  //    break;
  //    //    }
  //  }

  //  response.pushable_object

//  std::cout << "Now we are out of callback" << std::endl;
  is_service_called = false;
  return true;
}

void
init ()
{
  nh = new ros::NodeHandle ();

  //nhfeat = new ros::NodeHandle("-");
  nh->getParam ("/use_sim_time", sim);
  if (sim)
    sub_pointcloud = nh->subscribe ("/swissranger/scan", 10, &pcRcvdCallback);
  else
    sub_pointcloud = nh->subscribe ("/swissranger/pointcloud2_raw", 10, &pc2RcvdCallback);

  pub_pointcloud_raw = nh->advertise<sensor_msgs::PointCloud2> ("/pointcloud_raw", 10);

  object_segmentation_srv
      = nh->serviceClient<tabletop_object_detector::TabletopSegmentation> (WORKSPACE_SEGMENTATION_SRV_NAME, true);

  marker_array_pub = nh->advertise<visualization_msgs::MarkerArray> ("bounding_boxes", 1);
  pub_bb_texts = nh->advertise<visualization_msgs::MarkerArray> ("bounding_box_texts", 1);

  //  pub_all_clusters = nh->advertise<sensor_msgs::PointCloud2> ("tabletop_clusters", 10);
  pub_tabletop_detection
      = nh->advertise<tabletop_object_detector::TabletopDetectionResult> ("/tabletop_detection_result", 10);

  pub_tabletop_3d_features_ = nh->advertise<aff_msgs::Features>("/tabletop_3D_features", 10);

  srv_perception = nh->advertiseService ("/perception", perceptionCallback);
  tf_listener = new tf::TransformListener ();

  //wait for first transform to be available
  bool got_tf = false;
  while (!got_tf && nh->ok ())
  {
    try
    {
      tf_listener->waitForTransform ("/base_link", "/swissranger_link", ros::Time (0), ros::Duration (0.1));
      tf_listener->lookupTransform ("/base_link", "/swissranger_link", ros::Time (0), tf_transform);
      got_tf = true;
    }
    catch (tf::LookupException& e)
    {
      got_tf = false;
    }
  }

  // build the filter
  outrem.setRadiusSearch (0.03);
  outrem.setMinNeighborsInRadius (45);
  cluster_outrem.setRadiusSearch (0.03);
  cluster_outrem.setMinNeighborsInRadius (25);

  pass_through_x.setFilterFieldName ("x");
  pass_through_x.setFilterLimits (-0.90, -0.1);
  pass_through_y.setFilterFieldName ("y");
  pass_through_y.setFilterLimits (-0.8, 0.6);
  pass_through_z.setFilterFieldName ("z");
  pass_through_z.setFilterLimits (-0.2, 0.5);

  featurize = new featurizer::Featurizer (nh);

  pc2_data_.reset (new pcl::PointCloud<pcl::PointXYZ>);

  //spinner for feature extraction and workspace detection services, and also pointcloud callbacks

  spinner_ = new ros::AsyncSpinner (4);
  spinner_->start ();
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "perception_manager");
  init ();
  //wait for detection client
  while (!ros::service::waitForService (WORKSPACE_SEGMENTATION_SRV_NAME, ros::Duration (2.0)) && nh->ok ())
  {
    ROS_INFO("Waiting for object detection service to come up");
  }
  ROS_INFO("object detection service is ready");

  ros::Rate r (50);
  while (nh->ok ())
  {
    if (!is_service_called)
      percept ();
    ros::spinOnce ();
    r.sleep ();
  }

  return 0;
}
