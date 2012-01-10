#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/extract_indices.h"

int
  main (int argc, char** argv){
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  	  cloud (new pcl::PointCloud<pcl::PointXYZ> ()),
  	  cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ()),
  	  cloud_removed (new pcl::PointCloud<pcl::PointXYZ> ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true);
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  pcl::IndicesConstPtr indices = sor.getRemovedIndices();
  std::cerr<<"n_indices removed: "<<indices->size()<<std::endl;

  pcl::PointIndices::Ptr removed_indices (new pcl::PointIndices());
  removed_indices->indices = *indices;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (removed_indices);
  extract.setNegative (false);
  extract.filter (*cloud_removed);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

  writer.write<pcl::PointXYZ> ("table_scene_lms400_removed.pcd", *cloud_removed, false);

  return (0);
}
/* ]--- */
