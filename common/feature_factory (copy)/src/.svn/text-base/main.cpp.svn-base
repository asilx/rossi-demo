/*  Copyright (C) 2011, Kadir Firat Uyanik
 *    Middle East Technical University, Kovan Research Lab
 *    kadir@ceng.metu.edu.tr
 *
 *    http://kovan.ceng.metu.edu.tr/~kadir
 *
 *  main.cpp is part of feature_factory.
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

// Do not ever run this if the real and the sim robot runs at the same time
// since point cloud subscriptions are done based on the assumption that
// sim and real environments are mutually exclusive.
#include "feature_factory/FeatureFactory.h"

int
main (int argc, char* argv[])
{

  ros::init (argc, argv, "feature_factory");
  ros::NodeHandle nh;

  std::cout << "hello world!" << std::endl;

//  cv::Mat data(cvSize(10,10),CV_32F);
//  cv::randn(data,cvScalar(5,5),cvScalar(2,2));
//  cv::randu(data, cv::Scalar(0), cv::Scalar(100));
//  std::cout<<data<<std::endl<<std::endl;
//
//  cv::Mat mask = cv::Mat::zeros(data.rows, data.cols,CV_32F);
//  mask.at<float>(0.0) = 1;
//
//  cv::Rect roi(0,0,5 ,5);
//
//  cv::Mat grid_data = data(roi);
//  std::cout<<grid_data<<std::endl;

  feature_factory::FeatureFactory ff (nh);
  ff.run ();

  //	pcl::io::savePCDFile("test.pcd", cloud);
  return 0;
}
