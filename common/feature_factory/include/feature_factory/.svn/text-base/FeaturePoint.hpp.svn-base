/*  Copyright (C) 2011, Kadir Firat Uyanik
 *    Middle East Technical University, Kovan Research Lab
 *    kadir@ceng.metu.edu.tr
 *
 *    http://kovan.ceng.metu.edu.tr/~kadir
 *
 *  FeaturePoint.h is part of feature_factory.
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

#ifndef FEATUREPOINT_H_
#define FEATUREPOINT_H_

#include "pcl/ros/register_point_struct.h"
#include "pcl/point_types.h"

#include "iostream"
#include "vector"

#include "opencv2/opencv.hpp"

//++Configuration Paramaters
#define SURFACE_FEATURES_HISTOGRAM 20
#define RANGE_FEATURES_HISTOGRAM 1
// TODO: change these names to more reasonable ones: FEATURE_VAL_16, FEATURE_VAL_8
#define MIN_DEPTH_FEATURE_VAL	 0
#define MAX_DEPTH_FEATURE_VAL	 65535
#define MIN_CONF_AMP_FEATURE_VAL 0
#define MAX_CONF_AMP_FEATURE_VAL 255

#define GRID_EDGE_SIZE 16

#define MAX_RANGE 5.0
//--Configuration Paramaters

namespace feature_factory
{
  template<int histogram_n_bins>
    struct FeaturePoint
    {
      PCL_ADD_POINT4D
      ;
      float min;
      float max;
      float mean;
      float std_dev;
      float histogram[histogram_n_bins];

      FeaturePoint ()
      {
        min = 0;
        max = 0;
        mean = 0;
        std_dev = 0;
        for (int i = 0; i < histogram_n_bins; i++)
          histogram[i] = 0;
      }

      FeaturePoint (float const& min_, float const& max_, float const& mean_, float const& std_dev_,
                    float const histogram_[histogram_n_bins])
      {
        min = min_;
        max = max_;
        mean = mean_;
        std_dev = std_dev_;
        for (int i = 0; i < histogram_n_bins; i++)
          histogram[i] = histogram_[i];
      }

      FeaturePoint (const FeaturePoint & tmp)
      {
        *this = tmp;
      }

      FeaturePoint&
      operator = (const FeaturePoint &tmp)
      {
        min = tmp.min;
        max = tmp.max;
        mean = tmp.mean;
        std_dev = tmp.std_dev;
        for (int i = 0; i < histogram_n_bins; i++)
          histogram[i] = tmp.histogram[i];
        return *this;
      }

      static FeaturePoint
      calculate (const cv::Mat& data, const cv::Rect& roi = cv::Rect (0, 0, GRID_EDGE_SIZE, GRID_EDGE_SIZE),
                 float min_possible_val = MIN_DEPTH_FEATURE_VAL, float max_possible_val = MAX_DEPTH_FEATURE_VAL)
      {
        //normalize data first
        cv::Mat grid_data = data (roi);

        double min = 0;
        double max = 0;
        cv::minMaxLoc (grid_data, &min, &max, 0, 0);

        cv::Scalar mean = 0;
        cv::Scalar std_dev = 0;
        cv::meanStdDev (grid_data, mean, std_dev);

        // Although no need to use float type here, it is recommended to use same types of data
        // inside a custom point cloud, a part of which this histogram will be converted to later.
        cv::MatND hist = cv::MatND (1, histogram_n_bins, CV_32F);
        float range[] = {min_possible_val, max_possible_val};
        const float* ranges[] = {range};
        const int channel = 0;
        const int n_bins = histogram_n_bins;
        cv::calcHist(&grid_data, 1, &channel, cv::Mat(), hist, 1, &n_bins, ranges, true, false);

        //now normalize min, max, mean values. Histogram stays the same, since it holds the frequency information.
        float possible_val_range = max_possible_val - min_possible_val;
        min = (min - min_possible_val) / possible_val_range;
        max = (max - min_possible_val) / possible_val_range;
        mean.val[0] = (mean.val[0] - min_possible_val) / possible_val_range;
        std_dev.val[0] = (std_dev.val[0] - min_possible_val) / possible_val_range;

        return FeaturePoint (min, max, (float)mean.val[0], (float)std_dev.val[0], (float*)hist.data);
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
    }EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
    feature_factory::FeaturePoint<SURFACE_FEATURES_HISTOGRAM>,
    (float, min, min) (float, max, max) (float, mean, mean) (float, std_dev, std_dev) (float[SURFACE_FEATURES_HISTOGRAM],histogram, histogram))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    feature_factory::FeaturePoint<RANGE_FEATURES_HISTOGRAM>,
    (float, min, min) (float, max, max) (float, mean, mean) (float, std_dev, std_dev) (float[RANGE_FEATURES_HISTOGRAM],histogram, histogram));

#endif /* FEATUREPOINT_H_ */
