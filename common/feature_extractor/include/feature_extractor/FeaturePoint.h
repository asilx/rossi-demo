/*
 * FeaturePoint.h
 *
 *  Created on: Feb 13, 2011
 *      Author: Kadir Firat Uyanik
 *      		www.kadiruyanik.wordpress.com
 */

#ifndef FEATUREPOINT_H_
#define FEATUREPOINT_H_
#include "pcl/ros/register_point_struct.h"
#include "iostream"
#include "iomanip"
#include "vector"

//++Configuration Paramaters
#define RANGE_FEATURES_HISTOGRAM 	1
#define SURFACE_FEATURES_HISTOGRAM 20
#define MIN_FEATURE_VALUE			0
#define MAX_FEATURE_VALUE		65536
//--Configuration Paramaters

using namespace std;

namespace feature_extractor
{

  template<int histogram_n_bins>
    struct Feature
    {
      float min;
      float max;
      float mean;
      float var;
      float histogram[histogram_n_bins];

      Feature ()
      {
        min = 0;
        max = 0;
        mean = 0;
        var = 0;
        for (int i = 0; i < histogram_n_bins; i++)
          histogram[i] = 0;
      }

      Feature (float const& min_, float const& max_, float const& mean_, float const& var_,
               float const histogram_[histogram_n_bins])
      {
        min = min_;
        max = max_;
        mean = mean_;
        var = var_;
        for (int i = 0; i < histogram_n_bins; i++)
          histogram[i] = histogram_[i];
      }

      Feature (const Feature & tmp)
      {
        *this = tmp;
      }

      Feature&
      operator = (const Feature &tmp)
      {
        min = tmp.min;
        max = tmp.max;
        mean = tmp.mean;
        var = tmp.var;
        for (int i = 0; i < histogram_n_bins; i++)
          histogram[i] = tmp.histogram[i];
        return *this;
      }

      void
      print ()
      {
        std::cout << "min :" << setiosflags (ios::fixed) << setprecision (3) << setw (7) << min << "\t" << "max :"
            << setiosflags (ios::fixed) << setprecision (3) << setw (7) << max << "\t" << "mean:"
            << setiosflags (ios::fixed) << setprecision (3) << setw (7) << mean << "\t" << "var :"
            << setiosflags (ios::fixed) << setprecision (3) << setw (7) << var << std::endl;
        std::cout << "hist:";
        for (int i = 0; i < histogram_n_bins; i++)
          std::cout << setiosflags (ios::fixed) << setprecision (3) << setw (7) << histogram[i] << " ";
        std::cout << endl;
      }

      Feature
      calculate (std::vector<float> const& data, bool apply_itself, float min_possible_val = MIN_FEATURE_VALUE,
                 float max_possible_val = MAX_FEATURE_VALUE)
      {
        float tmp_min = max_possible_val;
        float tmp_max = min_possible_val;
        float tmp_mean = 0;
        float tmp_var = 0;

        float sum_dev_squared = 0;

        unsigned int n_data = data.size();
        for(unsigned int i=0;
            i < n_data;
            i++)
            {
              if(data[i] < tmp_min)
              tmp_min = data[i];
              else if(data[i]>tmp_max)
              tmp_max = data[i];
              tmp_mean += data[i]/n_data;
            }
            for(unsigned int i=0;
            i < n_data;
            i++)
            {
              sum_dev_squared+= (data[i]-tmp_mean)*(data[i]-tmp_mean);
            }
            tmp_var = sum_dev_squared / n_data;

            float tmp_histogram[histogram_n_bins];
            for(unsigned int i=0;
                i < histogram_n_bins;
                i++)
                tmp_histogram[i]=0;
                if (histogram_n_bins)
                {
                  float bin_step_size = (float)(max_possible_val - min_possible_val) / histogram_n_bins;
                  for(unsigned int i=0;
                      i < n_data;
++                    )
                    tmp_histogram[(int)floor((data[i]-min_possible_val)/bin_step_size)]++;
                  }

                  Feature<histogram_n_bins> tmp(tmp_min, tmp_max, tmp_mean, tmp_var , tmp_histogram);
                  if(apply_itself)
                  *this = tmp;
                  return tmp;
                }

                Feature normalize(float const& min_val, float const& max_val, bool apply_itself=true)
                {
                  Feature tmp;
                  tmp.min = (min-min_val) /(max_val-min_val);
                  tmp.max = (max-min_val) /(max_val-min_val);
                  tmp.mean= (mean-min_val)/(max_val-min_val);

                  tmp.var = var/mean;

                  int data_cnt = 0;
                  for(int i=0;i<histogram_n_bins;i++)
                  data_cnt += histogram[i];

                  for(int i=0;i<histogram_n_bins;i++)
                  tmp.histogram[i] = histogram[i]/data_cnt;

                  if(apply_itself)
                  {
                    *this = tmp;
                  }
                  return tmp;
                }
              };
            }

POINT_CLOUD_REGISTER_POINT_STRUCT(
    feature_extractor::Feature<RANGE_FEATURES_HISTOGRAM>,
    (float, min, min)
    (float, max, max)
    (float, mean, mean)
    (float, var, var)
    (float[RANGE_FEATURES_HISTOGRAM],histogram, histogram))
;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    feature_extractor::Feature<SURFACE_FEATURES_HISTOGRAM>,
    (float, min, min)
    (float, max, max)
    (float, mean, mean)
    (float, var, var)
    (float[SURFACE_FEATURES_HISTOGRAM],histogram, histogram))
;
#endif /* FEATUREPOINT_H_ */
