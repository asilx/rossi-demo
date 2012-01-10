/*
 * LearningManager.cpp
 *
 *  Created on: Nov 1, 2011
 *      Author: kadir
 */

#include "learning_manager/LearningManager.h"

LearningManager::LearningManager (ros::NodeHandle* nh)
{
  nh_ = nh;
  episode_index_ = 0; //this will be used to understand if experiment state changed, and concatenate feature vectors correspondingly
  //if any feature is obtained with larger episode_id, current features are cat'ed and new feature vector is formed.

//  sub_perceptor3D_features_ = nh->subscribe("/perceptor3D_features", 10, &LearningManager::perceptor3DFeaturesCallback, this);
//  sub_perceptor3D_features_ = nh->subscribe("/perceptor2D_features", 10, &LearningManager::perceptor2DFeaturesCallback, this);
//  sub_perceptor3D_features_ = nh->subscribe("/proprioceptive_features", 10, &LearningManager::proprioceptiveFeaturesCallback, this);


  sub_tabletop_2d_features_ = nh_->subscribe("/tabletop_2D_features", 10, &LearningManager::featureRcvdCallback, this);
  sub_tabletop_23_features_ = nh_->subscribe("/tabletop_3D_features", 10, &LearningManager::featureRcvdCallback, this);
  sub_internal_features_ = nh_->subscribe("/internal_features", 10, &LearningManager::featureRcvdCallback, this);
  sub_experiment_labels_ = nh_->subscribe("/experiment_labels", 10, &LearningManager::featureRcvdCallback, this);
}

LearningManager::~LearningManager ()
{
  // TODO Auto-generated destructor stub
}

void
LearningManager::featureRcvdCallback (aff_msgs::FeaturesConstPtr features)
{
  if(features->episode_index != episode_index_)
  {
    prev_feature_vector_ = curr_feature_vector_;
    curr_feature_vector_ = features->features;
    episode_index_ = features->episode_index;
    episode_finished_ = true;
  }
  else
    curr_feature_vector_.insert(curr_feature_vector_.end(), features->features.begin(), features->features.end());
}

//void
//LearningManager::perceptor3DFeaturesCallback (aff_msgs::FeaturesConstPtr features)
//{
//  if(features->episode_index != episode_index_)
//  {
//    prev_feature_vector_ = curr_feature_vector_;
//    curr_feature_vector_ = features->features;
//    episode_index_ = features->episode_index;
//    episode_finished_ = true;
//  }
//  else
//    curr_feature_vector_.insert(curr_feature_vector_.end(), features->features.begin(), features->features.end());
//}

//void
//LearningManager::perceptor2DFeaturesCallback (aff_msgs::FeaturesConstPtr features)
//{
//
//}
//
//void
//LearningManager::proprioceptiveFeaturesCallback (aff_msgs::FeaturesConstPtr features)
//{
//
//}

//std::vector<double>
//LearningManager::catFeatures (const std::vector<double> &proprioceptive, const std::vector<double> &perception3D, const std::vector<double> &perception2D)
//{
//  std::vector<double> features(proprioceptive.size() + perception3D.size() + perception2D.size());
//  features = perception3D;
//  features.insert(features.size()-1, perception2D.begin(),perception2D.end());
//  features.insert(features.size()-1, proprioceptive.begin(), proprioceptive.end());
//
//  return features;
//}

std::vector<double>
LearningManager::getRelevantFeatures (const std::vector<double> &features, std::vector<int> &relevancy_indices)
{
  std::vector<double> relevant_features(relevancy_indices.size());
  for(uint16_t i=0;i< relevancy_indices.size();i++)
  {
    if(relevancy_indices[i] < (int)features.size())
      relevant_features[i]= features[ relevancy_indices[i] ];
    else
    {
      ROS_ERROR("relevant feature index is not true, it is %dm but feature_vector is of size %d", relevancy_indices[i], (int)features.size());
      exit(-1);
    }
  }
  return relevant_features;
}

void LearningManager::run()
{
  while(nh_->ok())
  {
    if(episode_finished_)
    {
      episode_finished_ = false;
    }
    ros::spinOnce();
  }
}
