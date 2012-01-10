/*
 * LearningManager.h
 *
 *  Created on: Nov 1, 2011
 *      Author: kadir
 */

#ifndef LEARNINGMANAGER_H_
#define LEARNINGMANAGER_H_

#include "libsvm/svm.h"

#include "ros/ros.h"
#include "aff_msgs/Features.h"

struct Predictor
{
  std::vector<double> features_;// e.g. entity_features
  std::vector<int> relevant_feature_indices_;
  std::vector<double> relevant_features_;// e.g. entity_features
  int auxiliary_class_; // e.g. action_id
  int predicted_class_; // e.g. effect_id

  int
  predictClass (const std::vector<double>& features, int auxiliary_class);
};

class LearningManager
{
public:
  LearningManager (ros::NodeHandle* nh);

  ros::NodeHandle* nh_;
  ros::Subscriber sub_tabletop_2d_features_;
  ros::Subscriber sub_tabletop_23_features_;
  ros::Subscriber sub_internal_features_;
  ros::Subscriber sub_experiment_labels_;

  Predictor effect_predictor_;
  int16_t episode_index_;
  std::vector<float> curr_feature_vector_;
  std::vector<float> prev_feature_vector_;

  bool episode_finished_;

  virtual
  ~LearningManager ();

  void run();

private:

//  void
//  catFeatures (const std::vector<double> &proprioceptive, const std::vector<double> &perceptive);

  std::vector<double>
  getRelevantFeatures (const std::vector<double> &features, std::vector<int> &relevancy_indices);

  void
  featureRcvdCallback (aff_msgs::FeaturesConstPtr features);

//  void
//  perceptor3DFeaturesCallback (aff_msgs::FeaturesConstPtr features);
//
//  void
//  perceptor2DFeaturesCallback (aff_msgs::FeaturesConstPtr features);
//
//  void
//  proprioceptiveFeaturesCallback (aff_msgs::FeaturesConstPtr features);
};

#endif /* LEARNINGMANAGER_H_ */
