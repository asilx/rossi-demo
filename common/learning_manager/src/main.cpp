/*
 * main.cpp
 *
 *  Created on: Nov 1, 2011
 *      Author: kadir
 */

#include "iostream"
#include "string"
#include "vector"
#include "learning_manager/LearningManager.h"

#include "ros/ros.h"

enum LearningMode
{
  OFFLINE,
  ONLINE
};

typedef struct effect_predictor
{
  //action enumeration id (e.g. push_left for 15cm, grasp_and_lift for 20cm,
  //hide, knock_down (small_objects, high objects, wide objects etc), roll_left(push_left with
  //truefied spinning flag))
  int action_id_;

  std::vector<float> prototypes_;
  svm_model* learned_model_;
}EffectPredictor;

typedef struct learned_relation
{
  int8_t action_class;
  int8_t entity_class;
  int8_t effect_class;
  std::vector<float> action_features;
  std::vector<float> entity_features;
  std::vector<float> effect_features;
  svm_model* learned_model;

}LearnedRelation;

const int LEARNING_MODE = OFFLINE;
const int N_ACTIONS = 2;
const int N_EFFECTS = 2;
std::vector<svm_model> effect_predictor_models_;
std::vector<svm_model> action_predictor_models_;

void learn()
{

}

//look for the folder experiment_id inside the experiment_manager folder
void learnOffline(int8_t experiment_id)
{

}

//requires experiment service call (does by itself or wait for an
//experiment supervision (voice or command line instructions))
//it takes action, feature and label related information
//and updates the svm model
void learnOnline();

//it takes the action as input, and checks the svm model corresponding
//to this action, given that the entity feature vector is provided
//NOTE: hence we need an svm model which takes entity and actions as input
//and effect label as the ouput so that it can predict if a specific action
//is applied for a specific entity feature vector.
void predictEffect(const std::string& action_name );

//checks if any of the learned effect_predictor_models_ can classify
void predictEffect(const int8_t& action_id, const std::vector<float>& entity_features )
{

}

//it takes a goal effect as input, then asks the pre-learned svm models
//which action may generate a similar effect.
//NOTE: we need to apply various actions in an experiment and label the
//outcomes of each actions. Therefore an action can be predicted for a
//given effect in a statistical manner.
void predictAction();

//it initializes experiment manager so that a dataset is to be collected.
void doExperiment();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "learning_manager");
  ros::NodeHandle nh;
  LearningManager learning_manager(&nh);
  learning_manager.run();

  std::cout<<"hello world!"<<std::endl;
  return 0;
}
