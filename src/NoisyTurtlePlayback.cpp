/**
 * @file NoisyTurtlePlayback.cpp
 * @author Sandip Das (sd13535@outlook.com)
 * @brief Implements member functions of the Noise adder node class
 * @version 0.1
 * @date 2024-05-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "noisy_turtle_playback/NoisyTurtlePlayback.hpp"
#include "noisy_turtle_playback/GaussianNoiseGenerator.hpp"

//STD
#include <tuple>

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>

namespace noisy_turtle_playback {

NoisyTurtlePlayback::NoisyTurtlePlayback(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
                                      &NoisyTurtlePlayback::topicCallback, this);
  publisher_ = nodeHandle.advertise<geometry_msgs::Twist>(publisherTopic_, queueSize_);
  p_gaussianNoiseGenerator_ = new GaussianNoiseGenerator(varLinX_, varAngZ_, covariance_);
  ROS_INFO("Successfully launched node.");
}

NoisyTurtlePlayback::~NoisyTurtlePlayback()
{
  delete p_gaussianNoiseGenerator_;
}

bool NoisyTurtlePlayback::readParameters()
{
  if (!nodeHandle_.getParam("odom_topic_name", subscriberTopic_)) return false;
  if (!nodeHandle_.getParam("queue_size", queueSize_)) return false;
  if (!nodeHandle_.getParam("vel_pub_topic_name", publisherTopic_)) return false;
  if (!nodeHandle_.getParam("var_lin_x", varLinX_)) return false;
  if (!nodeHandle_.getParam("var_ang_z", varAngZ_)) return false;
  if (!nodeHandle_.getParam("covariance", covariance_)) return false;
  return true;
}

void NoisyTurtlePlayback::topicCallback(const nav_msgs::Odometry& messageIn)
{
  double noiseLinX, noiseAngZ;
  /// Generates some Gaussian Noise according to given Covariance Matrix and stores it
  std::tie(noiseLinX, noiseAngZ) = p_gaussianNoiseGenerator_ -> generateGaussianNoise();

  geometry_msgs::Twist messageOut = messageIn.twist.twist;

  /// Adds the generated Gaussian Noise to incoming message
  messageOut.linear.x += noiseLinX;
  messageOut.angular.z += noiseAngZ;

  /// Publishes the message with Gaussian Noise added to incoming velocity description
  publisher_.publish(messageOut);
}

} /* namespace */