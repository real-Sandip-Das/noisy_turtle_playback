/**
 * @file NoisyTurtlePlayback.hpp
 * @author Sandip Das (sd13535@outlook.com)
 * @brief Header for the Node Class that will add Gaussian Noise to Odometry Readings and then Republish it to some other topic
 * @version 0.1
 * @date 2024-05-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "noisy_turtle_playback/GaussianNoiseGenerator.hpp"

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

namespace noisy_turtle_playback {

class NoisyTurtlePlayback {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  NoisyTurtlePlayback(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~NoisyTurtlePlayback();
 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void topicCallback(const nav_msgs::Odometry& message);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber for laser scan data.
  ros::Subscriber subscriber_;

  //! ROS topic publisher object for sending dummy messages
  ros::Publisher publisher_;

  //! ROS topic name to subscribe to for laser scan data.
  std::string subscriberTopic_;

  //! ROS topic name to publish dummy messages to
  std::string publisherTopic_;

  //! Maximum number of incoming/outgoing messages to be queued for delivery to subscribers
  int queueSize_;

  //! Variance of linear velocity in the x direction of added Gaussian Noise
  double varLinX_;

  //! Variance of angular velocity in the z direction of added Gaussian Noise
  double varAngZ_;

  //! Covariance between the two dimensionalities of added Gaussian Noise
  double covariance_;

  //! Pointer to the Gaussian Noise generator object
  GaussianNoiseGenerator* p_gaussianNoiseGenerator_;
};

} /* namespace */