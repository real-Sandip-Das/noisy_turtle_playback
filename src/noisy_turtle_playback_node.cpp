/**
 * @file noisy_turtle_playback_node.cpp
 * @author Sandip Das (sd13535@outlook.com)
 * @brief Source file containing main function for running the Noise adder Node (just calls methods of the Node class)
 * @version 0.1
 * @date 2024-05-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <ros/ros.h>
#include "noisy_turtle_playback/NoisyTurtlePlayback.hpp"

/**
 * @brief The main function for running the noisy_turtle_playback Node
 * 
 * @return int 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle nodeHandle("~");

  noisy_turtle_playback::NoisyTurtlePlayback noiseAdderNode(nodeHandle);

  ros::spin();
  return 0;
}