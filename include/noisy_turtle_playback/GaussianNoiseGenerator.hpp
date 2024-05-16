/**
 * @file GaussianNoiseGenerator.hpp
 * @author Sandip Das (sd13535@outlook.com)
 * @brief Header class containing methods that uses Cholesky Decomposition to generate 2D Gaussian Noise of a given Covariance matrix with mean (0, 0)
 * @version 0.1
 * @date 2024-05-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <vector>
#include <utility>
#include <random>

namespace noisy_turtle_playback {

/**
 * @brief Class containing methods to generate 2D Gaussian Noise with given Variances and Covariance
 * 
 */
class GaussianNoiseGenerator
{
  public:
  
  /**
   * @brief Construct a new Gaussian Noise Generator object
   * 
   * @param var_x Variance of the first quantity (let, x)
   * @param var_y Variance of the second quantity (let, y)
   * @param cov_xy Covariance between the two quantities
   */
  GaussianNoiseGenerator(double var_x, double var_y, double cov_xy);

  /**
   * @brief Destroy the Gaussian Noise Generator object
   * 
   */
  virtual ~GaussianNoiseGenerator();

  /**
   * @brief Generates a Sample point, using Cholesky Decomposition of Covariance Matrix, that follows the Bivariate Gaussian Distribution described by `var_x`, `var_y`, and `cov_xy`
   * 
   * @return std::pair<double, double> A sample point from the Bivariate Gaussian Distribution described by `var_x`, `var_y` and `cov_xy`
   */
  std::pair<double, double> generateGaussianNoise();

  private:

  /// Variance of the first quantity (let, x)
  const double var_x_;
  /// Variance of the second quantity (let, y)
  const double var_y_;
  /// Covariance between the two quantities
  const double cov_xy_;

  /**
   * @brief Uses Box-Muller Transform to create a Sample point following the 2D Standard(i.e. having the Identity Matrix as its Covariance Matrix) Normal Distribution
   * 
   * @return std::pair<double, double> (the sample point as a pair of doubles in the format \f$ (x, y) \f$ )
   */
  std::pair<double, double> biVariateStandardNormal();

  std::uniform_real_distribution<double> uniform_dis_;
  std::random_device rd_;

  /**
   * @brief generates a random number between 0 to 1 following an uniform probability density between 0 and 1 (and zero elsewhere)
   * 
   * Uses the a std::random_device object fed into a std::uniform_real_distribution<double> object
   * 
   * @return the generated random number
   */
  double generateRandomUniform0to1();
};

} /* namespace */