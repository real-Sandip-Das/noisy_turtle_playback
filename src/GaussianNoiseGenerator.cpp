/**
 * @file GaussianNoiseGenerator.cpp
 * @author Sandip Das (sd13535@outlook.com)
 * @brief Source file that implements the class containing member functions to generate 2D Gaussian Noise of given Covariance Matrix and mean (0, 0) using Cholesky Decomposition
 * @version 0.1
 * @date 2024-05-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <noisy_turtle_playback/GaussianNoiseGenerator.hpp>
#include <cmath>
#include <tuple>

namespace noisy_turtle_playback {

GaussianNoiseGenerator::GaussianNoiseGenerator(double var_x, double var_y, double cov_xy):
var_x_(var_x),
var_y_(var_y),
cov_xy_(cov_xy),
uniform_dis_(0.0, 1.0)
{
}

std::pair<double, double> GaussianNoiseGenerator::generateGaussianNoise()
{
  double x, y;
  std::tie(x, y) = biVariateStandardNormal();
  /**
   * If we know the Cholesky Decomposition of the Covariance Matrix (let, \f$\Sigma = LL^{\mathsf{T}}\f$)
   * 
   * We can use it to generate samples of a Gaussian Distribution of any Covariance Matrix
   * from samples of a Standard Normal Distribution
   * 
   * Finding out the Cholesky decomposition of a general 2D Covariance Matrix:
   * 
   * let, \f[ \Sigma = A^{(1)} = \begin{pmatrix} \sigma_{xx} && \sigma_{xy} \\ \sigma_{xy} && \sigma_{yy} \end{pmatrix} \f]
   * 
   * (assuming \f$ \sigma_{xx} \f$ and \f$ \sigma_{yy} \f$ are the variances of \f$ x \f$ and \f$ y \f$ respectively)
   * 
   * Clearly,
   * \f[
   * L_1 = \begin{pmatrix} \sqrt{\sigma_{xx}} && 0 \\ \frac{\sigma_{xy}}{\sqrt{\sigma_{xx}}} && 1 \end{pmatrix}
   * \f]
   * 
   * \f[
   * A^{(2)} = L_1^{-1}A^{(1)}(L_1^T)^{-1} = \begin{pmatrix} \frac{1}{\sqrt{\sigma_{xx}}} && 0 \\ -\frac{\sigma_{xy}}{\sigma_{xx}} && 1 \end{pmatrix} \begin{pmatrix} \sigma_{xx} && \sigma_{xy} \\ \sigma_{xy} && \sigma_{yy} \end{pmatrix} \begin{pmatrix} \frac{1}{\sqrt{\sigma_{xx}}} && -\frac{\sigma_{xy}}{\sigma_{xx}} \\ 0 && 1 \end{pmatrix}
   * \f]
   * \f[
   * = \begin{pmatrix} 1 && 0 \\ 0 && -\frac{\sigma_{xy}^2}{\sigma_{xx}}+\sigma_{yy} \end{pmatrix}
   * \f]
   * 
   * \f[
   * L_2 = \begin{pmatrix} 1 && 0 \\ 0 && \sqrt{-\frac{\sigma_{xy}^2}{\sigma_{xx}}+\sigma_{yy}} \end{pmatrix}
   * \f]
   * 
   * \f[
   * L = L_1L_2 = \begin{pmatrix} \sqrt{\sigma_{xx}} && 0 \\ \frac{\sigma_{xy}}{\sqrt{\sigma_{xx}}} && \sqrt{-\frac{\sigma_{xy}^2}{\sigma_{xx}}+\sigma_{yy}} \end{pmatrix}
   * \f]
   * 
   * \f[
   * \therefore \Sigma = LL^\mathsf{T}
   * \f]
   * 
   * Now that we have found out the Cholesky Decomposition, in order to generate Gaussian Noise of Covariance Matrix \f$\Sigma\f$,
   * from independent samples \f$X\sim\mathcal{N}(0,1),Y\sim\mathcal{N}(0,1)\f$, we need to multiply:
   * 
   * \f[
   * \begin{pmatrix}X' \\ Y'\end{pmatrix} = L\begin{pmatrix} X \\ Y\end{pmatrix} = \begin{pmatrix} \sqrt{\sigma_{xx}} x \\ \frac{\sigma_{xy}}{\sqrt{\sigma_{xx}}} x+\sqrt{-\frac{\sigma_{xy}^2}{\sigma_{xx}}+\sigma_{yy}} y\end{pmatrix}\sim\mathcal{N}(\mathbf{0},\Sigma)
   * \f]
   * 
   */
  double x1 = std::sqrt(var_x_)*x;
  double y1 = (cov_xy_*x)/std::sqrt(var_x_)+std::sqrt(var_y_-(std::pow(cov_xy_, 2))/var_x_)*y;
  return std::make_pair(x1, y1);
}

std::pair<double, double> GaussianNoiseGenerator::biVariateStandardNormal()
{
  /**
   * Brief description of the Box Muller Transform:
   * 
   * Suppose \f$ u_1 \f$ and \f$ u_2 \f$ be independent samples chosen from the uniform distribution on the unit interval \f$ (0, 1) \f$
   * and, let \f$ R = \sqrt{-2\ln{u_1}}, \Theta = 2\pi u_2 \f$
   * 
   * Then, \f$ Z_0 = R\cos\Theta, Z_1 = R\sin\Theta \f$ follow Standard Normal Distribution and their covariance is zero
   *
   * \f$ \therefore (Z_0, Z_1) \f$ follows the Bi variate Standard Normal distribution
   * 
   */
  double r = std::sqrt(-2*std::log(generateRandomUniform0to1()));
  double theta = 2*M_PI*generateRandomUniform0to1();
  return std::make_pair(r*std::cos(theta), r*std::sin(theta));
}

double GaussianNoiseGenerator::generateRandomUniform0to1()
{
  return uniform_dis_(rd_);
}

GaussianNoiseGenerator::~GaussianNoiseGenerator() = default;
} /* namespace */