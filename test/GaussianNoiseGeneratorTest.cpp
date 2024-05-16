#include "noisy_turtle_playback/GaussianNoiseGenerator.hpp"

// gtest
#include <gtest/gtest.h>

// STD
//#include <vector>
//#include <tuple>

using namespace noisy_turtle_playback;

TEST(GaussianNoiseGenerator, checkMeanAndCovariances)
{
    const int n = 100000; //number of data points used for testing
    const double expected_var_x = 34.0;
    const double expected_var_y = 50.0;
    const double expected_cov_xy = 12.0;
    GaussianNoiseGenerator gaussianNoiseGenerator(expected_var_x, expected_var_y, expected_cov_xy);
    std::array<std::pair<double, double>, n> gaussianNoise;
    double sum_x = 0, sum_y = 0;
    for(auto it = gaussianNoise.begin(); it != gaussianNoise.end(); it++) {
        *it = gaussianNoiseGenerator.generateGaussianNoise();
        sum_x += (*it).first;
        sum_y += (*it).second;
    }
    double mean_x = sum_x/n;
    double mean_y = sum_y/n;
    double var_x = 0, var_y = 0, cov_xy = 0;
    for(auto it = gaussianNoise.begin(); it != gaussianNoise.end(); it++) {
        var_x += std::pow(((*it).first - mean_x), 2);
        var_y += std::pow(((*it).second - mean_y), 2);
        cov_xy += ((*it).first - mean_x)*((*it).second - mean_y);
    }
    var_x /= n;
    var_y /= n;
    cov_xy /= n;
    EXPECT_NEAR(0.0, mean_x, 0.5);
    EXPECT_NEAR(0.0, mean_y, 0.5);

    EXPECT_NEAR(expected_var_x, var_x, 0.5);
    EXPECT_NEAR(expected_var_y, var_y, 0.5);
    EXPECT_NEAR(expected_cov_xy, cov_xy, 0.5);
}
