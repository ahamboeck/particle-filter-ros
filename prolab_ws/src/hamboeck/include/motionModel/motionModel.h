#pragma once

#include "../../include/utils/utils.h"
#include <random>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

class MotionModel
{
public:
    MotionModel() : gen(std::random_device{}()) {}

    void sampleMotionModel(double &x, double &y, double &theta, double v, double w, double dt, double var_v, double var_w) const
    {
        // Add noise to the control inputs
        double v_hat = v + sampleNormalDistribution(var_v);
        double w_hat = w + sampleNormalDistribution(var_w);

        // Update the particle state with the motion model
        if (fabs(w_hat) > 1e-6)
        { // Avoid division by zero for very small angular velocities
            x += -v_hat / w_hat * sin(theta) + v_hat / w_hat * sin(theta + w_hat * dt);
            y += v_hat / w_hat * cos(theta) - v_hat / w_hat * cos(theta + w_hat * dt);
        }
        else
        { // Straight line motion
            x += v_hat * dt * cos(theta);
            y += v_hat * dt * sin(theta);
        }
        
        theta += w_hat * dt;

        // Normalize the orientation angle
        theta = Utils::normalizeAngle(theta);

        // Check all the v_hat and w_hat values
        // ROS_INFO("v_hat = %f, w_hat = %f", v_hat, w_hat);

        // Check the noise values
        // ROS_INFO("sampleNormalDistribution(var_v) = %f, sampleNormalDistribution(var_w) = %f", sampleNormalDistribution(var_v), sampleNormalDistribution(var_w));
    }

private:
    mutable std::mt19937 gen;

    double sampleNormalDistribution(double variance) const
    {
        std::normal_distribution<> dist(0.0, std::sqrt(variance));
        return dist(gen);
    }
};
