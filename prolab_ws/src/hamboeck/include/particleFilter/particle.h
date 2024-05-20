#pragma once

#include "../../include/utils/utils.h"  // Include the header where MapUtils is defined
#include "../../include/occupancyGrid/occupancyGrid.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <random>

class Particle
{
public:
    double x, y, theta; // Particle's position and orientation
    double weight;      // Particle's weight

    // Noise generation
    static std::default_random_engine generator;
    static std::normal_distribution<double> distribution_x;
    static std::normal_distribution<double> distribution_y;
    static std::normal_distribution<double> distribution_theta;

    // Constructor
    Particle(double x = 0.0, double y = 0.0, double theta = 0.0, double weight = 1.0);

    // Method to update the particle's state based on velocity commands and noise
    void move(double linear_velocity, double angular_velocity, double dt);
    void updateWeight(const std::vector<float> &actual_scan, double angle_min,
                 double angle_increment, const OccupancyGrid &map, double max_range);
    double noise_model_x();
    double noise_model_y();
    double noise_model_theta();
};