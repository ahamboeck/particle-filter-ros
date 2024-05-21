#pragma once

#include <cmath>
#include <vector>
#include "../../include/occupancyGrid/occupancyGrid.h"

#include <ros/ros.h>

class Utils
{
public:
    static double rayCast(double x, double y, double theta, const OccupancyGrid &map, double max_range)
    {
        double step_size = map.getResolution();
        double distance = 0.0;
        while (distance < max_range)
        {
            double check_x = x + distance * cos(theta);
            double check_y = y + distance * sin(theta);

            int ix = static_cast<int>(check_x / map.getResolution());
            int iy = static_cast<int>(check_y / map.getResolution());

            if (ix < 0 || iy < 0 || ix >= map.getWidth() || iy >= map.getHeight())
            {
                // Log out-of-bounds conditions and treat as max range
                // ROS_WARN("RayCast out of bounds: check_x = %f, check_y = %f, ix = %d, iy = %d", check_x, check_y, ix, iy);
                return max_range;
            }

            if (map.isOccupied(check_x, check_y))
            {
                // ROS_INFO("RayCast hit: x = %f, y = %f, distance = %f", check_x, check_y, distance);
                return distance;
            }
            distance += step_size;
        }
        // ROS_INFO("RayCast max range: distance = %f", max_range);
        return max_range;
    }

    static double calculateLikelihood(double actual, double predicted)
    {
        static const double std_dev = 50.0; // Adjusted standard deviation for more tolerance
        static const double gauss_norm = 1.0 / sqrt(2.0 * M_PI * std_dev * std_dev);
        static const double max_sensor_range = 100.0; // Example max range
        static const double epsilon = 1e-3;           // Adjusted to handle maximum range readings

        if (isinf(actual) || actual >= max_sensor_range)
        {
            // Handling max range readings with a small but non-zero likelihood
            return (predicted >= max_sensor_range) ? epsilon : 1e-6;
        }

        double diff = actual - predicted;
        double likelihood = gauss_norm * exp(-0.5 * (diff * diff) / (std_dev * std_dev));
        // ROS_INFO("Actual: %f, Predicted: %f, Likelihood: %f", actual, predicted, likelihood);
        return std::max(likelihood, 1e-6); // Ensure likelihood is never zero
    }
};
