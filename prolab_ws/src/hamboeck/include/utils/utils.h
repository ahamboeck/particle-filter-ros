#pragma once

#include <cmath>
#include <vector>
#include "../../include/occupancyGrid/occupancyGrid.h"

class Utils
{
public:
    static double rayCast(double x, double y, double theta, const OccupancyGrid &map, double max_range)
    {
        double step_size = map.getResolution(); // Assuming getResolution() gives us the map resolution
        double distance = 0.0;
        while (distance < max_range)
        {
            double check_x = x + distance * cos(theta);
            double check_y = y + distance * sin(theta);

            if (map.isOccupied(check_x, check_y))
            {
                return distance;
            }
            distance += step_size;
        }
        return max_range;
    }

    static double calculateLikelihood(double actual, double predicted)
    {
        static const double std_dev = 0.1; // Standard deviation for the sensor noise
        static const double gauss_norm = 1.0 / sqrt(2.0 * M_PI * std_dev * std_dev);
        double diff = actual - predicted;
        return gauss_norm * exp(-0.5 * (diff * diff) / (std_dev * std_dev));
    }
};
