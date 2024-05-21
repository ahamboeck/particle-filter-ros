#pragma once
#include <cmath>

class Utils
{
public:
    /**
     * @brief Normalizes an angle to the range [-π, π].
     *
     * This function takes an angle in radians and normalizes it to the range [-π, π].
     *
     * @param angle The angle to be normalized.
     * @return The normalized angle in the range [-π, π].
     */
    static double normalizeAngle(double angle)
    {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0)                          
            angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    // static double normalize_angle(double angle);
    // static double euclidean_distance(double x1, double y1, double x2, double y2);
    // static double gaussian(double mu, double sigma, double x);
};