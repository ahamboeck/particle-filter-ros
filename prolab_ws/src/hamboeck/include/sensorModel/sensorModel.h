#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include "../particle/particle.h"

class SensorModel
{
public:
    SensorModel() {}

    // Method to update particle weights based on the laser scan data
    void updateParticleWeights(std::vector<Particle> &particles, const sensor_msgs::LaserScan &scan, double sigma)
    {
        const double sigma_sq = sigma * sigma;
        const double gaussian_norm = std::log(sigma * std::sqrt(2.0 * M_PI));

        for (auto &particle : particles)
        {
            double log_weight = 0.0;
            for (size_t i = 0; i < scan.ranges.size(); ++i)
            {
                double range = scan.ranges[i];
                if (range > scan.range_min && range < scan.range_max)
                {
                    double beam_angle = particle.getTheta() + scan.angle_min + i * scan.angle_increment;
                    double expected_range = raycast(particle.getX(), particle.getY(), beam_angle);
                    double range_difference = range - expected_range;
                    double log_prob = -0.5 * (range_difference * range_difference / sigma_sq) - gaussian_norm;

                    log_weight += log_prob;
                }
            }
            // ROS_INFO("Particle %u: Log weight = %f", particle.getID(), log_weight);
            particle.setLogWeight(log_weight); // Directly set the particle's log weight
        }

        // Normalize log weights and convert to regular weights
        normalizeLogWeights(particles);
    }

    void updateMap(const nav_msgs::OccupancyGrid &map)
    {
        map_ = map;
    }

private:
    nav_msgs::OccupancyGrid map_;

    void normalizeLogWeights(std::vector<Particle> &particles)
    {
        double max_log_weight = -std::numeric_limits<double>::infinity();

        // Find the maximum log weight
        for (const auto &particle : particles)
        {
            if (particle.getLogWeight() > max_log_weight)
            {
                max_log_weight = particle.getLogWeight();
            }
        }

        // Subtract the maximum log weight and convert to regular weights
        double weight_sum = 0.0;
        for (auto &particle : particles)
        {
            double log_weight = particle.getLogWeight();
            log_weight -= max_log_weight;
            double weight = std::exp(log_weight);
            particle.setWeight(weight); // Temporarily store regular weight for summation
            weight_sum += weight;
        }

        // Normalize the weights to sum to 1
        if (weight_sum > 0)
        {
            for (auto &particle : particles)
            {
                particle.setWeight(particle.getWeight() / weight_sum); // Normalize weight
            }
        }
        else
        {
            ROS_WARN("Sum of particle weights is zero. Cannot normalize weights.");
        }
    }

    // Raycasting method to determine the expected measurement
    double raycast(double x, double y, double theta) const
    {
        int map_x = std::floor((x - map_.info.origin.position.x) / map_.info.resolution);
        int map_y = std::floor((y - map_.info.origin.position.y) / map_.info.resolution);
        int max_range_in_cells = std::ceil(10.0 / map_.info.resolution); // Example for 10 meter max range

        double step_size = 1; // Step by one cell at a time
        for (double step = 0; step < max_range_in_cells; step += step_size)
        {
            int check_x = map_x + std::round(step * std::cos(theta));
            int check_y = map_y + std::round(step * std::sin(theta));

            if (check_x < 0 || check_x >= map_.info.width ||
                check_y < 0 || check_y >= map_.info.height)
            {
                return step * map_.info.resolution; // Return the last valid step as max range
            }

            int index = check_y * map_.info.width + check_x;
            if (map_.data[index] == 100)
            { // Assuming 100 is occupied
                return step * map_.info.resolution;
            }
        }

        return max_range_in_cells * map_.info.resolution; // No obstacle within the max range
    }
};
