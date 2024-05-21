#include "../../include/particleFilter/particle.h"

#include <cmath> // For basic math functions

std::default_random_engine Particle::generator(std::chrono::system_clock::now().time_since_epoch().count());
std::normal_distribution<double> Particle::distribution_x(0.0, 1);            // Adjust std_dev as needed
std::normal_distribution<double> Particle::distribution_y(0.0, 1);            // Adjust std_dev as needed
std::normal_distribution<double> Particle::distribution_theta(0.0, M_PI / 4); // Adjust std_dev as needed // Adjust range as needed

Particle::Particle(double x, double y, double theta, double weight)
    : x(x), y(y), theta(theta), weight(weight) {}

void Particle::move(double v_c, double omega_c, double dt)
{
    // Update particle position based on odometry
    if (fabs(omega_c) < 1e-6)
    { // Straight motion
        x += v_c * dt * cos(theta);
        y += v_c * dt * sin(theta);
    }
    else
    { // Curved motion
        double new_theta = theta + omega_c * dt;
        x += -v_c / omega_c * (sin(theta) - sin(new_theta));
        y += v_c / omega_c * (cos(theta) - cos(new_theta));
        theta = new_theta;
    }

    // Add noise to the state
    x += noise_model_x();         // Noise for x position
    y += noise_model_y();         // Noise for y position
    theta += noise_model_theta(); // Noise for orientation
}

// Implement the weight update method
void Particle::updateWeight(const std::vector<float> &actual_scan, double angle_min, double angle_increment, const OccupancyGrid &map, double max_range)
{
    double log_likelihood_sum = 0.0;  // Start with log likelihood sum as 0
    for (size_t i = 0; i < actual_scan.size(); ++i)
    {
        double angle = this->theta + angle_min + i * angle_increment;
        double predicted_distance = Utils::rayCast(this->x, this->y, angle, map, max_range);
        double single_likelihood = Utils::calculateLikelihood(actual_scan[i], predicted_distance);
        log_likelihood_sum += log(single_likelihood);
    }

    this->weight = exp(log_likelihood_sum);  // Convert log likelihood sum back to normal scale
    if (this->weight < 1e-10)
    {
        this->weight = 1e-10; // Avoid weights being too small, which can cause numerical instability
    }
    ROS_INFO("Particle weight after sensor update: %f", this->weight);
}

// Example functions to add noise
double Particle::noise_model_x()
{
    return distribution_x(generator); // Generate and return noise for x position
}

double Particle::noise_model_y()
{
    return distribution_y(generator); // Generate and return noise for y position
}

double Particle::noise_model_theta()
{
    return distribution_theta(generator); // Generate and return noise for theta (orientation)
}
