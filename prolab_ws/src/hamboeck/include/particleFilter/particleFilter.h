#pragma once

#include "../motionModel/motionModel.h"
#include "../particle/particle.h"
#include "ros/ros.h"
#include <vector>
#include <random>
#include <chrono> // Include this for time-based seeding

class ParticleFilter
{
public:
    ParticleFilter(){};

    // Retrieves a copy of all particles
    std::vector<Particle> getParticles() const
    {
        return particles;
    }

    // Sets the current list of particles to a new list
    void setParticles(const std::vector<Particle> &new_particles)
    {
        particles = new_particles;
    }

    void initialize(double x_min, double x_max, double y_min, double y_max, double theta_min, double theta_max, int num_particles)
    {
        // Seed the random number generator
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> x_distribution(x_min, x_max);
        std::uniform_real_distribution<double> y_distribution(y_min, y_max);
        std::uniform_real_distribution<double> theta_distribution(theta_min, theta_max);

        // Initialize particles
        for (int i = 0; i < num_particles; i++)
        {
            Particle p;
            p.setPose(x_distribution(generator), y_distribution(generator), theta_distribution(generator));
            p.setWeight(1.0 / num_particles); // Uniform weight (I am not sure at this point if it should be normalized or not)
            p.setID(i);
            particles.push_back(p);
        }
    }

    void predict(double v, double w, double dt, const MotionModel &motion_model, double var_v, double var_w)
    {
        for (auto &particle : particles)
        {
            double x, y, theta;
            particle.getPose(x, y, theta);
            motion_model.sampleMotionModel(x, y, theta, v, w, dt, var_v, var_w);
            particle.setPose(x, y, theta);
            // ROS_INFO("Predicted pose of particle %d: x = %f, y = %f, theta = %f", particle.getID(), x, y, theta);
        }
    }

void resample(double x_min, double x_max, double y_min, double y_max, double theta_min, double theta_max, double percentage_rand_particles)
{
    std::vector<Particle> new_particles;
    std::vector<double> weights;
    for (const auto &particle : particles)
    {
        weights.push_back(particle.getWeight());
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> distribution(weights.begin(), weights.end());

    int num_random_particles = static_cast<int>(particles.size() * percentage_rand_particles);
    int num_resampled_particles = particles.size() - num_random_particles;

    // Standard deviations for Gaussian distribution around each particle
    double std_dev_position = 0.2; // Standard deviation for x and y
    double std_dev_theta = M_PI / 36; // Standard deviation for theta (5 degrees)

    std::normal_distribution<> gauss_x(0, std_dev_position);
    std::normal_distribution<> gauss_y(0, std_dev_position);
    std::normal_distribution<> gauss_theta(0, std_dev_theta);

    // Resample particles with added Gaussian noise
    for (int i = 0; i < num_resampled_particles; ++i)
    {
        Particle original = particles[distribution(gen)];
        double x, y, theta;
        original.getPose(x, y, theta);

        // Add Gaussian noise to the pose
        x += gauss_x(gen);
        y += gauss_y(gen);
        theta += gauss_theta(gen);

        // Ensure theta remains within the range [-π, π]
        theta = std::fmod(theta, 2 * M_PI);
        if (theta > M_PI)
            theta -= 2 * M_PI;
        else if (theta < -M_PI)
            theta += 2 * M_PI;

        Particle new_particle;
        new_particle.setPose(x, y, theta);
        new_particle.setWeight(1.0 / particles.size()); // Reset weights if necessary
        new_particles.push_back(new_particle);
    }

    // Add completely random particles
    std::uniform_real_distribution<double> dist_x(x_min, x_max);
    std::uniform_real_distribution<double> dist_y(y_min, y_max);
    std::uniform_real_distribution<double> dist_theta(theta_min, theta_max);
    for (int i = 0; i < num_random_particles; ++i)
    {
        Particle random_particle;
        random_particle.setPose(dist_x(gen), dist_y(gen), dist_theta(gen));
        random_particle.setWeight(1.0 / particles.size()); // Reset weights if necessary
        new_particles.push_back(random_particle);
    }

    particles = new_particles;
}


private:
    std::vector<Particle> particles;
};
