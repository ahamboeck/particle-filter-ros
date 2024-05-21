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
            ROS_INFO("Predicted pose of particle %d: x = %f, y = %f, theta = %f", particle.getID(), x, y, theta);
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

        int num_random_particles = particles.size() * percentage_rand_particles; // 5% random particles
        for (int i = 0; i < particles.size() - num_random_particles; ++i)
        {
            new_particles.push_back(particles[distribution(gen)]);
        }

        // Add random particles
        std::uniform_real_distribution<double> dist_x(x_min, x_max);
        std::uniform_real_distribution<double> dist_y(y_min, y_max);
        std::uniform_real_distribution<double> dist_theta(theta_min, theta_max);
        for (int i = 0; i < num_random_particles; ++i)
        {
            Particle random_particle;
            random_particle.setPose(dist_x(gen), dist_y(gen), dist_theta(gen));
            random_particle.setWeight(1.0 / particles.size()); // Optional: uniform or small weight
            new_particles.push_back(random_particle);
        }

        particles = new_particles;
    }

private:
    std::vector<Particle> particles;
};
