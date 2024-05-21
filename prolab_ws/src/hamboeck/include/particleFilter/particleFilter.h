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
    // void updateWeights(const SensorModel& sensor_model, const SensorData& data);
    // void resample();

private:
    std::vector<Particle> particles;
};
