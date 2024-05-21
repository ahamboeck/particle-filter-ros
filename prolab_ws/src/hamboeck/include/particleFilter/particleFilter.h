#include "../particle/particle.h"
#include <vector>
#include <random>
#include <chrono> // Include this for time-based seeding

class ParticleFilter {
private:
    std::vector<Particle> particles;
    int num_particles; // Number of particles
    std::default_random_engine generator; // Random engine for normal distributions

public:
    ParticleFilter(int num = 100, unsigned seed = std::chrono::system_clock::now().time_since_epoch().count())
        : num_particles(num), generator(seed) {
        // Initialize particles
        std::uniform_real_distribution<double> distribution_x(-5.0, 5.0);
        std::uniform_real_distribution<double> distribution_y(-5.0, 5.0);
        std::uniform_real_distribution<double> distribution_theta(-M_PI, M_PI);

        for (int i = 0; i < num_particles; ++i) {
            double x = distribution_x(generator);
            double y = distribution_y(generator);
            double theta = distribution_theta(generator);
            particles.push_back(Particle(x, y, theta));
        }
    }
};