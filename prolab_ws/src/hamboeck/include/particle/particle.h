#pragma once

class Particle
{
public:
    Particle(){};
    Particle(double x = 0, double y = 0, double theta = 0, double weight = 1.0)
        : x(x), y(y), theta(theta), weight(weight) {}
    ~Particle(){};

    double x, y, theta;
    double weight;
};