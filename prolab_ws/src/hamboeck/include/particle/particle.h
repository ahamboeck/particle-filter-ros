#pragma once

#include <ros/ros.h>
class Particle
{
public:
    Particle() : x(0.0), y(0.0), theta(0.0), weight(1.0) {}

    void setPose(double x, double y, double theta)
    {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    void getPose(double &x, double &y, double &theta) const
    {
        x = this->x;
        y = this->y;
        theta = this->theta;
    }

    double getX() const
    {
        return x;
    }

    void setX(double x)
    {
        this->x = x;
    }

    double getY() const
    {
        return y;
    }

    void setY(double y)
    {
        this->y = y;
    }

    double getTheta() const
    {
        return theta;
    }

    void setTheta(double theta)
    {
        this->theta = theta;
    }

    double getWeight() const
    {
        return weight;
    }

    void setWeight(double weight)
    {
        this->weight = weight;
    }

    void setLogWeight(double log_likelihood)
    {
        log_weight = log_likelihood;
    }

    double getLogWeight() const
    {
        return log_weight;
    }

    void updateWeight(double likelihood)
    {   
        this->weight *= likelihood;
        ROS_INFO("Particle %u: Weight after update = %f", particle_id, weight);
    }

    void updateLogWeight(double log_likelihood)
    {
        log_weight += log_likelihood;
        ROS_INFO("Particle %u: Log weight after update = %f", particle_id, log_weight);
    }

    void setID(int id)
    {
        this->particle_id = id;
    }

    int getID() const
    {
        return particle_id;
    }

private:
    double x, y, theta; // Position and orientation
    double weight;      // Importance weight of this particle
    double log_weight;
    int particle_id; // Unique identifier for this particle
};
