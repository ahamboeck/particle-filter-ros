#pragma once
class Particle
{
public:
    Particle() : x(0.0), y(0.0), theta(0.0), weight(1.0) {}
    
    void setPose(double x, double y, double theta) {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    void getPose(double& x, double& y, double& theta) const {
        x = this->x;
        y = this->y;
        theta = this->theta;
    }

    void setWeight(double weight) {
        this->weight = weight;
    }

    double getWeight() const {
        return weight;
    }

    void setID(int id) {
        this->particle_id = id;
    }

    int getID() const {
        return particle_id;
    }
private:
    double x, y, theta; // Position and orientation
    double weight;      // Importance weight of this particle
    int particle_id;    // Unique identifier for this particle
};
