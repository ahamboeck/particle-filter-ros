#pragma once

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "../../include/particleFilter/particle.h"
#include <vector>
#include <chrono>

class LocalizationNode
{
private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Subscriber sub_odom, sub_scan, sub_map;
    std::vector<Particle> particles;
    OccupancyGrid map;
    int num_particles;
    double map_width, map_height;

public:
    LocalizationNode() : nh("~")
    {
        nh.param("number_of_particles", num_particles, 100);
        nh.param("map_width", map_width, 88.0); // Default values
        nh.param("map_height", map_height, 80.0);

        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("estimated_pose", 10);
        sub_odom = nh.subscribe("odom", 1000, &LocalizationNode::odometryCallback, this);
        sub_scan = nh.subscribe("scan", 1000, &LocalizationNode::sensorCallback, this);
        sub_map = nh.subscribe("map", 1, &LocalizationNode::mapCallback, this);

        initialize_particles(num_particles, map_width, map_height);
    }

    void initialize_particles(int num_particles, double map_width, double map_height)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution_x(0, map_width);
        std::uniform_real_distribution<double> distribution_y(0, map_height);
        std::uniform_real_distribution<double> distribution_theta(0.0, 2 * M_PI);

        particles.clear();
        for (int i = 0; i < num_particles; i++)
        {
            double init_x = distribution_x(generator);
            double init_y = distribution_y(generator);
            double init_theta = distribution_theta(generator);
            particles.emplace_back(init_x, init_y, init_theta);
        }
    }

    void resampleParticles()
    {
        std::vector<Particle> new_particles;
        double total_weight = 0.0;
        for (const auto &particle : particles)
        {
            total_weight += particle.weight;
        }

        // Calculate the interval
        double step = total_weight / particles.size();
        double position = step * (rand() / double(RAND_MAX)); // Random start within step

        // Cumulative sum
        double cum_sum = 0.0;
        size_t index = 0;

        // Systematic resampling
        for (const auto &particle : particles)
        {
            cum_sum += particle.weight;
            while (cum_sum > position)
            {
                new_particles.push_back(particle); // Add particle to the new set
                position += step;
                if (new_particles.size() == particles.size())
                    break;
            }
            if (new_particles.size() == particles.size())
                break;
        }

        particles = new_particles; // Replace old particles with the resampled ones
    }

    geometry_msgs::PoseStamped estimate_pose()
    {
        double sum_x = 0, sum_y = 0, sum_cos = 0, sum_sin = 0;
        double total_weight = 0;

        for (const auto &particle : particles)
        {
            sum_x += particle.x * particle.weight;
            sum_y += particle.y * particle.weight;
            sum_cos += cos(particle.theta) * particle.weight;
            sum_sin += sin(particle.theta) * particle.weight;
            total_weight += particle.weight;
        }

        geometry_msgs::PoseStamped estimated_pose;
        estimated_pose.pose.position.x = sum_x / total_weight;
        estimated_pose.pose.position.y = sum_y / total_weight;
        estimated_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(sum_sin, sum_cos));

        return estimated_pose;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        double v = msg->twist.twist.linear.x;      // Linear velocity
        double omega = msg->twist.twist.angular.z; // Angular velocity
        double dt = 0.1;                           // Example time step, adjust based on your actual update rate

        for (auto &particle : particles)
        {
            particle.move(v, omega, dt);
        }
    }

    void sensorCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        double total_weight = 0.0;

        // First, update all particle weights based on the sensor data
        for (auto &particle : particles)
        {
            particle.updateWeight(msg->ranges, msg->angle_min, msg->angle_increment, map, msg->range_max);
            total_weight += particle.weight;
        }

        // Normalize the weights
        if (total_weight > 0)
        { // Avoid division by zero
            for (auto &particle : particles)
            {
                particle.weight /= total_weight;
            }
        }

        // Optionally, perform resampling based on the normalized weights
        resampleParticles();

        geometry_msgs::PoseStamped estimated_pose = estimate_pose();
        ROS_INFO("Estimated Pose: x = %f, y = %f", estimated_pose.pose.position.x, estimated_pose.pose.position.y);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        map.setMap(msg->data, msg->info.width, msg->info.height, msg->info.resolution);
    }
};
