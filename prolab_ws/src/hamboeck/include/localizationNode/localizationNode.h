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
    LocalizationNode();

    void initialize_particles(int num_particles, double map_width, double map_height);
    void normalizeWeights();
    void resampleParticles();
    geometry_msgs::PoseStamped estimate_pose();
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void sensorCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
};
