#pragma once
#include "../../include/particleFilter/particleFilter.h" // Ensure this includes the definition of your ParticleFilter class
#include "../../include/sensorModel/sensorModel.h"       // Ensure this includes the definition of your SensorModel class
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class LocalizationHandler
{
public:
    LocalizationHandler()
    {
        initialize();
    }

    void initialize()
    {
        ros::NodeHandle nh;
        pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("mcl_pose_estimate", 10);
        marker_publisher = nh.advertise<visualization_msgs::Marker>("particle_markers", 10);
        odometry_subscriber = nh.subscribe("odom", 1, &LocalizationHandler::odometryCallback, this);
        sensor_data_subscriber = nh.subscribe("scan", 1, &LocalizationHandler::scanCallback, this);
        map_subscriber = nh.subscribe("map", 1, &LocalizationHandler::mapCallback, this);
        // Initialize the particle filter with parameters
        double x_min, x_max, y_min, y_max, theta_min, theta_max;
        int num_particles;
        nh.getParam("initial_x_min", x_min);
        nh.getParam("initial_x_max", x_max);
        nh.getParam("initial_y_min", y_min);
        nh.getParam("initial_y_max", y_max);
        nh.getParam("initial_theta_min", theta_min);
        nh.getParam("initial_theta_max", theta_max);
        nh.getParam("num_particles", num_particles);
        nh.getParam("sigma", sigma);
        ROS_INFO("Sigma: %f", sigma);
         // Default to 0.1 if not set
        // Initialize the motion model parameters
        nh.getParam("var_v", var_v);
        nh.getParam("var_w", var_w);

        filter.initialize(x_min, x_max, y_min, y_max, theta_min, theta_max, num_particles);

        // Initialize last_time to zero time
        last_time = ros::Time(0);
    }
    // void predictionStep(const Odometry &odometry);
    // void correctionStep(const SensorData &data);
    // void resampleParticles();
    // void publishPose() {
    //     geometry_msgs::PoseWithCovarianceStamped pose_msg;
    //     // pose_msg.pose.pose.position.x = ...;
    //     // pose_msg.pose.pose.position.y = ...;
    //     // pose_msg.pose.pose.orientation.z = ...;
    //     // pose_msg.pose.pose.orientation.w = ...;
    //     pose_publisher.publish(pose_msg);
    // }

private:
    ParticleFilter filter;
    MotionModel motion_model;
    SensorModel sensor_model;
    ros::Time last_time;
    ros::Publisher pose_publisher;
    ros::Publisher marker_publisher;
    ros::Subscriber odometry_subscriber;
    ros::Subscriber sensor_data_subscriber;
    ros::Subscriber map_subscriber;

    double var_v, var_w;
    double sigma;

    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        ros::Time current_time = msg->header.stamp;

        // Handle first message case
        if (last_time.toSec() == 0)
        {
            last_time = current_time;
            return;
        }

        // Extract the ground truth pose from the odometry message
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = tf::getYaw(msg->pose.pose.orientation);

        ROS_INFO("Odometry pose: x = %f, y = %f, theta = %f", x, y, theta);

        // Extract the control inputs from the odometry message
        double v = msg->twist.twist.linear.x;                // Linear velocity
        double w = msg->twist.twist.angular.z;               // Angular velocity
        double dt = (msg->header.stamp - last_time).toSec(); // Time difference since last update
        last_time = msg->header.stamp;

        // ROS_INFO("Odometry callback called! v: %f, w: %f", v, w);

        filter.predict(v, w, dt, motion_model, var_v, var_w);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        std::vector<Particle> particles = filter.getParticles();

        // Debugging: Print weights before update
        // for (size_t i = 0; i < particles.size(); ++i)
        // {
        //     ROS_INFO("Particle %zu: Weight before update = %f", i, particles[i].getWeight());
        // }

        sensor_model.updateParticleWeights(particles, *msg, sigma);
        normalizeWeights(particles);
        filter.setParticles(particles);
        filter.resample();
        publishParticles();
        publishPose();
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        // Update the map in the sensor model
        ROS_INFO("Map callback called!");
        sensor_model.updateMap(*msg);
    }

    // Raycasting method to determine the expected measurement
    void normalizeWeights(std::vector<Particle> &particles)
    {
        double sum_weights = 0.0;
        for (const auto &particle : particles)
        {
            sum_weights += particle.getWeight();
        }
        if (sum_weights > 0)
        {
            for (auto &particle : particles)
            {
                particle.setWeight(particle.getWeight() / sum_weights);
            }
        }
        else
        {
            ROS_WARN("Sum of particle weights is zero. Cannot normalize weights.");
        }
    }

    // Publish the estimated pose of the robot as marker, use the 100 weighted particles, their average pose as the estimated pose and use the color red
    void publishPose()
    {
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = ros::Time::now();

        std::vector<Particle> particles = filter.getParticles();
        if (particles.empty())
        {
            ROS_WARN("No particles available to calculate pose.");
            return;
        }

        double x_sum = 0.0, y_sum = 0.0, theta_sum = 0.0;
        for (const auto &particle : particles)
        {
            double x, y, theta;
            particle.getPose(x, y, theta);
            x_sum += x;
            y_sum += y;
            theta_sum += theta;
        }

        double n = static_cast<double>(particles.size());
        double x_avg = x_sum / n;
        double y_avg = y_sum / n;
        double theta_avg = theta_sum / n;

        double sum_sq_x = 0.0, sum_sq_y = 0.0, sum_sq_theta = 0.0;
        for (const auto &particle : particles)
        {
            double x, y, theta;
            particle.getPose(x, y, theta);
            sum_sq_x += std::pow(x - x_avg, 2);
            sum_sq_y += std::pow(y - y_avg, 2);
            sum_sq_theta += std::pow(theta - theta_avg, 2);
        }

        double var_x = sum_sq_x / n;
        double var_y = sum_sq_y / n;
        double var_theta = sum_sq_theta / n;

        // Populate the PoseWithCovariance
        pose_msg.pose.pose.position.x = x_avg;
        pose_msg.pose.pose.position.y = y_avg;
        pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_avg);

        // Set covariance
        // For simplicity, assume no correlation between different dimensions
        pose_msg.pose.covariance[0] = var_x;      // Variance of x
        pose_msg.pose.covariance[7] = var_y;      // Variance of y
        pose_msg.pose.covariance[35] = var_theta; // Variance of theta (yaw)

        // Publish the message
        pose_publisher.publish(pose_msg);
        ROS_INFO("Published pose estimate: x = %f, y = %f, theta = %f", x_avg, y_avg, theta_avg);
    }

    void publishParticles()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "particles";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.5; // Size of the points
        marker.scale.y = 0.5;

        // Set the color of the points (red)
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        // Add points for each particle
        std::vector<Particle> particles = filter.getParticles();
        for (const auto &particle : particles)
        {
            geometry_msgs::Point p;
            double x, y, theta;
            particle.getPose(x, y, theta);
            p.x = x;
            p.y = y;
            p.z = 0.0; // Assuming particles are on the ground plane
            marker.points.push_back(p);
        }

        marker_publisher.publish(marker);
    }
};
