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
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
        pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose", 10);
        marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("particle_marker_array", 1);
        odometry_subscriber = nh.subscribe("odom", 1, &LocalizationHandler::odometryCallback, this);
        sensor_data_subscriber = nh.subscribe("scan", 1, &LocalizationHandler::scanCallback, this);
        map_subscriber = nh.subscribe("map", 1, &LocalizationHandler::mapCallback, this);

        // Initialize the particle filter with parameters
        int num_particles;
        nh.getParam("initial_x_min", x_min);
        nh.getParam("initial_x_max", x_max);
        nh.getParam("initial_y_min", y_min);
        nh.getParam("initial_y_max", y_max);
        nh.getParam("initial_theta_min", theta_min);
        nh.getParam("initial_theta_max", theta_max);
        nh.getParam("num_particles", num_particles);
        nh.getParam("sigma", sigma);
        ROS_INFO("sigma: %f", sigma);  
        nh.getParam("percentage_rand_particles", percentage_rand_particles);
        ROS_INFO("percentage_rand_particles: %f", percentage_rand_particles);
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
    ros::Publisher marker_array_publisher;
    ros::Subscriber odometry_subscriber;
    ros::Subscriber sensor_data_subscriber;
    ros::Subscriber map_subscriber;

    double x_min, x_max, y_min, y_max, theta_min, theta_max;
    double var_v, var_w;
    double percentage_rand_particles;
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

        // ROS_INFO("Odometry pose: x = %f, y = %f, theta = %f", x, y, theta);

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
        filter.resample(x_min, x_max, y_min, y_max, theta_min, theta_max, percentage_rand_particles); // Resample with 10% random particles

        // Debugging: Print weights after update
        // for (size_t i = 0; i < particles.size(); ++i)
        // {
        //     ROS_INFO("Particle %zu: Weight after update = %f", i, particles[i].getWeight());
        // }
        publishParticles();
        // filter.updateWeights(*msg);
        // filter.resample();
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
        // ROS_INFO("Published pose estimate: x = %f, y = %f, theta = %f", x_avg, y_avg, theta_avg);
    }

    void publishParticles()
    {
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0; // Keep track of individual marker IDs within the array

        std::vector<Particle> particles = filter.getParticles();
        for (const auto &particle : particles)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "particle_arrows";
            marker.id = marker_id++; // Assign and increment the marker ID
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            // Scale factor based on particle weight
            double scale_factor = std::max(particle.getWeight() * 10, 0.5); // Ensuring a minimum size
            marker.scale.x = 0.1 * scale_factor;                            // Shaft diameter
            marker.scale.y = 0.2 * scale_factor;                            // Head diameter
            marker.scale.z = 0.15 * scale_factor;                           // Head length

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;

            double x, y, theta;
            particle.getPose(x, y, theta);

            // Set the position of the marker
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0.0;

            // Calculate the quaternion from theta
            tf2::Quaternion quat;
            quat.setRPY(0, 0, theta); // Roll, pitch, yaw
            marker.pose.orientation = tf2::toMsg(quat);

            // Define arrow length within the marker's local coordinate frame
            double arrow_length = 1.5 * scale_factor; // Arrow length increases with weight

            // Set the points for the arrow relative to the local coordinate frame
            geometry_msgs::Point start_point, end_point;
            start_point.x = 0; // Start at the local origin
            start_point.y = 0;
            start_point.z = 0;
            end_point.x = arrow_length; // Extend in the x-direction of the local frame
            end_point.y = 0;
            end_point.z = 0;

            marker.points.push_back(start_point);
            marker.points.push_back(end_point);

            marker_array.markers.push_back(marker);
        }

        marker_array_publisher.publish(marker_array); // Publish the entire array
    }
};
