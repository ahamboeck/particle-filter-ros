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
        odometry_subscriber = nh.subscribe("odom", 50, &LocalizationHandler::odometryCallback, this);
        sensor_data_subscriber = nh.subscribe("scan", 50, &LocalizationHandler::scanCallback, this);
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
        nh.param("sigma", sigma, 0.0);  
        nh.param("percentage_rand_particles", percentage_rand_particles, 0.0);
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
        filter.resample(x_min, x_max, y_min, y_max, theta_min, theta_max, percentage_rand_particles); // Resample with 10% random particles

        // Debugging: Print weights after update
        // for (size_t i = 0; i < particles.size(); ++i)
        // {
        //     ROS_INFO("Particle %zu: Weight after update = %f", i, particles[i].getWeight());
        // }
        publishParticles();
        // filter.updateWeights(*msg);
        // filter.resample();
        // publishPose();
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
