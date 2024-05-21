#include "../../include/localizationNode/localizationNode.h"
#include "../../include/particleFilter/particle.h"
#include <cmath>

LocalizationNode::LocalizationNode() : nh("~")
{
    nh.param("number_of_particles", num_particles, 10);
    nh.param("map_width", map_width, 88.0); // Default values
    nh.param("map_height", map_height, 80.0);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("estimated_pose", 10);
    sub_odom = nh.subscribe("/odom", 1, &LocalizationNode::odometryCallback, this);
    sub_scan = nh.subscribe("/scan", 1, &LocalizationNode::sensorCallback, this);
    sub_map = nh.subscribe("/map", 1, &LocalizationNode::mapCallback, this);

    initialize_particles(num_particles, map_width, map_height);
    ROS_INFO("LocalizationNode initialized with %d particles", num_particles);
}

void LocalizationNode::initialize_particles(int num_particles, double map_width, double map_height)
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
    ROS_INFO("Particles initialized");
}

void LocalizationNode::normalizeWeights()
{
    double total_weight = std::accumulate(particles.begin(), particles.end(), 0.0,
                                          [](double sum, const Particle &p)
                                          { return sum + p.weight; });

    if (total_weight > 0)
    {
        for (auto &particle : particles)
        {
            particle.weight /= total_weight;
        }
        ROS_INFO("Weights normalized");
    }
    else
    {
        ROS_WARN("Total weight is zero after normalization");
    }
}

void LocalizationNode::resampleParticles()
{
    std::vector<Particle> new_particles;
    double total_weight = 0.0;
    for (const auto &particle : particles)
    {
        ROS_INFO("Particle weight during resampling: %f", particle.weight);
        total_weight += particle.weight;
    }

    // ROS_INFO("Total weight: %f", total_weight);

    if (total_weight == 0)
    {
        ROS_WARN("Total weight is zero during resampling");
        return;
    }

    double step = total_weight / particles.size();
    double position = step * (rand() / double(RAND_MAX));
    double cum_sum = 0.0;
    size_t index = 0;

    for (const auto &particle : particles)
    {
        cum_sum += particle.weight;
        while (cum_sum > position)
        {
            new_particles.push_back(particle);
            position += step;
            if (new_particles.size() == particles.size())
                break;
        }
        if (new_particles.size() == particles.size())
            break;
    }

    particles = new_particles;
    ROS_INFO("Particles resampled");
}

geometry_msgs::PoseStamped LocalizationNode::estimate_pose()
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
    if (total_weight > 0)
    {
        estimated_pose.pose.position.x = sum_x / total_weight;
        estimated_pose.pose.position.y = sum_y / total_weight;
        estimated_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(sum_sin, sum_cos));
    }
    else
    {
        ROS_WARN("Total weight is zero during pose estimation");
    }

    return estimated_pose;
}

void LocalizationNode::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double v = msg->twist.twist.linear.x;
    double omega = msg->twist.twist.angular.z;
    double dt = 0.1;

    for (auto &particle : particles)
    {
        particle.move(v, omega, dt);
        // ROS_INFO("Particle moved: x = %f, y = %f, theta = %f", particle.x, particle.y, particle.theta);
    }
    ROS_INFO("Odometry callback executed: v = %f, omega = %f", v, omega);
}

void LocalizationNode::sensorCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if (msg->ranges.empty())
    {
        ROS_WARN("Received empty scan data.");
        return;
    }

    double total_weight = 0.0;

    // Update all particle weights based on the sensor data
    for (auto &particle : particles)
    {
        particle.updateWeight(msg->ranges, msg->angle_min, msg->angle_increment, map, msg->range_max);
        ROS_INFO("Particle weight after sensor update: %f", particle.weight);
        total_weight += particle.weight;
    }

    // Normalize the weights to ensure they sum to one
    if (total_weight > 0)
    {
        normalizeWeights();
    }
    else
    {
        ROS_WARN("Total weight is zero after updating weights; check sensor model or data.");
    }

    // Resample based on the normalized weights
    resampleParticles();

    // Estimate the new pose and publish
    geometry_msgs::PoseStamped estimated_pose = estimate_pose();
    pose_pub.publish(estimated_pose);
    ROS_INFO("Estimated Pose: x = %f, y = %f", estimated_pose.pose.position.x, estimated_pose.pose.position.y);
}

void LocalizationNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map.setMap(msg->data, msg->info.width, msg->info.height, msg->info.resolution);
    ROS_INFO("Map callback executed");
}
