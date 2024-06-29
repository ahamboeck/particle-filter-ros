#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>

bool goalReached = true;
int goalId = 0;
double tolerance = 0.2; // Tolerance in meters
geometry_msgs::Pose currentPose;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose)
{
    currentPose = pose->pose.pose;
}

bool checkGoalReached(const geometry_msgs::Pose &goal)
{
    double dx = goal.position.x - currentPose.position.x;
    double dy = goal.position.y - currentPose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    return distance < tolerance;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle nh;

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    ros::Subscriber pose_sub = nh.subscribe("amcl_pose", 10, poseCallback);

    ros::Rate loop_rate(0.1);
    ros::Duration(1.0).sleep();

    while (ros::ok())
    {
        if (goalReached && goalId < 4)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();

            switch (goalId)
            {
            case 0:
                pose.pose.position.x = -2.0;
                pose.pose.position.y = 0.0;
                pose.pose.orientation.w = 1.0;
                break;
            case 1:
                pose.pose.position.x = -2.0;
                pose.pose.position.y = -2.0;
                pose.pose.orientation.w = 1.0;
                break;
            case 2:
                pose.pose.position.x = 0.0;
                pose.pose.position.y = -2.0;
                pose.pose.orientation.w = 1.0;
                break;
            case 3:
                pose.pose.position.x = 0.0;
                pose.pose.position.y = 0.0;
                pose.pose.orientation.w = 1.0;
                break;
            }

            pose_pub.publish(pose);
            ROS_INFO("Sending goal %d: Position (%.2f, %.2f)", goalId + 1, pose.pose.position.x, pose.pose.position.y);
            goalReached = false;
            goalId = (goalId + 1) % 4;
        }
        else if (!goalReached)
        {
            // Check if the goal has been reached within the tolerance
            if (checkGoalReached(currentPose))
            {
                ROS_INFO("Goal %d reached within tolerance.", goalId);
                goalReached = true;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
