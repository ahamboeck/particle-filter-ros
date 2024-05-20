#include "ros/ros.h"
#include "../include/particleFilter/particle.h"
#include "../include/localizationNode/localizationNode.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization_node");

    LocalizationNode node; // This sets up and runs everything

    ros::spin();
    return 0;
}