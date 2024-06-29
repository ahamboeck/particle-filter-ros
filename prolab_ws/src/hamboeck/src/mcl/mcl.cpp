#include "../../include/utils/utils.h"
#include "../../include/particle/particle.h"
#include "../../include/particleFilter/particleFilter.h"
#include "../../include/localizationHandler/localizationHandler.h"

#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mcl_localization");
    LocalizationHandler handler;
    
    ros::Rate rate(10); // Loop at 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}