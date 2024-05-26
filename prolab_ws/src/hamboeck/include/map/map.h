#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <vector>

class Map {
public:
    int width, height;
    double resolution;
    std::vector<int8_t> grid;
    double origin_x, origin_y;

    void update(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        width = msg->info.width;
        height = msg->info.height;
        resolution = msg->info.resolution;
        origin_x = msg->info.origin.position.x;
        origin_y = msg->info.origin.position.y;
        grid.assign(msg->data.begin(), msg->data.end());

        // ROS_INFO("Map updated: %dx%d, res: %f", width, height, resolution);
    }

    int getCellValue(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return grid[y * width + x];
        }
        return -1; // Return -1 for out-of-bounds
    }
};
