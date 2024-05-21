#include "../../include/occupancyGrid/occupancyGrid.h"

void OccupancyGrid::setMap(const std::vector<int8_t>& map_data, unsigned int map_width, unsigned int map_height, float map_resolution) {
    data = map_data;
    width = map_width;
    height = map_height;
    resolution = map_resolution;
}

bool OccupancyGrid::isOccupied(float x, float y) const {
        int ix = static_cast<int>(x / resolution);
        int iy = static_cast<int>(y / resolution);

        if (ix < 0 || iy < 0 || ix >= width || iy >= height)
            return false;
        
        int index = iy * width + ix;
        return data[index] > 50;
    }

float OccupancyGrid::getResolution() const {
    return resolution;
}

float OccupancyGrid::getWidth() const {
    return width;
}

float OccupancyGrid::getHeight() const {
    return height;
}