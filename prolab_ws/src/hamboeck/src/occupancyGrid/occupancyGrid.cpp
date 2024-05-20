#include "../../include/occupancyGrid/occupancyGrid.h"

void OccupancyGrid::setMap(const std::vector<int8_t>& map_data, unsigned int map_width, unsigned int map_height, float map_resolution) {
    data = map_data;
    width = map_width;
    height = map_height;
    resolution = map_resolution;
}

bool OccupancyGrid::isOccupied(float x, float y) const {
        // Convert world coordinates to grid coordinates
        int ix = static_cast<int>(x / resolution);
        int iy = static_cast<int>(y / resolution);

        // Ensure the grid coordinates are within map bounds
        if (ix < 0 || iy < 0 || ix >= width || iy >= height)
            return false;  // Out of bounds of the map is considered not occupied
        
        // Calculate the index in the flat data array
        int index = iy * width + ix;
        // Consider thresholds for occupancy; typical threshold might be 50%
        return data[index] > 50;  // Return true if the cell is considered occupied
    }

float OccupancyGrid::getResolution() const {
    return resolution;
}