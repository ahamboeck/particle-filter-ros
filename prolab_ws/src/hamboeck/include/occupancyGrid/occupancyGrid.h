#pragma once

#include <cstdint>
#include <vector>

class OccupancyGrid {
private:
    std::vector<int8_t> data;  // Stores the map data
    unsigned int width;        // Width of the map
    unsigned int height;       // Height of the map
    float resolution;          // The map resolution in meters/cell

public:
    OccupancyGrid() : width(0), height(0), resolution(0.0f) {}

    void setMap(const std::vector<int8_t>& map_data, unsigned int map_width, unsigned int map_height, float map_resolution);

    bool isOccupied(float x, float y) const;

    // Accessor for resolution
    float getResolution() const;
    float getWidth() const;
    float getHeight() const;
};