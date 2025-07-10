#include "CollisionChecking.h"
#include <cmath>

///////////////////////////////////////
// RBE 550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////


// TODO: Copy your implementation from previous projects

bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    for(auto i: obstacles)
    {
        if(x>=(i.x) && x<=(i.x + i.width) && y>=(i.y) && y<=(i.y + i.height))
        {
            return false;
        }
    }

    return true;
}

bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    for (const auto &rect : obstacles)
    {
        // Find the nearest point on the rectangle to the circle's center
        double nearestX = std::max(rect.x, std::min(x, rect.x + rect.width));
        double nearestY = std::max(rect.y, std::min(y, rect.y + rect.height));
        
        // Calculate the distance from the circle's center to this nearest point
        double dx = x - nearestX;
        double dy = y - nearestY;
        double distanceSquared = dx * dx + dy * dy;

        // If this distance is less than or equal to the radius squared, there's an intersection
        if (distanceSquared <= radius * radius)
        {
            return false;
        }
    }

    // If no intersections were found, the circle is valid
    return true;
}

bool doLinesIntersect(double x1, double y1, double x2, double y2,
                      double x3, double y3, double x4, double y4) {
    auto orientation = [](double x1, double y1, double x2, double y2, double x3, double y3) {
        double val = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
        if (val == 0.0) return 0; // Collinear
        return (val > 0.0) ? 1 : -1; // Clockwise or Counterclockwise
    };

    int o1 = orientation(x1, y1, x2, y2, x3, y3);
    int o2 = orientation(x1, y1, x2, y2, x4, y4);
    int o3 = orientation(x3, y3, x4, y4, x1, y1);
    int o4 = orientation(x3, y3, x4, y4, x2, y2);

    // General case
    if (o1 != o2 && o3 != o4) return true;

    
    return false;
}

bool isValidSquare(double x, double y, double theta, double sideLength, 
                   const std::vector<Rectangle>& obstacles) {
    std::vector<std::pair<double, double>> squareCorners(4);

    double halfSide = sideLength / 2.0;

    // Corners of the square before rotation, relative to (x, y)
    std::vector<std::pair<double, double>> unrotatedCorners = {
        {x - halfSide, y - halfSide},  // Bottom-left corner
        {x + halfSide, y - halfSide},  // Bottom-right corner
        {x + halfSide, y + halfSide},  // Top-right corner
        {x - halfSide, y + halfSide}   // Top-left corner
    };

    // Apply rotation to each corner using the 2D rotation matrix
    for (int i = 0; i < 4; ++i) {
        double x0 = unrotatedCorners[i].first;
        double y0 = unrotatedCorners[i].second;

        squareCorners[i].first = x + (x0 - x) * cos(theta) - (y0 - y) * sin(theta);
        squareCorners[i].second = y + (x0 - x) * sin(theta) + (y0 - y) * cos(theta);
    }
    double hbound=14.0;
    double lbound=0.0;

    // Check if any corner of the square goes out of the environment bounds
    for (const auto& corner : squareCorners) {
        if (corner.first < lbound || corner.first > hbound || 
            corner.second < lbound || corner.second > hbound) {
            return false; // Corner is out of bounds
        }
    }


    // Check if any corner of the square collides with any rectangle obstacle



    for (const Rectangle& rect : obstacles) {
        for (const auto& corner : squareCorners) {
            if (corner.first >= rect.x && corner.first <= rect.x + rect.width &&
                corner.second >= rect.y && corner.second <= rect.y + rect.height) {
                return false; // Collision detected with an obstacle
            }
        }

        // Check if any edge of the square intersects any side of the obstacle
        for (int i = 0; i < 4; ++i) {
            int next = (i + 1) % 4; // Get the next corner to form an edge
            if (doLinesIntersect(squareCorners[i].first, squareCorners[i].second,
                                 squareCorners[next].first, squareCorners[next].second,
                                 rect.x, rect.y, rect.x, rect.y + rect.height) ||
                doLinesIntersect(squareCorners[i].first, squareCorners[i].second,
                                 squareCorners[next].first, squareCorners[next].second,
                                 rect.x, rect.y, rect.x + rect.width, rect.y) ||
                doLinesIntersect(squareCorners[i].first, squareCorners[i].second,
                                 squareCorners[next].first, squareCorners[next].second,
                                 rect.x + rect.width, rect.y, rect.x + rect.width, rect.y + rect.height) ||
                doLinesIntersect(squareCorners[i].first, squareCorners[i].second,
                                 squareCorners[next].first, squareCorners[next].second,
                                 rect.x, rect.y + rect.height, rect.x + rect.width, rect.y + rect.height)) {
                return false;  // Collision detected between square edge and obstacle edge
            }
        }
    }

    return true; 
}
