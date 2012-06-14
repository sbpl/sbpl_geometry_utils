#ifndef SBPL_GEOMETRY_UTILS_TRIANGLE_H
#define SBPL_GEOMETRY_UTILS_TRIANGLE_H

#include <geometry_msgs/Point.h>

namespace sbpl
{

class Triangle
{
public:
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    geometry_msgs::Point p3;

    Triangle();
    Triangle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3);
    ~Triangle();
};

}

#endif
