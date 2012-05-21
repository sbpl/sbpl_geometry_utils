#ifndef SBPL_TRIANGLE_H
#define SBPL_TRIANGLE_H

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
    Triangle(const Point3D& p1, const Point3D& p2, const Point3D& p3);
    ~Triangle();
};
}

#endif
