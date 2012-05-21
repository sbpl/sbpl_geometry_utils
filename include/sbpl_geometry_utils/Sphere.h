#ifndef SBPL_SPHERE_H
#define SBPL_SPHERE_H

#include <geometry_msgs/Point.h>

namespace sbpl
{

class Sphere
{
public:
    geometry_msgs::Point center;
    double radius;

    Sphere();
    Sphere(const geometry_msgs::Point& center, double radius);
    ~Sphere();
};

}

#endif
