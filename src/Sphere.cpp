#include <sbpl_geometry_utils/Sphere.h>

namespace sbpl
{

Sphere::Sphere() :
    center(),
    radius(0.0)
{
}

Sphere::Sphere(const Point3D& center, double radius) :
    center(center),
    radius(radius)
{
}

Sphere::~Sphere()
{
}

}
