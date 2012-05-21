#include <sbpl_geometry_utils/Triangle.h>

namespace sbpl
{

Triangle::Triangle() :
    p1(),
    p2(),
    p3()
{
}

Triangle::Triangle(const Point3D& p1, const Point3D& p2, const Point3D& p3) :
    p1(p1),
    p2(p2),
    p3(p3)
{
}

Triangle::~Triangle()
{
}

}
