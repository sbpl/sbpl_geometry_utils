#include <sbpl_geometry_utils/Triangle.h>

namespace sbpl
{

Triangle::Triangle() :
    p1(),
    p2(),
    p3()
{
}

Triangle::Triangle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3) :
    p1(p1),
    p2(p2),
    p3(p3)
{
}

Triangle::~Triangle()
{
}

}
