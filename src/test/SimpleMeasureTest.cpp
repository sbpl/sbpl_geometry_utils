#include <iostream>
#include <sbpl_geometry_utils/PathSimilarityMeasurer.h>
#include <geometry_msgs/Point.h>

void constructFirstPath(std::vector<geometry_msgs::Point>& path)
{
    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    path.push_back(p);

    p.x = 1.0;
    p.y = 1.0;
    p.z = 0.0;
    path.push_back(p);

    p.x = 2.0;
    p.y = 0.0;
    p.z = 0.0;
    path.push_back(p);

    p.x = 3.0;
    p.y = -1.0;
    p.z = 0.0;
    path.push_back(p);

    p.x = 4.0;
    p.y = 0.0;
    p.z = 0.0;
    path.push_back(p);
}

void constructSecondPath(std::vector<geometry_msgs::Point>& path)
{
    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    path.push_back(p);

    p.x = 1.5;
    p.y = 1.0;
    p.z = 0.0;
    path.push_back(p);

    p.x = 2.0;
    p.y = 0.0;
    p.z = 0.0;
    path.push_back(p);

    p.x = 2.5;
    p.y = -1.0;
    p.z = 0.0;
    path.push_back(p);

    p.x = 4.0;
    p.y = 0.0;
    p.z = 0.0;
    path.push_back(p);
}

int main(int argc, char** argv)
{
    // *Really this is testing my ability to create vim macros*
    using namespace std;
    using namespace sbpl;

    geometry_msgs::Point p;

    std::vector<geometry_msgs::Point> firstPath;
    std::vector<geometry_msgs::Point> secondPath;

    constructFirstPath(firstPath);
    constructSecondPath(secondPath);

    std::vector<const PathSimilarityMeasurer::Trajectory*> trajectories;
    trajectories.push_back(&firstPath);
    trajectories.push_back(&secondPath);

    int numWaypoints = 7;
    double sim = PathSimilarityMeasurer::measure(trajectories, numWaypoints);

    cout << "Similarity is " << sim << endl;

    return 0;
}
