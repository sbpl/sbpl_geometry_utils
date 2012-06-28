#include <iostream>
#include <sbpl_geometry_utils/PathSimilarityMeasurer.h>
#include <geometry_msgs/Point.h>

void constructSimilarPaths(std::vector<geometry_msgs::Point>* const path1,
                           std::vector<geometry_msgs::Point>* const path2)
{
    path1->clear();
    path2->clear();

    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 1.0;
    p.y = 1.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 2.0;
    p.y = 0.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 3.0;
    p.y = -1.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 4.0;
    p.y = 0.0;
    p.z = 0.0;
    path1->push_back(p);

    // construct second path
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    path2->push_back(p);

    p.x = 1.5;
    p.y = 1.0;
    p.z = 0.0;
    path2->push_back(p);

    p.x = 2.0;
    p.y = 0.0;
    p.z = 0.0;
    path2->push_back(p);

    p.x = 2.5;
    p.y = -1.0;
    p.z = 0.0;
    path2->push_back(p);

    p.x = 4.0;
    p.y = 0.0;
    p.z = 0.0;
    path2->push_back(p);
}

void constructMostlySimilarPaths(std::vector<geometry_msgs::Point>* const path1,
                                 std::vector<geometry_msgs::Point>* const path2)
{
    path1->clear();
    path2->clear();

    geometry_msgs::Point p;

    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 0.142857142857;
    p.y = 1.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 0.285714285714;
    p.y = -1.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 0.428571428571;
    p.y = 1.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 0.571428571429;
    p.y = -1.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 0.714285714286;
    p.y = 1.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 0.857142857143;
    p.y = -1.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 2.0;
    p.y = 0.0;
    p.z = 0.0;
    path1->push_back(p);

    p.x = 10.0;
    p.y = 0.0;
    p.z = 0.0;
    path1->push_back(p);


    // Construct the second "simple" path
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    path2->push_back(p);

    p.x = 10.0;
    p.y = 0.0;
    p.z = 0.0;
    path2->push_back(p);
}

void constructSetOfPaths(std::vector<geometry_msgs::Point>* const path1,
                         std::vector<geometry_msgs::Point>* const path2,
                         std::vector<geometry_msgs::Point>* const path3)
{
    path1->clear();
    path2->clear();
    path3->clear();

    // reuse the first two paths
    constructSimilarPaths(path1, path2);

    geometry_msgs::Point p;

    p.x = 1.0;
    p.y = 0.0;
    p.z = 0.0;
    path3->push_back(p);

    p.x = 0.4;
    p.y = 1.0;
    p.z = 0.0;
    path3->push_back(p);

    p.x = 0.8;
    p.y = -1.0;
    p.z = 0.0;
    path3->push_back(p);

    p.x = 1.2;
    p.y = 1.0;
    p.z = 0.0;
    path3->push_back(p);

    p.x = 1.6;
    p.y = -1.0;
    p.z = 0.0;
    path3->push_back(p);

    p.x = 2.0;
    p.y = 0.0;
    p.z = 0.0;
    path3->push_back(p);

    p.x = 3.0;
    p.y = -1.0;
    p.z = 0.0;
    path3->push_back(p);

    p.x = 4.0;
    p.y = 0.0;
    p.z = 0.0;
    path3->push_back(p);

//    p.x = 0.0;
//    p.y = 0.0;
//    p.z = 0.0;
//    path3->push_back(p);
//    p.x = 0.25;
//    p.y = -0.5;
//    p.z = 0.0;
//    path3->push_back(p);
//    p.x = 0.50;
//    p.y = 0.50;
//    p.z = 0.0;
//    path3->push_back(p);
//    p.x = 0.75;
//    p.y = -0.5;
//    p.z = 0.0;
//    path3->push_back(p);
//    p.x = 1.0;
//    p.y = 1.0;
//    p.z = 0.0;
//    path3->push_back(p);
//    p.x = 3.0;
//    p.y = -1.0;
//    p.z = 0.0;
//    path3->push_back(p);
//    p.x = 4.0;
//    p.y = 0.0;
//    p.z = 0.0;
//    path3->push_back(p);
}

int main(int argc, char** argv)
{
    using namespace std;
    using namespace sbpl;

    geometry_msgs::Point p;

    std::vector<geometry_msgs::Point> firstPath;
    std::vector<geometry_msgs::Point> secondPath;

    constructSimilarPaths(&firstPath, &secondPath);

    std::vector<const PathSimilarityMeasurer::Trajectory*> trajectories;
    trajectories.push_back(&firstPath);
    trajectories.push_back(&secondPath);

    int numWaypoints = 7;
    double sim = PathSimilarityMeasurer::measure(trajectories, numWaypoints);
    double dtwSim = PathSimilarityMeasurer::measureDTW(trajectories, numWaypoints);
    cout << "Similarity is " << sim << " while DTW Similarity is " << dtwSim << endl;

    // Compare a set of paths that are mostly exactly the same but one of them
    // has some intense wobbling at the start
    constructMostlySimilarPaths(&firstPath, &secondPath);
    numWaypoints = 25;

    sim = PathSimilarityMeasurer::measure(trajectories, numWaypoints);
    dtwSim = PathSimilarityMeasurer::measureDTW(trajectories, numWaypoints);
    cout << "Similarity is " << sim << " while DTW Similarity is " << dtwSim << endl;

    std::vector<geometry_msgs::Point> thirdPath;
    // compare a set of three trajectories simultaneously (use the first two similar paths plus one that's almost
    // the same as the first path but messed up at the beginning
    constructSetOfPaths(&firstPath, &secondPath, &thirdPath);
    trajectories.push_back(&thirdPath);
    numWaypoints = 50;
    sim = PathSimilarityMeasurer::measure(trajectories, numWaypoints);
    dtwSim = PathSimilarityMeasurer::measureDTW(trajectories, numWaypoints);
    cout << "Similarity is " << sim << " while DTW Similarity is " << dtwSim << endl;

    return 0;
}
