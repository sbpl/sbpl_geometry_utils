#ifndef SBPL_GEOMETRY_UTILS_PATH_SIMILARITY_MEASURER_H
#define SBPL_GEOMETRY_UTILS_PATH_SIMILARITY_MEASURER_H

#include <iostream>
#include <vector>
#include <geometry_msgs/Point.h>

namespace sbpl
{

class PathSimilarityMeasurer
{
public:
    typedef std::vector<geometry_msgs::Point> Trajectory;

    ~PathSimilarityMeasurer();

    /**
     * @brief Computes the similarity of a set of paths as the sum of variances between sets of corresponding waypoints
     *        along a set of given paths
     *
     * @param[in] trajectories A non-empty set of non-NULL pointers to trajectories to compare for similarity
     * @param[in] numWaypoints The number of corresponding waypoints along each path that are used for
     *            testing variance; must be at least greater than or equal to 2 (for the two endpoints)
     * @return Value representing the similarity between paths; larger numbers represent more dissimilar paths; -1.0
     *         represents an input error; equal to the sum of the variances of all the corresponding
     *         waypoints along all given paths
     */
    static double measure(const std::vector<const Trajectory*>& trajectories, int numWaypoints);

    /**
     * @brief Computes the similarity of a set of paths using dynamic time warping on waypoints along the paths
     *
     * This algorithm differs from measure() in that correspondance between waypoints is determined by their distance
     * from one another rather than just their index in the waypoint list. This is to solve the issue where paths are
     * very similar for the most part and their dissimilarity is concentrated around a certain region.
     *
     * @param[in] trajectories A non-empty set of non-NULL pointers to trajectories to compare for similarity
     * @param[in] numWaypoints The number of corresponding wayoints along each path that are used for testing variance;
     *                         must be at least greater than or equal to 2 (for the two endpoints)
     * @return Value representing the similarity between paths; larger numbers represent more dissimilar paths; -1.0
     *         represents an input error
     */
    static double measureDTW(const std::vector<const Trajectory*>& trajectories, int numWaypoints);

private:
    PathSimilarityMeasurer();

    static double euclidDist(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    static double distSqrd(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    static double calcPathLength(const Trajectory& traj);
    static bool generateNewWaypoints(const Trajectory& traj, double pathLength,
                                     int numWaypoints, Trajectory& newTraj);
    static double compareDTW(const Trajectory& traj1, const Trajectory& traj2);
};

std::ostream& operator<<(std::ostream& o, const geometry_msgs::Point& p);

}

#endif
