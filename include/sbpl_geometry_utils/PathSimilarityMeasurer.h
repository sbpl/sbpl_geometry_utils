//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2014, Andrew Dornbush
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

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
