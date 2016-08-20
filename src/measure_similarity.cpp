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

#include <sbpl_geometry_utils/measure_similarity.h>
#include <cmath>
#include <cstdlib>
#include <algorithm>

namespace sbpl
{

namespace stats
{

namespace
{

/// Compute the distance between two points.
double distance(const geometry_msgs::Point& p, const geometry_msgs::Point& q);
double distance_sqrd(const geometry_msgs::Point& p, const geometry_msgs::Point& q);

/// Compute the total length of a path.
double compute_path_length(const Path& p);

/// Add or remove points from a path so that the resulting path has a given number of waypoints. Samples points are
/// determined by linear interpolation between given waypoints.
Path interpolate_path(const Path& p, int num_waypoints);

double distance(const geometry_msgs::Point& p, const geometry_msgs::Point& q)
{
    return sqrt(distance_sqrd(p, q));
}

double distance_sqrd(const geometry_msgs::Point& p, const geometry_msgs::Point& q)
{
    double dx = p.x - q.x;
    double dy = p.y - q.y;
    double dz = p.z - q.z;
    return dx * dx + dy * dy + dz * dz;
}

double compute_path_length(const Path& path)
{
    if (path.size() < 2) {
        return 0.0;
    }

    double path_length = 0.0;
    for (unsigned i = 1; i < path.size(); i++) {
        const geometry_msgs::Point& p = path[i - 1];
        const geometry_msgs::Point& q = path[i];
        path_length += distance(p, q);
    }
    return path_length;
}

geometry_msgs::Point interpolate(const geometry_msgs::Point& p, const geometry_msgs::Point& q, double alpha)
{
    geometry_msgs::Point out;
    out.x = (1.0 - alpha) * p.x + alpha * q.x;
    out.y = (1.0 - alpha) * p.y + alpha * q.y;
    out.z = (1.0 - alpha) * p.z + alpha * q.z;
    return out;
}

Path interpolate_path(const Path& path, int num_waypoints)
{
    Path ret;
    if (num_waypoints < 2) {
        return ret;
    }

    ret.reserve(num_waypoints);

    const double path_length = compute_path_length(path);
    double path_inc = path_length / (num_waypoints - 1);

    // keep the same start point
    ret.push_back(path.front());
    int num_placed_waypoints = 1;

    int passed_wp = 0;

    // generate the new waypoints
    while (num_placed_waypoints != num_waypoints) {
        // push back the last waypoint and return
        if (num_placed_waypoints == num_waypoints - 1) {
            ret.push_back(path.back());
            break;
        }

        double dist_between_actual_wps = distance(path[passed_wp], path[passed_wp + 1]);
        double dist_until_next_wp = distance(ret.back(), path[passed_wp + 1]);

        if (path_inc < dist_until_next_wp) {
            // find a spot between the last placed waypoint and the next original waypoint further along the path

            double alpha = path_inc / dist_until_next_wp;

            // calculate the point between our most latest waypoint and the next one we want
            // to generate
            geometry_msgs::Point p = interpolate(ret.back(), path[passed_wp + 1], alpha);

            ret.push_back(p);
            num_placed_waypoints++;
        }
        else {
            // need to find what waypoints we are now in interpolating between

            passed_wp++; // we passed another of the actual waypoints
            dist_between_actual_wps = distance(path[passed_wp], path[passed_wp + 1]);

            double new_dist = path_inc - dist_until_next_wp;

            // there may still be waypoints to pass
            while (new_dist > distance(path[passed_wp], path[passed_wp + 1])) {
                new_dist -= distance(path[passed_wp], path[passed_wp + 1]);
                passed_wp++;
                dist_between_actual_wps = distance(path[passed_wp], path[passed_wp + 1]);
            }

            double alpha = new_dist / dist_between_actual_wps;

            geometry_msgs::Point p = interpolate(path[passed_wp], path[passed_wp + 1], alpha);

            ret.push_back(p);
            num_placed_waypoints++;
        }
    }

    return ret;
}

} // empty namespace

ConstPathRange entire_path(const Path& p)
{
    return ConstPathRange(p.cbegin(), p.cend());
}

double measure_path_similarity(
    const std::vector<ConstPathRange>& paths,
    int num_waypoints)
{
    if (paths.empty()) {
        return 0.0;
    }

    if (num_waypoints < 2 || paths.size() < 2) {
        // deficient number of points/paths for comparison
        return -1.0;
    }

    // 1. reinterpolate all trajectories
    std::vector<Path> interp_trajectories;
    interp_trajectories.reserve(paths.size());
    for (const ConstPathRange& pends : paths) {
        // TODO: eliminate the need to create a complete copy of each path here.
        Path p;
        for (auto pit = pends.first; pit != pends.second; ++pit) {
            p.push_back(*pit);
        }
        interp_trajectories.push_back(interpolate_path(p, num_waypoints));
    }

    int num_comparisons = 0;
    double total_cost = 0.0;
    for (size_t i = 0; i < interp_trajectories.size(); ++i) {
        for (size_t j = i + 1; j < interp_trajectories.size(); ++j) {
            const Path& p1 = interp_trajectories[i];
            const Path& p2 = interp_trajectories[j];
            total_cost += dynamic_time_warping(p1.cbegin(), p1.cend(), p2.cbegin(), p2.cend(), distance);
            ++num_comparisons;
        }
    }

    return total_cost / num_comparisons;
}

} // namespace stats

double PathSimilarityMeasurer::measure(
    const std::vector<const Trajectory*>& trajectories,
    int numWaypoints)
{
    // check for invalid number of waypoints or empty list of trajectories
    if (numWaypoints < 2 || trajectories.size() == 0) {
        return -1.0;
    }

    // check to make sure all trajectories are non-NULL
    for (int i = 0; i < (int)trajectories.size(); i++) {
        if (trajectories[i] == NULL) {
            return -1.0;
        }
    }

    std::vector<double> pathLengths(trajectories.size(), 0);

    // calculate pathLengths
    for (unsigned i = 0; i < trajectories.size(); i++) {
        pathLengths[i] = calcPathLength(*(trajectories[i]));
    }

    // calculate a new trajectories representing the old trajectories but having the same amount of
    // waypoints each
    std::vector<Trajectory> newTrajs;
    newTrajs.reserve(trajectories.size());
    for (unsigned i = 0; i < trajectories.size(); i++) {
        newTrajs.push_back(Trajectory());
        generateNewWaypoints(*trajectories[i], pathLengths[i], numWaypoints, newTrajs[i]);
    }

    // make sure we actually have numWaypoints for each path
    for (int i = 0; i < (int)newTrajs.size(); i++) {
        assert((int)newTrajs[i].size() == numWaypoints);
    }

    Trajectory avgTrajectory;
    avgTrajectory.reserve(numWaypoints);

    // calculate the average trajectory for this set of trajectories
    for (int i = 0; i < numWaypoints; i++) {
        geometry_msgs::Point avg;
        avg.x = 0.0; avg.y = 0.0; avg.z = 0.0;

        for (int j = 0; j < (int)newTrajs.size(); j++) {
            avg.x += newTrajs[j][i].x;
            avg.y += newTrajs[j][i].y;
            avg.z += newTrajs[j][i].z;
        }

        avg.x /= newTrajs.size();
        avg.y /= newTrajs.size();
        avg.z /= newTrajs.size();
        avgTrajectory.push_back(avg);
    }

    std::vector<double> variances;
    variances.reserve(numWaypoints);
    for (int i = 0; i < numWaypoints; i++) {
        double variance = 0.0;
        for (int j = 0; j < (int)newTrajs.size(); j++) {
            variance += distSqrd(newTrajs[j][i], avgTrajectory[i]);
        }
        variances.push_back(variance);
    }

    double totalVariance = 0.0;
    for (int i = 0; i < numWaypoints; i++) {
        totalVariance += variances[i];
    }

    return totalVariance;
}

double PathSimilarityMeasurer::measureDTW(const std::vector<const Trajectory*>& trajectories, int numWaypoints)
{
    std::vector<sbpl::stats::ConstPathRange> path_ends;
    path_ends.reserve(trajectories.size());
    for (const Trajectory* traj : trajectories) {
        path_ends.push_back(sbpl::stats::entire_path(*traj));
    }
    return sbpl::stats::measure_path_similarity(path_ends, numWaypoints);
}

double PathSimilarityMeasurer::euclidDist(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
    return sbpl::stats::distance(p1, p2);
}

double PathSimilarityMeasurer::distSqrd(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
    return sbpl::stats::distance_sqrd(p1, p2);
}

double PathSimilarityMeasurer::calcPathLength(const Trajectory& traj)
{
    return sbpl::stats::compute_path_length(traj);
}

bool PathSimilarityMeasurer::generateNewWaypoints(
    const Trajectory& traj,
    double pathLength,
    int numWaypoints,
    Trajectory& newTraj)
{
    newTraj = sbpl::stats::interpolate_path(traj, numWaypoints);
    return true;
}

double PathSimilarityMeasurer::compareDTW(const Trajectory& traj1, const Trajectory& traj2)
{
    return sbpl::stats::dynamic_time_warping(
            traj1.cbegin(), traj1.cend(), traj2.cbegin(), traj2.cend(), sbpl::stats::distance);
}

double PathSimilarityMeasurer::weird_distance(const geometry_msgs::Point& p, const geometry_msgs::Point& q)
{
    geometry_msgs::Point averagePt;
    averagePt.x = (p.x + q.x) / 2.0;
    averagePt.y = (p.y + q.y) / 2.0;
    averagePt.z = (p.z + q.z) / 2.0;
    return distSqrd(p, averagePt) + distSqrd(q, averagePt);
}

std::ostream& operator<<(std::ostream& o, const geometry_msgs::Point& p)
{
    o << "{x: " << p.x << ",  y: " << p.y << ", z: " << p.z << "}";
    return o;
}

}
