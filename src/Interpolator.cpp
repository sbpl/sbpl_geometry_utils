#include <sbpl_geometry_utils/Interpolator.h>
#include <iostream>
#include <cstdlib>
#include <cassert>
#include <cmath>

namespace sbpl
{

Interpolator::~Interpolator()
{

}

bool Interpolator::interpolatePath(const std::vector<double>& start,
                                   const std::vector<double>& end,
                                   const std::vector<double>& min_limits,
                                   const std::vector<double>& max_limits,
                                   const std::vector<double>& inc,
                                   std::vector<std::vector<double> >& path)
{
    // make copies so i can normalize things
    std::vector<double> startCopy = start;
    std::vector<double> endCopy = end;

    assert(startCopy.size() <= endCopy.size());
    assert(startCopy.size() <= min_limits.size());
    assert(startCopy.size() <= max_limits.size());
    assert(startCopy.size() <= inc.size());

    unsigned dim = startCopy.size();

    if (!normalizeAnglesIntoRange(startCopy, min_limits, max_limits)) {
        path.clear();
        return false;
    }

    if (!normalizeAnglesIntoRange(endCopy, min_limits, max_limits)) {
        path.clear();
        return false;
    }

    // check for valid end points
    for (unsigned i = 0; i < dim; i++) {
        if (startCopy[i] < min_limits[i] || startCopy[i] > max_limits[i] ||
            endCopy[i] < min_limits[i] || endCopy[i] > max_limits[i])
        {
            path.clear();
            return false;
        }
    }

    // will be initialized to 1 for counter-clockwise and -1 for clockwise
    std::vector<int> travelDirs(dim, 0);
    for (unsigned i = 0; i < dim; i++) {
        double anglediff = angleDiff(startCopy[i], endCopy[i]);
        double endPos = startCopy[i] + anglediff;
        if ((endPos > max_limits[i] || endPos < min_limits[i]) &&
            // aweful way of saying there are no joint limits here
            fabs(angleDiff(max_limits[i], min_limits[i])) > inc[i])
        {
            travelDirs[i] = anglediff >= 0 ? -1 : 1;
        }
        else {
            travelDirs[i] = anglediff >= 0 ? 1 : -1;
        }
    }

    std::vector<bool> doneAngles(dim, false);
    std::vector<double> currCfg = startCopy;
    path.push_back(currCfg);

    bool done = false;
    while (!done) {
        // inch all the joint angles towards the end
        for (unsigned i = 0; i < dim; i++) {
            // don't add anything if this joint has already reached the end
            if (doneAngles[i]) {
                continue;
            }

            double anglediff = angleDiff(currCfg[i], endCopy[i]);

            // add the last little bit to reach the end for this joint angle
            if (fabs(anglediff) < inc[i]) {
                currCfg[i] += anglediff;
            }
            // add increment in the direction of the shortest legal angle
            else {
                currCfg[i] += travelDirs[i] * inc[i];
            }

            // wrapping into the [min_limits[i], max_limits[i]] range. This is necessary for instance
            // if you have joint limits such as [-pi, pi] which are meant to represent no joint limits
            // at all
            if (currCfg[i] > max_limits[i]) {
                while (currCfg[i] > max_limits[i]) currCfg[i] -= 2 * M_PI;
            }
            if (currCfg[i] < min_limits[i]) {
                while (currCfg[i] < min_limits[i]) currCfg[i] += 2 * M_PI;
            }
        }

        // add this waypoint to the path
        path.push_back(currCfg);

        // mark angles done if they've reached the end; check for done overall
        done = true;
        for (unsigned i = 0; i < dim; i++) {
            const double epsilon = 1.0e-6;
            doneAngles[i] = (fabs(endCopy[i] - currCfg[i]) < epsilon);
            done &= doneAngles[i];
        }
    }

    return true;
}

Interpolator::Interpolator()
{

}

double Interpolator::angleDiff(double angle1, double angle2)
{
    // translate angles into [-M_PI, M_PI] range
    while (angle1 > M_PI) {
        angle1 -= 2 * M_PI;
    }
    while (angle1 < -M_PI) {
        angle1 += 2 * M_PI;
    }
    while (angle2 > M_PI) {
        angle2 -= 2 * M_PI;
    }
    while (angle2 < -M_PI) {
        angle2 += 2 * M_PI;
    }

    double angleDist = angle2 - angle1;
    if (fabs(angleDist) <= M_PI) {
        return angleDist;
    }
    else {
        if (angleDist < 0) {
            return 2 * M_PI - angleDist;
        }
        else {
            return angleDist - 2 * M_PI;
        }
    }
}

bool Interpolator::normalizeAnglesIntoRange(std::vector<double>& angles,
                                            const std::vector<double>& minLimits,
                                            const std::vector<double>& maxLimits)
{
    unsigned dim = angles.size();
    std::vector<double> anglesCopy = angles;

    // normalize the startCopy configuration to be in the range [min_limit[angle], max_limit[angle]]
    for (unsigned i = 0; i < dim; i++) {
        // first normalize anglesCopy[i] into the -2*pi, 2*pi range
        if (fabs(anglesCopy[i]) > 2.0 * M_PI) {
            anglesCopy[i] -= ((int)(anglesCopy[i] / (2.0 * M_PI))) * 2.0 * M_PI;
        }

        if (anglesCopy[i] > maxLimits[i]) {
            while (anglesCopy[i] > maxLimits[i]) {
                anglesCopy[i] -= 2 * M_PI;
            }
            if (anglesCopy[i] < minLimits[i]) {
                return false;
            }
        }
        if (anglesCopy[i] < minLimits[i]) {
            while (anglesCopy[i] < minLimits[i]) {
                anglesCopy[i] += 2 * M_PI;
            }
            if (anglesCopy[i] > maxLimits[i]) {
                return false;
            }
        }
    }

    angles = anglesCopy;
    return true;
}

}
