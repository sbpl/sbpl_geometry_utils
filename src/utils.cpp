#include <cassert>
#include <cmath>
#include <algorithm>
#include <sbpl_geometry_utils/utils.h>

namespace sbpl
{
    namespace utils
    {

double NormalizeAngle(double angle_rad, double angle_min_rad, double angle_max_rad)
{
    if (fabs(angle_rad) > 2.0 * M_PI) { // normalize to [-2*pi, 2*pi] range
        angle_rad -= ((int)(angle_rad / (2.0 * M_PI))) * 2.0 * M_PI;
    }

    while (angle_rad > angle_max_rad) {
        angle_rad -= 2.0 * M_PI;
    }

    while (angle_rad < angle_min_rad) {
        angle_rad += 2 * M_PI;
    }

    return angle_rad;
}

bool NormalizeAnglesIntoRange(std::vector<double>& angles,
                              const std::vector<double>& min_limits,
                              const std::vector<double>& max_limits)
{
    unsigned dim = angles.size();
    if (min_limits.size() != dim || max_limits.size() != dim) {
        return false;
    }

    for (unsigned i = 0; i < dim; i++) {
        if (min_limits[i] > max_limits[i]) {
            return false;
        }
    }

    for (unsigned i = 0; i < dim; i++) {
        double min_angle_norm = utils::NormalizeAngle(min_limits[i], 0.0, 2.0 * M_PI);
        angles[i] = utils::NormalizeAngle(angles[i], min_limits[i], min_angle_norm);
        if (angles[i] < min_limits[i] || angles[i] > max_limits[i]) {
            return false;
        }
    }

    return true;
}

bool AreJointsWithinLimits(const std::vector<double>& angles,
                           const std::vector<double>& min_limits,
                           const std::vector<double>& max_limits)
{
    assert(min_limits.size() == angles.size() && max_limits.size() == angles.size());

    for (int i = 0; i < (int)angles.size(); i++) {
        if (angles[i] < min_limits[i] || angles[i] > max_limits[i]) {
            return false;
        }
    }
    return true;
}

double ShortestAngleDist(double a1_rad, double a2_rad)
{
    double a1_norm = NormalizeAngle(a1_rad, 0.0, 2.0 * M_PI);
    double a2_norm = NormalizeAngle(a2_rad, 0.0, 2.0 * M_PI);
    return std::min(fabs(a1_norm - a2_norm), 2.0 * M_PI - fabs(a2_norm - a1_norm));
}

double ShortestAngleDiff(double a1_rad, double a2_rad)
{
    double a1_norm = NormalizeAngle(a1_rad, 0.0, 2.0 * M_PI);
    double a2_norm = NormalizeAngle(a2_rad, 0.0, 2.0 * M_PI);

    double dist = ShortestAngleDist(a1_rad, a2_rad);
    if (ShortestAngleDist(a1_norm + dist, a2_norm) < ShortestAngleDist(a1_norm - dist, a2_norm)) {
        return -dist;
    }
    else {
        return dist;
    }
}

int Sign(double val)
{
    if (val >= 0.0) {
        return 1;
    }
    else {
        return -1;
    }
}

double ToDegrees(double angle_rad)
{
    return angle_rad * 180.0 / M_PI;
}

double ToRadians(double angle_deg)
{
    return angle_deg * M_PI / 180.0;
}

    } // end namespace utils
} // end namespace sbpl
