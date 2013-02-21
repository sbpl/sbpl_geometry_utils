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

    while (angle_rad >= angle_max_rad) {
        angle_rad -= 2.0 * M_PI;
    }

    while (angle_rad < angle_min_rad) {
        angle_rad += 2 * M_PI;
    }

    return angle_rad;
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
    if (fabs(NormalizeAngle(a1_norm + dist, 0.0, 2.0 * M_PI) - a2_norm) < 1e-4) {
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
