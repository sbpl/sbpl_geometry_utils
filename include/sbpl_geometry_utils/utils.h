#ifndef SBPL_UTILS_UTILS_H
#define SBPL_UTILS_UTILS_H

namespace sbpl
{
    namespace utils
    {

/// @brief Normalize an angle into the range [angle_min, angle_max].
///
/// Assumes that the difference between \angle_max and \angle_min is 2*pi such
/// as the ranges [-pi, pi] and [0, 2*pi].
double NormalizeAngle(double angle_rad, double angle_min_rad, double angle_max_rad);

/// @brief Return the shortest distance between two angles, in radians.
double ShortestAngleDist(double a1_rad, double a2_rad);

/// @brief Return the shortest signed difference between two angles, in radians. The returned value
///        is positive if to follow along the shortest angular path from a2 to a1, you have to move
///        counter-clockwise.
double ShortestAngleDiff(double a1_rad, double a2_rad);

int Sign(double val);

double ToDegrees(double angle_rad);
double ToRadians(double angle_deg);

    } // end namespace utils
} // end namespace sbpl

#endif
