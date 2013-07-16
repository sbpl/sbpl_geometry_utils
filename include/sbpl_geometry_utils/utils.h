#ifndef SBPL_UTILS_UTILS_H
#define SBPL_UTILS_UTILS_H

#include <vector>

namespace sbpl
{
    namespace utils
    {

/// @brief Normalize an angle into the range [angle_min, angle_max].
///
/// Assumes that the difference between \angle_max and \angle_min is 2*pi such
/// as the ranges [-pi, pi] and [0, 2*pi].
double NormalizeAngle(double angle_rad, double angle_min_rad, double angle_max_rad);

/// @brief Attempt to normalize a joint angle vector with given joint limits.
/// @param[in,out] angles The unnormalized vector of joint angles
/// @param[in] min_limits The corresponding minimum angle limits
/// @param[in] max_limits The corresponding maximum angle limits
/// @return Whether the normalized joint angles lie within the bounds specified by min_limits and max_limits; also
///         returns false if the sizes of any of the input vectors differ or if the i'th element of min_limits is
///         greater than the i'th element of max_limits
bool NormalizeAnglesIntoRange(std::vector<double>& angles,
                              const std::vector<double>& min_limits,
                              const std::vector<double>& max_limits);

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
