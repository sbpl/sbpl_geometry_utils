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

#ifndef SBPL_GEOMETRY_UTILS_ANGLES_H
#define SBPL_GEOMETRY_UTILS_ANGLES_H

#include <vector>

namespace sbpl {
namespace angles {

/// \name Angles API
///@{

/// \brief Convert an angle specified in radians to degrees
constexpr double ToDegrees(double a_rad);

/// \brief Convert an angle specified in degrees to radians
constexpr double ToRadians(double a_deg);

/// \brief Normalize an angle into the range [-pi, pi]
double NormalizeAngle(double a);

/// \brief Normalize an angle into the range [0, 2*pi]
double NormalizeAnglePositive(double a);

/// \brief Normalize an angle into the range [angle_min, angle_max].
///
/// If the range [a_min, a_max] does not span 2*pi, the unwound angle equivalent
/// after a_min is returned.
double NormalizeAngle(double a, double a_min, double a_max);

/// \brief Return the shortest signed difference between two angles
///
/// The returned value lies within the range [-pi, pi]. Adding the result to ai
/// will produce an angle equivalent to af.
double ShortestAngleDiff(double af, double ai);

/// \brief Return the shortest distance between two angles
///
/// The returned value lies within the range [0, 2*pi]
double ShortestAngleDist(double af, double ai);

double MinorArcDiff(double af, double ai);
double MajorArcDiff(double af, double ai);
double MinorArcDist(double af, double ai);
double MajorArcDist(double af, double ai);

/// \brief Return the closest angle equivalent to af that is numerically greater
///        than ai
double Unwind(double ai, double af);

///@}

/// \name Limits API
///@{

/// \brief Normalize an angle vector with given joint limits
///
/// \param[in,out] angles The unnormalized vector of joint angles
/// \param[in] min_limits The corresponding minimum angle limits
/// \param[in] max_limits The corresponding maximum angle limits
/// \return Whether the normalized joint angles lie within the bounds specified by min_limits and max_limits; also
///         returns false if the sizes of any of the input vectors differ or if the i'th element of min_limits is
///         greater than the i'th element of max_limits
bool NormalizeAnglesIntoRange(
    std::vector<double>& angles,
    const std::vector<double>& min_limits,
    const std::vector<double>& max_limits);

bool IsJointWithinLimits(double angle, double min_angle, double max_angle);

/// \brief Return whether or not all joints are within their [min, max] limits.
///        All input vectors must have the same length.
/// \param[in] angles The vector of joint angles
/// \param[in] min_limits The corresponding minimum joint angles
/// \param[in] max_limits The corresponding maximum joint angles
/// \return true if all joints are within their [min, max] limits; false otherwise
bool AreJointsWithinLimits(
    const std::vector<double>& angles,
    const std::vector<double>& min_limits,
    const std::vector<double>& max_limits);

/// \brief Return the shortest distance between two angles, considering joint
///        limits.
///
/// The length of the major arc is returned if traveling along the minor arc
/// would violate the given angle limits.
double ShortestAngleDistWithLimits(
    double a1_rad,
    double a2_rad,
    double min_angle,
    double max_angle);

double ShortestAngleDiffWithLimits(
    double a1_rad,
    double a2_rad,
    double min_angle,
    double max_angle);

///@}

} // namespace angles
} // namespace sbpl

#include "detail/angles.h"

#endif
