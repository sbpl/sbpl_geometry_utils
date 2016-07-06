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

#include <cassert>
#include <cmath>
#include <algorithm>
#include <sbpl_geometry_utils/utils.h>

namespace sbpl {
namespace utils {

template <typename T>
constexpr
T SignT(T val)
{
    return val == 0 ? 0 : (val > 0 ? T(1) : T(-1));
}

constexpr
int Sign(int val)
{
    return SignT(val);
}

constexpr
double Signd(double val)
{
    return SignT(val);
}

constexpr
double ToDegrees(double a_rad)
{
    return a_rad * 180.0 / M_PI;
}

constexpr
double ToRadians(double a_deg)
{
    return a_deg * M_PI / 180.0;
}

inline
double NormalizeAngle(double a)
{
    // normalize to [-2*pi, 2*pi] range
    if (std::fabs(a) > 2.0 * M_PI) {
        a = std::fmod(a, 2.0 * M_PI);
    }

    if (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    if (a > M_PI) {
        a -= 2.0 * M_PI;
    }

    return a;
}

inline
double NormalizeAnglePositive(double a)
{
    // normalize to [-2*pi, 2*pi] range
    if (std::fabs(a) > 2.0 * M_PI) {
        a = std::fmod(a, 2.0 * M_PI);
    }

    if (a < 0) {
        a += 2.0 * M_PI;
    }

    return a;
}

inline
double NormalizeAngle(double a, double a_min, double a_max)
{
    // normalize to [-2*pi, 2*pi] range
    if (std::fabs(a) > 2.0 * M_PI) {
        a = std::fmod(a, 2.0 * M_PI);
    }

    while (a > a_max) {
        a -= 2.0 * M_PI;
    }

    while (a < a_min) {
        a += 2.0 * M_PI;
    }

    return a;
}

inline
double ShortestAngleDiff(double af, double ai)
{
    return NormalizeAngle(af - ai);
}

inline
double ShortestAngleDist(double af, double ai)
{
    return std::fabs(ShortestAngleDiff(af, ai));
}

inline
double MinorArcDiff(double af, double ai)
{
    return ShortestAngleDiff(af, ai);
}

inline
double MajorArcDiff(double af, double ai)
{
    double diff = ShortestAngleDiff(af, ai);
    return -1.0 * std::copysign(1.0, diff) * (2.0 * M_PI - std::fabs(diff));
}

inline
double MinorArcDist(double af, double ai)
{
    return std::fabs(MinorArcDiff(af, ai));
}

inline
double MajorArcDist(double af, double ai)
{
    return std::fabs(MajorArcDiff(af, ai));
}

inline
double Unwind(double ai, double af)
{
    af = std::remainder(af - ai, 2.0 * M_PI); //2.0 * M_PI * std::floor((af - ai) / (2.0 * M_PI));
    if (af < ai) {
        af += 2.0 * M_PI;
    }
    return af;
}

inline
bool NormalizeAnglesIntoRange(
    std::vector<double>& angles,
    const std::vector<double>& angle_mins,
    const std::vector<double>& angle_maxs)
{
    size_t dim = angles.size();
    if (angle_mins.size() != dim || angle_maxs.size() != dim) {
        return false;
    }

    for (size_t i = 0; i < dim; i++) {
        if (angle_mins[i] > angle_maxs[i]) {
            return false;
        }
    }

    for (size_t i = 0; i < dim; i++) {
        double min_angle_norm = utils::NormalizeAngle(angle_mins[i]);
        angles[i] = utils::NormalizeAngle(angles[i], angle_mins[i], min_angle_norm);
        if (angles[i] < angle_mins[i] || angles[i] > angle_maxs[i]) {
            return false;
        }
    }

    return true;
}

inline
bool IsJointWithinLimits(double a, double a_min, double a_max)
{
    return !(a < a_min || a > a_max);
}

inline
bool AreJointsWithinLimits(
    const std::vector<double>& angles,
    const std::vector<double>& angle_mins,
    const std::vector<double>& angle_maxs)
{
    assert(angle_mins.size() == angles.size() && angle_maxs.size() == angles.size());

    for (int i = 0; i < (int)angles.size(); i++) {
        if (!IsJointWithinLimits(angles[i], angle_mins[i], angle_maxs[i])) {
            return false;
        }
    }
    return true;
}

inline
double ShortestAngleDistWithLimits(
    double af,
    double ai,
    double a_min,
    double a_max)
{
    // normalize angles and check that they fall within [a_min, a_max]
    double a1_norm = NormalizeAngle(af, a_min, a_max);
    double a2_norm = NormalizeAngle(ai, a_min, a_max);
    if (!AreJointsWithinLimits({ a1_norm, a2_norm }, { a_min, a_min }, { a_max, a_max })) {
        return -1.0;
    }
    double angle_diff = ShortestAngleDiff(a1_norm, a2_norm);
    if (a2_norm + angle_diff > a_max || a2_norm + angle_diff < a_min) {
        return 2.0 * M_PI - fabs(angle_diff);
    }
    else {
        return fabs(angle_diff);
    }
}

inline
double ShortestAngleDiffWithLimits(
    double af,
    double ai,
    double a_min,
    double a_max)
{
    // normalize angles and check that they fall within [a_min, a_max]

    double a1_norm = NormalizeAngle(af, a_min, a_max);
    double a2_norm = NormalizeAngle(ai, a_min, a_max);

    if (!IsJointWithinLimits(a1_norm, a_min, a_max) ||
        !IsJointWithinLimits(a2_norm, a_min, a_max))
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double angle_diff = ShortestAngleDiff(a1_norm, a2_norm);
    if (a2_norm + angle_diff > a_max || a2_norm + angle_diff < a_min) {
        return -Signd(angle_diff) * (2.0 * M_PI - fabs(angle_diff));
    }
    else {
        return angle_diff;
    }
}

} // end namespace utils
} // end namespace sbpl
