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

namespace sbpl
{
    namespace utils
    {


double NormalizeAngle(double angle_rad, double angle_min_rad, double angle_max_rad)
{
    if (fabs(angle_rad) > 2.0 * M_PI) { // normalize to [-2*pi, 2*pi] range
        angle_rad = fmod(angle_rad, 2.0 * M_PI);
    }

    while (angle_rad > angle_max_rad) {
        angle_rad -= 2.0 * M_PI;
    }

    while (angle_rad < angle_min_rad) {
        angle_rad += 2.0 * M_PI;
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

bool IsJointWithinLimits(double angle, double min_angle, double max_angle)
{
    return !(angle < min_angle || angle > max_angle);
}

bool AreJointsWithinLimits(const std::vector<double>& angles,
                           const std::vector<double>& min_limits,
                           const std::vector<double>& max_limits)
{
    assert(min_limits.size() == angles.size() && max_limits.size() == angles.size());

    for (int i = 0; i < (int)angles.size(); i++) {
        if (!IsJointWithinLimits(angles[i], min_limits[i], max_limits[i])) {
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

double ShortestAngleDistWithLimits(double a1_rad, double a2_rad, double min_angle, double max_angle)
{
    // normalize angles and check that they fall within [min_angle, max_angle]
    double a1_norm = NormalizeAngle(a1_rad, min_angle, max_angle);
    double a2_norm = NormalizeAngle(a2_rad, min_angle, max_angle);
    if (!AreJointsWithinLimits({ a1_norm, a2_norm }, { min_angle, min_angle }, { max_angle, max_angle })) {
        return -1.0;
    }
    double angle_diff = ShortestAngleDiff(a1_norm, a2_norm);
    if (a2_norm + angle_diff > max_angle || a2_norm + angle_diff < min_angle) {
        return 2.0 * M_PI - fabs(angle_diff);
    }
    else {
        return fabs(angle_diff);
    }
}

double ShortestAngleDiffWithLimits(double a1_rad, double a2_rad, double min_angle, double max_angle)
{
    // normalize angles and check that they fall within [min_angle, max_angle]

    double a1_norm = NormalizeAngle(a1_rad, min_angle, max_angle);
    double a2_norm = NormalizeAngle(a2_rad, min_angle, max_angle);

    if (!IsJointWithinLimits(a1_norm, min_angle, max_angle) || !IsJointWithinLimits(a2_norm, min_angle, max_angle)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double angle_diff = ShortestAngleDiff(a1_norm, a2_norm);
    if (a2_norm + angle_diff > max_angle || a2_norm + angle_diff < min_angle) {
        return -Signd(angle_diff) * (2.0 * M_PI - fabs(angle_diff));
    }
    else {
        return angle_diff;
    }
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

template <typename T>
T SignT(T val)
{
    if (val == 0) {
        return 0;
    }
    else if (val > 0) {
        return T(1);
    }
    else {
        return T(-1);
    }
}

int Sign(int val)
{
    return SignT(val);
}

double Signd(double val)
{
    return SignT(val);
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
