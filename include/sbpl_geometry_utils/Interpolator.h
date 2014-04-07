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

#ifndef SBPL_GEOMETRY_UTILS_INTERPOLATOR_H
#define SBPL_GEOMETRY_UTILS_INTERPOLATOR_H

#include <vector>

namespace sbpl
{

class Interpolator
{
public:
    /**
     * @brief interpolates a set of joint angles by a given increment
     * @param[in] start the start joint configuration
     * @param[in] end the end joint configuration
     * @param[in] min_limits the minimum joint limits
     * @param[in] max_limits the maximum joint limits
     * @param[in] inc the value by which each joint is moved for each waypoint
     *                until the end configuration for that joint is reached
     * @param[out] path the interpolated path returned as a list of joint
     *                  configurations
     */
    static bool interpolatePath(const std::vector<double>& start,
                                const std::vector<double>& end,
                                const std::vector<double>& min_limits,
                                const std::vector<double>& max_limits,
                                const std::vector<double>& inc,
                                std::vector<std::vector<double> >& path);

    /**
     * @brief interpolates a set of joint angles by a given increment
     * @param[in] start the start joint configuration
     * @param[in] end the end joint configuration
     * @param[in] min_limits the minimum joint limits
     * @param[in] max_limits the maximum joint limits
     * @param[in] inc the value by which each joint is moved for each waypoint
     *                until the end configuration for that joint is reached
     * @param[out] path the interpolated path returned as a list of joint
     *                  configurations
     */
    static bool interpolatePath(const std::vector<double>& start,
                                const std::vector<double>& end,
                                const std::vector<double>& min_limits,
                                const std::vector<double>& max_limits,
                                const std::vector<double>& inc,
                                const std::vector<bool>& continuous_joints,
                                std::vector<std::vector<double> >& path);
private:
    Interpolator();
};

}

#endif
