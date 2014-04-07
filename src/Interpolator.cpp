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

#include <sbpl_geometry_utils/Interpolator.h>
#include <cstdlib>
#include <sbpl_geometry_utils/interpolation.h>

namespace sbpl
{

bool Interpolator::interpolatePath(const std::vector<double>& start,
                                   const std::vector<double>& end,
                                   const std::vector<double>& min_limits,
                                   const std::vector<double>& max_limits,
                                   const std::vector<double>& inc,
                                   std::vector<std::vector<double> >& path)
{
    return interp::InterpolatePath(start, end, min_limits, max_limits, inc, path);
}

bool Interpolator::interpolatePath(const std::vector<double>& start,
                                   const std::vector<double>& end,
                                   const std::vector<double>& min_limits,
                                   const std::vector<double>& max_limits,
                                   const std::vector<double>& inc,
                                   const std::vector<bool>& continuous_joints,
                                   std::vector<std::vector<double> >& path)
{
    return interp::InterpolatePath(start, end, min_limits, max_limits,
                                   inc, continuous_joints, path);
}

} // end namespace sbpl
