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

#ifndef sbpl_shortcut_h
#define sbpl_shortcut_h

#include <functional>
#include <vector>

namespace sbpl
{

namespace shortcut
{

/// @brief Convenience class to specify the requirements for a path generator passed to ShortcutPath
template <typename PointType, typename CostType, typename PathContainerType = std::vector<PointType>>
struct PathGenerator
{
    typedef PointType Point;
    typedef CostType Cost;
    typedef PathContainerType PathContainer;

    virtual ~PathGenerator() { }

    virtual bool generate_path(const PointType& start,
                               const PointType& end,
                               PathContainerType& path_out,
                               CostType& costs_out) const = 0;
};

/// @tparam PathContainer An stl-style container of path points where PathContainer::value_type is the point type
/// @tparam CostsContainer An stl-style container of costs where CostsContainer::value_type is the cost type
/// @tparam PathGeneratorsContainer An stl-style container of path generators where PathGeneratorsContainer::value_type is the path generator type that implements the PathGenerator interface
/// @tparam ShortcutPathContainer An stl-style container where ShortcutPathContainer::value_type should be the same as for PathContainer
/// @tparam CostCompare Comparison function to compare types of CostsContainer::value_type
/// @param window Unimplemented
/// @param granularity The number of points down the path to move before attempting another shortcut
template <typename PathContainer,
          typename CostsContainer,
          typename PathGeneratorsContainer,
          typename ShortcutPathContainer,
          typename CostCompare = std::less_equal<typename CostsContainer::value_type>>
bool ShortcutPath(const PathContainer& orig_path,
                  const CostsContainer& orig_path_costs,
                  const PathGeneratorsContainer& path_generators,
                  ShortcutPathContainer& shortcut_path,
                  unsigned window = 1,
                  unsigned granularity = 1,
                  const CostCompare& leq = CostCompare());

template <typename PathContainer,
          typename CostsContainer,
          typename PathGeneratorsContainer,
          typename ShortcutPathContainer,
          typename CostCompare = std::less_equal<typename CostsContainer::value_type>>
bool DivideAndConquerShortcutPath(const PathContainer& orig_path,
                                  const CostsContainer& orig_path_costs,
                                  const PathGeneratorsContainer& path_generators,
                                  ShortcutPathContainer& shortcut_path,
                                  const CostCompare& leq = CostCompare());


} // namespace shortcut

} // namespace sbpl

#include <sbpl_geometry_utils/shortcut-inl.h>

#endif

