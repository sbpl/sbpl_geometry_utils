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

namespace sbpl {
namespace shortcut {

/// \brief Convenience class for specifying path generator requirements
template <
    typename PointType,
    typename CostType,
    typename PathContainerType = std::vector<PointType>>
struct PathGenerator
{
    typedef PointType Point;
    typedef CostType Cost;
    typedef PathContainerType PathContainer;

    virtual ~PathGenerator() { }

    virtual bool generate_path(
        const PointType& start,
        const PointType& end,
        PathContainerType& path_out,
        CostType& costs_out) const = 0;
};

/// \brief Apply iterative path shortcutting to an container of path elements.
///
/// The routine works by iteratively moving further and further down the path,
/// attempting to replace the original path with shortcut paths provided by path
/// generators, until an invalid shortcut path is produced or the end of the
/// path is reached.
///
/// \tparam PathContainer An stl-style sequence of path points
///
/// \tparam CostsContainer An stl-style sequence of costs
///
/// \tparam PathGeneratorsContainer An stl-style sequence of path generators.
///     Each element of the sequence must implement the PathGenerator interface
///
/// \tparam ShortcutPathContainer An stl-style sequence container to store the
///     resulting path. This should contain elements of the same type as
///     PathContainer
///
/// \tparam CostCompare Comparison function to compare costs
///
/// \param window Unimplemented
///
/// \param granularity The number of points down the path to move before
///     attempting another shortcut
///
/// \return true if inputs are valid, i.e., either the input path is empty or
///     there are N points with N - 1 costs; false otherwise
template <
    typename PathContainer,
    typename CostsContainer,
    typename PathGeneratorsContainer,
    typename ShortcutPathContainer,
    typename CostCompare = std::less_equal<typename CostsContainer::value_type>>
bool ShortcutPath(
    const PathContainer&            points,
    const CostsContainer&           costs,
    const PathGeneratorsContainer&  generators,
    ShortcutPathContainer&          shortcut_points,
    size_t                          window = 1,
    size_t                          granularity = 1,
    const CostCompare&              leq = CostCompare());

/// \brief Apply iterative path shortcutting to a range of path elements.
///
/// The routine works by iteratively moving further and further down the path,
/// attempting to replace the original path with shortcut paths provided by path
/// generators, until an invalid shortcut path is produced or the end of the
/// path is reached.
///
/// \param pfirst Input iterator pointing to the beginning of the range of points
///
/// \param plast  Input iterator pointing to the end of the range of points
///
/// \param cfirst Input iterator pointing to the beginning of the range of costs
///
/// \param clast  Input iterator pointing to the end of the range of costs
///
/// \param ofirst Output iterator to store the resulting path
///
/// \param window Unimplemented
///
/// \param granularity The number of points to advance in the path before
///     attempting a shortcut
///
/// \param leq Comparison object for comparing costs
///
/// \return true if inputs are valid, i.e., either the input path range has
///     zero length or the input path range has length N and the input cost range
///     has length N - 1.
template <
    typename InputPathIt,
    typename InputCostIt,
    typename GeneratorIt,
    typename OutputPathIt,
    typename CostCompare = std::less_equal<
            typename std::iterator_traits<InputCostIt>::value_type>>
bool ShortcutPath(
    InputPathIt pfirst, InputPathIt plast,
    InputCostIt cfirst, InputCostIt clast,
    GeneratorIt gfirst, GeneratorIt glast,
    OutputPathIt ofirst,
    size_t window = 1,
    size_t granularity = 1,
    const CostCompare& leq = CostCompare());

/// \brief Apply recursive path shortcutting to a container of path elements.
///
/// The routine works by recursively attempting to replace the original path
/// with shortcut paths provided by path generators, subdividing and attempting
/// to shortcut smaller segments when an invalid shortcut path is produced.
///
/// \param points The original sequence of points
/// \param costs The original sequence of costs of transitions between points
/// \param generators The sequence of path generators
/// \param shortcut_points The resulting shortcut path
/// \param leq A comparison object used to compare costs of original and
///     shortcut paths
template <
    typename PathContainer,
    typename CostsContainer,
    typename PathGeneratorsContainer,
    typename ShortcutPathContainer,
    typename CostCompare = std::less_equal<typename CostsContainer::value_type>>
bool DivideAndConquerShortcutPath(
    const PathContainer&            points,
    const CostsContainer&           costs,
    const PathGeneratorsContainer&  generators,
    ShortcutPathContainer&          shortcut_points,
    const CostCompare&              leq = CostCompare());

/// \brief Apply recursive path shortcutting to a range of path elements.
///
/// The routine works by recursively attempting to replace the original path
/// with shortcut paths provided by path generators, subdividing and attempting
/// to shortcut smaller segments when an invalid shortcut path is produced.
///
template <
    typename InputPathIt,
    typename InputCostIt,
    typename GeneratorIt,
    typename OutputPathIt,
    typename CostCompare = std::less_equal<
            typename std::iterator_traits<InputCostIt>::value_type>>
bool DivideAndConquerShortcutPath(
    InputPathIt pfirst, InputPathIt plast,
    InputCostIt cfirst, InputCostIt clast,
    GeneratorIt gfirst, GeneratorIt glast,
    OutputPathIt ofirst,
    const CostCompare& leq = CostCompare());

} // namespace shortcut
} // namespace sbpl

#include <sbpl_geometry_utils/shortcut-inl.h>

#endif

