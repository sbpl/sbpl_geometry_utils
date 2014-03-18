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

