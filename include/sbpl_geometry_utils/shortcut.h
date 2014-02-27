#ifndef sbpl_shortcutting_h
#define sbpl_shortcutting_h

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

    virtual bool generate_path(const PointType& start, const PointType& end, PathContainerType& path_out, CostType& costs_out) const = 0;
};

template <typename PathContainer,
          typename CostsContainer,
          typename PathGeneratorsContainer,
          typename ShortcutPathContainer,
          typename CostCompare = std::less_equal<typename CostsContainer::value_type>>
bool ShortcutPath(const PathContainer& orig_path,
                  const CostsContainer& orig_path_costs,
                  const PathGeneratorsContainer& path_generators,
                  ShortcutPathContainer& shortcut_path,
                  const CostCompare& leq = CostCompare());

} // namespace shortcut

} // namespace sbpl

#include <sbpl_geometry_utils/shortcut-inl.h>

#endif

