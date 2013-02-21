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
