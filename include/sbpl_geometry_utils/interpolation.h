#ifndef SBPL_INTERP_INTERPOLATION_H
#define SBPL_INTERP_INTERPOLATION_H

#include <vector>

namespace sbpl
{
    namespace interp
    {

/// @brief interpolates a set of joint angles by a given increment
/// @param[in] start the start joint configuration
/// @param[in] end the end joint configuration
/// @param[in] min_limits the minimum joint limits
/// @param[in] max_limits the maximum joint limits
/// @param[in] inc the value by which each joint is moved for each waypoint
///                until the end configuration for that joint is reached
/// @param[out] path the interpolated path returned as a list of joint
///                  configurations
bool InterpolatePath(const std::vector<double>& start, const std::vector<double>& end,
                     const std::vector<double>& min_limits, const std::vector<double>& max_limits,
                     const std::vector<double>& inc,
                     std::vector<std::vector<double> >& path);

/// @brief interpolates a set of joint angles by a given increment
/// @param[in] start the start joint configuration
/// @param[in] end the end joint configuration
/// @param[in] min_limits the minimum joint limits
/// @param[in] max_limits the maximum joint limits
/// @param[in] inc the value by which each joint is moved for each waypoint
///                until the end configuration for that joint is reached
/// @param[in] continuous_joints which joints are continuous (true for continuous)
/// @param[out] path the interpolated path returned as a list of joint
///                  configurations
bool InterpolatePath(const std::vector<double>& start, const std::vector<double>& end,
                     const std::vector<double>& min_limits, const std::vector<double>& max_limits,
                     const std::vector<double>& inc, const std::vector<bool>& continuous_joints,
                     std::vector<std::vector<double> >& path);

/// @brief interpolates a set of joint angles by a given increment
/// @param[in] start the start joint configuration
/// @param[in] end the end joint configuration
/// @param[in] min_limits the minimum joint limits
/// @param[in] max_limits the maximum joint limits
/// @param[in] inc the value by which each joint is moved for each waypoint
///                until the end configuration for that joint is reached
/// @param[in] eps floating-point epsilon used for "almost-zero" comparisons
/// @param[out] path the interpolated path returned as a list of joint
///                  configurations
bool InterpolatePath(const std::vector<double>& start, const std::vector<double>& end,
                     const std::vector<double>& min_limits, const std::vector<double>& max_limits,
                     const std::vector<double>& inc, double eps,
                     std::vector<std::vector<double> >& path);

/// @brief interpolates a set of joint angles by a given increment
/// @param[in] start the start joint configuration
/// @param[in] end the end joint configuration
/// @param[in] min_limits the minimum joint limits
/// @param[in] max_limits the maximum joint limits
/// @param[in] inc the value by which each joint is moved for each waypoint
///                until the end configuration for that joint is reached
/// @param[in] continuous_joints which joints are continuous (true for continuous)
/// @param[in] eps floating-point epsilon used for "almost-zero" comparisons
/// @param[out] path the interpolated path returned as a list of joint
///                  configurations
bool InterpolatePath(const std::vector<double>& start, const std::vector<double>& end,
                     const std::vector<double>& min_limits, const std::vector<double>& max_limits,
                     const std::vector<double>& inc, const std::vector<bool>& continuous_joints,
                     double eps, std::vector<std::vector<double> >& path);

    } // end namespace interp
} // end namespace sbpl

#endif
