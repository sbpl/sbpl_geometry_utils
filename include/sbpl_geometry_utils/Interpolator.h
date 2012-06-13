#ifndef SBPL_INTERPOLATOR_H
#define SBPL_INTERPOLATOR_H

#include <vector>

namespace sbpl
{

class Interpolator
{
public:
    ~Interpolator();

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
private:
    Interpolator();

    /**
     * @brief returns the shortest distance between two angles
     * @param[in] angle1 the first angle in the range [-pi, pi]
     * @param[in] angle2 the second angle in the range [-pi, pi]
     * @return the shortest distance between two angles; negative if one
     *         would travel clockwise from angle1 to angle2 to traverse
     *         this shortest distance; positive if ccw
     */
    static double angleDiff(double angle1, double angle2);

    static bool normalizeAnglesIntoRange(std::vector<double>& angles, const std::vector<double>& minLimits,
                                         const std::vector<double>& maxLimits);
};

}

#endif
