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

#ifndef sbpl_shortcut_inl_h
#define sbpl_shortcut_inl_h

//#include <sbpl_geometry_utils/shortcut.h>
//#include <iterator>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <vector>

namespace sbpl
{

namespace shortcut
{

template <
    typename PathContainer,
    typename CostsContainer,
    typename PathGeneratorsContainer,
    typename ShortcutPathContainer,
    typename CostCompare>
bool ShortcutPath(
    const PathContainer& orig_path,
    const CostsContainer& orig_path_costs,
    const PathGeneratorsContainer& path_generators,
    ShortcutPathContainer& shortcut_path,
    unsigned window,
    unsigned granularity,
    const CostCompare& leq)
{
    typedef typename PathContainer::value_type PointType;
    typedef typename CostsContainer::value_type CostType;
    typedef typename PathGeneratorsContainer::value_type PathGeneratorType;

    // assert one cost per point transition
    if (orig_path.size() != orig_path_costs.size() + 1) {
        return false;
    }

    // nothing to do with a trivial path
    if (orig_path.size() < 2) {
        return true;
    }

    // gather the accumulated original costs per point
    std::vector<CostType> accum_point_costs(orig_path.size());
    accum_point_costs[0] = 0.0;
    for (int i = 1; i < (int)accum_point_costs.size(); ++i) {
        accum_point_costs[i] = accum_point_costs[i - 1] + orig_path_costs[i - 1];
    }

    std::vector<PointType> result_traj;

    typename PathContainer::const_iterator curr_start = orig_path.begin();
    typename PathContainer::const_iterator curr_end;
    if (granularity > orig_path.size() - 1) {
        curr_end = orig_path.end();
        --curr_end;
    }
    else {
        curr_end = orig_path.begin();
        std::advance(curr_end, granularity);
    }

    // maintain indices into the original path for accessing the accumulated costs vector
    unsigned start_index = 0;
    unsigned end_index = std::min(orig_path.size() - 1, (typename PathContainer::size_type)granularity);

    bool cost_improved = false;
    typename PathGeneratorType::PathContainer best_path;
    for (auto it = curr_start; it != curr_end; ++it) {
        best_path.push_back(*it);
    }
    best_path.push_back(*curr_end);
    typename PathGeneratorType::Cost best_cost = accum_point_costs[end_index] - accum_point_costs[start_index];
    {
        typename PathGeneratorType::PathContainer path;
        typename PathGeneratorType::Cost cost;
        for (auto it = path_generators.begin(); it != path_generators.end(); ++it) {
            if (it->generate_path(*curr_start, *curr_end, path, cost) &&
                leq(cost, best_cost))
            {
                best_cost = cost;
                best_path.swap(path);
            }
        }
    }

    while (curr_end != orig_path.end()) {
        cost_improved = false;
        CostType orig_cost = best_cost;

        // advance forward by as much as we can up to granularity without falling off the edge
        typename PathContainer::const_iterator lookahead = curr_end;
        typedef typename PathContainer::difference_type difference_type;
        difference_type distance_forward = std::min((difference_type)granularity, std::distance(curr_end, orig_path.end()) - 1);
        std::advance(lookahead, distance_forward);

        // look for a better path using the trajectories generated by the path generators
        if (distance_forward != 0)
        {
            for (auto it = path_generators.begin(); it != path_generators.end(); ++it) {
                // find the path between these two points with the lowest cost
                typename PathGeneratorType::PathContainer path;
                typename PathGeneratorType::Cost cost;
                if (it->generate_path(*curr_start, *lookahead, path, cost) &&
                    leq(cost, best_cost + accum_point_costs[end_index + distance_forward] - accum_point_costs[end_index]))
                {
                    cost_improved = true;
                    best_cost = cost;
                    best_path.swap(path);
                }
            }
        }

        if (cost_improved) {
            // keep on truckin'
            std::advance(curr_end, distance_forward);
            end_index += distance_forward;
        }
        else {
            if (distance_forward == 0) {
                // terminate without adding back the final best path since it gets added after the loop
                curr_end = orig_path.end();
                end_index = orig_path.size();
            }
            else {
                // remove duplicate points to avoid double counting
                if (!result_traj.empty()) {
                    result_traj.pop_back();
                }
                result_traj.insert(result_traj.end(), best_path.begin(), best_path.end()); // insert path from start to end

                curr_start = curr_end;
                start_index = end_index;
                std::advance(curr_end, distance_forward);
                end_index += distance_forward;

                // reinitialize best_path
                cost_improved = false;
                best_path.clear();
                for (auto it = curr_start; it != curr_end; ++it) {
                    best_path.push_back(*it);
                }
                best_path.push_back(*curr_end);
                best_cost = accum_point_costs[end_index] - accum_point_costs[start_index];
                {
                    typename PathGeneratorType::PathContainer path;
                    typename PathGeneratorType::Cost cost;
                    for (auto it = path_generators.begin(); it != path_generators.end(); ++it) {
                        if (it->generate_path(*curr_start, *curr_end, path, cost) &&
                            leq(cost, best_cost))
                        {
                            best_cost = cost;
                            best_path.swap(path);
                        }
                    }
                }
            }
        }
    }

    if (!result_traj.empty()) {
        result_traj.pop_back();
    }
    result_traj.insert(result_traj.end(), best_path.begin(), best_path.end());

    shortcut_path.assign(result_traj.begin(), result_traj.end());
    return true;
}

template <
    typename PathContainerIt,
    typename AccumCostsContainerIt,
    typename PathGeneratorsContainer,
    typename ShortcutPathContainer,
    typename CostCompare>
bool DivideAndConquerShortcutPath(
    const PathContainerIt& orig_path_start,
    const PathContainerIt& orig_path_end,
    const AccumCostsContainerIt& cost_start,
    const AccumCostsContainerIt& cost_end,
    const PathGeneratorsContainer& path_generators,
    ShortcutPathContainer& accum_shortcut_path,
    const CostCompare& leq)
{

    typedef typename PathGeneratorsContainer::value_type PathGeneratorType;
    typedef typename PathGeneratorType::PathContainer PathContainer;
    typedef typename PathGeneratorType::Cost Cost;
    typedef typename PathContainer::difference_type difference_type;

    if (std::distance(orig_path_start, orig_path_end) == 1) {
        // need to push back these two points
        accum_shortcut_path.push_back(*orig_path_start);
        accum_shortcut_path.push_back(*orig_path_end);
        return true;
    }
    else {
        bool cost_improved = false;
        Cost best_cost = *cost_end - *cost_start;
        PathContainer best_path;

        // ask the path generators for a cheaper path between these two points
        for (auto it = path_generators.cbegin(); it != path_generators.cend(); ++it) {
            PathContainer path;
            Cost cost;
            if (it->generate_path(*orig_path_start, *orig_path_end, path, cost) && leq(cost, best_cost)) {
                cost_improved = true;
                best_cost = cost;
                best_path.swap(path);
            }
        }

        if (cost_improved) {
            // store the result in the accumulated path
            if (!accum_shortcut_path.empty()) {
                accum_shortcut_path.pop_back();
            }
            accum_shortcut_path.insert(accum_shortcut_path.end(), best_path.begin(), best_path.end());
            return true;
        }
        else {
            // divide this segment in two and attempt to shortcut both halves
            difference_type segment_size = std::distance(orig_path_start, orig_path_end);
            auto mid = orig_path_start;
            auto cost_mid = cost_start;
            std::advance(mid, (segment_size >> 1));
            std::advance(cost_mid, (segment_size >> 1));

            DivideAndConquerShortcutPath(orig_path_start, mid, cost_start, cost_mid, path_generators, accum_shortcut_path, leq);
            DivideAndConquerShortcutPath(mid, orig_path_end, cost_mid, cost_end, path_generators, accum_shortcut_path, leq);
            return true;
        }
    }

    return true;
}

template <
    typename PathContainer,
    typename CostsContainer,
    typename PathGeneratorsContainer,
    typename ShortcutPathContainer,
    typename CostCompare>
bool DivideAndConquerShortcutPath(
    const PathContainer& orig_path,
    const CostsContainer& orig_path_costs,
    const PathGeneratorsContainer& path_generators,
    ShortcutPathContainer& shortcut_path,
    const CostCompare& leq)
{
    typedef typename PathContainer::value_type PointType;
    typedef typename CostsContainer::value_type CostType;
    typedef typename PathGeneratorsContainer::value_type PathGeneratorType;

    // assert one cost per point transition
    if (orig_path.size() != orig_path_costs.size() + 1) {
        return false;
    }

    // nothing to do with a trivial path
    if (orig_path.size() < 2) {
        return true;
    }

    // gather the accumulated original costs per point
    std::vector<CostType> accum_point_costs(orig_path.size());
    accum_point_costs[0] = 0.0;
    for (int i = 1; i < (int)accum_point_costs.size(); ++i) {
        accum_point_costs[i] = accum_point_costs[i - 1] + orig_path_costs[i - 1];
    }

    typename PathContainer::const_iterator start = orig_path.cbegin();
    typename PathContainer::const_iterator end = --orig_path.cend();
    typename std::vector<CostType>::const_iterator costs_start = accum_point_costs.cbegin();
    typename std::vector<CostType>::const_iterator costs_end = accum_point_costs.cend();

    return DivideAndConquerShortcutPath(start, end, costs_start, costs_end, path_generators, shortcut_path, leq);
}

} // namespace shortcut

} // namespace sbpl

#endif

