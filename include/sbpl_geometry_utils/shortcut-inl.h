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

namespace sbpl {
namespace shortcut {

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
    typename InputPathIt,
    typename InputCostIt,
    typename GeneratorIt,
    typename OutputPathIt,
    typename CostCompare>
bool ShortcutPath(
    InputPathIt pfirst, InputPathIt plast,
    InputCostIt cfirst, InputCostIt clast,
    GeneratorIt gfirst, GeneratorIt glast,
    OutputPathIt ofirst,
    size_t window,
    size_t granularity,
    const CostCompare& leq)
{
    typedef typename std::iterator_traits<InputPathIt>::value_type PointType;
    typedef typename std::iterator_traits<InputCostIt>::value_type CostType;

    const size_t psize = (size_t)std::distance(pfirst, plast);
    const size_t csize = (size_t)std::distance(cfirst, clast);

    if (psize == 0) {
        return true;
    }

    // in all other cases, assert one cost per point transition
    if (psize != csize + 1) {
        return false;
    }

    // nothing to do with a trivial path
    if (psize < 2) {
        for (auto pit = pfirst; pit != plast; ++pit) {
            *ofirst++ = *pit;
        }
        return true;
    }

    // gather the accumulated original costs per point
    std::vector<CostType> accum(psize);
    accum[0] = (CostType)0;
    auto cit = cfirst;
    for (size_t i = 1; i < accum.size(); ++i) {
        accum[i] = accum[i - 1] + *cit++;
    }

    // double-buffered path storage for maintaining the best path returned from
    // any generator for the segment of interest
    std::vector<PointType> path1;
    std::vector<PointType> path2;
    std::vector<PointType> *wpath = &path1, *rpath = &path2;

    CostType cost; // temporary cost returned from path generators

    // initilize curr_start and curr_end to point to the current segment of
    // interest
    InputPathIt curr_start = pfirst;
    InputPathIt curr_end;
    if (granularity > psize - 1) {
        curr_end = plast;
        --curr_end;
    }
    else {
        curr_end = pfirst;
        std::advance(curr_end, granularity);
    }

    // maintain indices into the original path for accessing the accumulated
    // costs vector
    size_t start_index = 0;
    size_t end_index = std::min(psize - 1, granularity);

    bool cost_improved = false;

    // iterators to the best path found for the segment of interest; either
    // point to the original path or to a range from rpath
    InputPathIt bpfirst = curr_start, bplast = curr_end; ++bplast;

    // best cost of the segment of interest
    CostType best_cost = accum[end_index] - accum[start_index];

    // initialize the best path for the segment of interest
    for (auto git = gfirst; git != glast; ++git) {
        wpath->clear();
        if ((*git)(*curr_start, *curr_end, std::back_inserter(*wpath), cost) &&
            leq(cost, best_cost))
        {
            bpfirst = wpath->begin(); bplast = wpath->end();
            best_cost = cost;
            std::swap(rpath, wpath);
        }
    }

    *ofirst++ = *pfirst;

    while (curr_end != plast) {
        cost_improved = false;

        // expand the segment by granularity, without falling off the edge
        auto lookahead = curr_end;
        size_t look_dist = std::min(granularity, (size_t)(std::distance(curr_end, plast) - 1));
        std::advance(lookahead, look_dist);

        // look for a better path using the trajectories generated by the path
        // generators
        if (look_dist != 0) {
            CostType exp_cost = accum[end_index + look_dist] - accum[end_index];
            CostType new_cost = best_cost + exp_cost;

            // find the path between these two points with the lowest cost
            for (auto git = gfirst; git != glast; ++git) {
                wpath->clear();
                if ((*git)(*curr_start, *lookahead, std::back_inserter(*wpath), cost) &&
                    leq(cost, new_cost))
                {
                    cost_improved = true;
                    bpfirst = wpath->begin(); bplast = wpath->end();
                    new_cost = cost;
                    std::swap(rpath, wpath);
                }
            }

            best_cost = new_cost;
        }

        if (cost_improved) {
            // keep on truckin'
            std::advance(curr_end, look_dist);
            end_index += look_dist;
        }
        else {
            if (look_dist == 0) {
                // terminate without adding back the final best path since it
                // gets added after the loop
                curr_end = plast;
                end_index = psize;
            }
            else {
                // record the best path
                auto bit = bpfirst; ++bit;
                for (; bit != bplast; ++bit) {
                    *ofirst++ = *bit;
                }

                curr_start = curr_end;
                start_index = end_index;
                std::advance(curr_end, look_dist);
                end_index += look_dist;

                // reinitialize best_path for brand new segment of interest
                cost_improved = false;
                bpfirst = curr_start; bplast = curr_end; ++bplast;
                best_cost = accum[end_index] - accum[start_index];

                for (auto git = gfirst; git != glast; ++git) {
                    wpath->clear();
                    if ((*git)(*curr_start, *curr_end, std::back_inserter(*wpath), cost) &&
                        leq(cost, best_cost))
                    {
                        bpfirst = wpath->begin(); bplast = wpath->end();
                        best_cost = cost;
                        std::swap(rpath, wpath);
                    }
                }
            }
        }
    }

    // record the best path
    auto bit = bpfirst; ++bit;
    for (; bit != bplast; ++bit) {
        *ofirst++ = *bit;
    }

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
    auto costs_start = accum_point_costs.cbegin();
    auto costs_end = accum_point_costs.cend();

    return DivideAndConquerShortcutPath(start, end, costs_start, costs_end, path_generators, shortcut_path, leq);
}

template <
    typename InputPathIt,
    typename InputCostIt,
    typename GeneratorIt,
    typename OutputPathIt,
    typename CostCompare>
void DivideAndConquerShortcutPathRec(
    InputPathIt pfirst, InputPathIt plast,
    InputCostIt cfirst, InputCostIt clast,
    GeneratorIt gfirst, GeneratorIt glast,
    OutputPathIt ofirst,
    const CostCompare& leq)
{
    typedef typename std::iterator_traits<InputPathIt>::value_type PointType;
    typedef typename std::iterator_traits<InputCostIt>::value_type CostType;

    if (std::distance(pfirst, plast) == 1) {
        // need to push back the final endpoint
        *ofirst++ = *plast;
        return;
    }
    else {
        bool cost_improved = false;
        CostType best_cost = *clast - *cfirst;
        std::vector<PointType> best_path;

        // ask the path generators for a cheaper path between these two points
        std::vector<PointType> path;
        for (auto git = gfirst; git != glast; ++git) {
            path.clear();
            CostType cost;
            if ((*git)(*pfirst, *plast, std::back_inserter(path), cost) &&
                leq(cost, best_cost))
            {
                cost_improved = true;
                best_cost = cost;
                best_path.swap(path);
            }
        }

        if (cost_improved) {
            // store the result in the accumulated path
            auto bpit = best_path.begin(); ++bpit;
            for (; bpit != best_path.end(); ++bpit) {
                *ofirst++ = *bpit;
            }
            return;
        }
        else {
            // divide this segment in two and attempt to shortcut both halves
            auto segment_size = std::distance(pfirst, plast);
            auto mid = pfirst;
            auto cmid = cfirst;
            std::advance(mid, (segment_size >> 1));
            std::advance(cmid, (segment_size >> 1));

            DivideAndConquerShortcutPathRec(
                    pfirst, mid, cfirst, cmid, gfirst, glast, ofirst, leq);
            DivideAndConquerShortcutPathRec(
                    mid, plast, cmid, clast, gfirst, glast, ofirst, leq);
            return;
        }
    }
}

template <
    typename InputPathIt,
    typename InputCostIt,
    typename GeneratorIt,
    typename OutputPathIt,
    typename CostCompare>
bool DivideAndConquerShortcutPath(
    InputPathIt pfirst, InputPathIt plast,
    InputCostIt cfirst, InputCostIt clast,
    GeneratorIt gfirst, GeneratorIt glast,
    OutputPathIt ofirst,
    const CostCompare& leq)
{
    typedef typename std::iterator_traits<InputPathIt>::value_type PointType;
    typedef typename std::iterator_traits<InputCostIt>::value_type CostType;

    const size_t psize = std::distance(pfirst, plast);
    const size_t csize = std::distance(cfirst, clast);

    if (psize == 0) {
        return true;
    }

    if (psize != csize + 1) {
        return false;
    }

    if (psize < 2) {
        for (auto pit = pfirst; pit != plast; ++pit) {
            *ofirst++ = *pit;
        }
        return true;
    }

    // compute accumulated costs at each point
    std::vector<CostType> accum(psize);
    accum[0] = (CostType)0;
    auto cit = cfirst;
    for (size_t i = 1; i < accum.size(); ++i) {
        accum[i] = accum[i - 1] + *cit++;
    }

    auto first = pfirst;
    auto last = plast; --last;
    auto acfirst = accum.cbegin();
    auto aclast = accum.cend(); --aclast;

    *ofirst++ = *pfirst;
    DivideAndConquerShortcutPathRec(
            first, last, acfirst, aclast, gfirst, glast, ofirst, leq);
    return true;
}

} // namespace shortcut
} // namespace sbpl

#endif

