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
    typename PointType,
    typename CostType,
    typename PathGeneratorType,
    typename OutputIt>
struct CallablePathGenerator
{
    typedef typename PathGeneratorType::PathContainer GeneratorPathContainer;

    const PathGeneratorType& gen;
    CallablePathGenerator(const PathGeneratorType& gen) : gen(gen) { }

    bool operator()(
        const PointType& start,
        const PointType& end,
        OutputIt ofirst,
        CostType& cost)
    {
        GeneratorPathContainer shortcut;
        if (!gen.generate_path(start, end, shortcut, cost)) {
            return false;
        }
        else {
            for (const auto& point : shortcut) {
                *ofirst++ = point;
            }
            return true;
        }
    }
};

template <
    typename PathContainer,
    typename CostsContainer,
    typename PathGeneratorsContainer,
    typename ShortcutPathContainer,
    typename CostCompare>
bool ShortcutPath(
    const PathContainer&            points,
    const CostsContainer&           costs,
    const PathGeneratorsContainer&  generators,
    ShortcutPathContainer&          shortcut_points,
    size_t                          window,
    size_t                          granularity,
    const CostCompare&              leq)
{
    typedef typename PathContainer::value_type Point;
    typedef typename CostsContainer::value_type Cost;
    typedef typename PathGeneratorsContainer::value_type PathGenerator;
    typedef typename std::back_insert_iterator<ShortcutPathContainer> OutputIt;

    typedef CallablePathGenerator<Point, Cost, PathGenerator, OutputIt>
    CallablePathGeneratorType;

    std::vector<CallablePathGeneratorType> gens;
    for (const auto& gen : generators) {
        gens.push_back(CallablePathGeneratorType(gen));
    }

    return ShortcutPath(
            points.begin(), points.end(),
            costs.begin(), costs.end(),
            gens.begin(), gens.end(),
            std::back_inserter(shortcut_points),
            window,
            granularity,
            leq);
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
    typename PathContainer,
    typename CostsContainer,
    typename PathGeneratorsContainer,
    typename ShortcutPathContainer,
    typename CostCompare>
bool DivideAndConquerShortcutPath(
    const PathContainer&            points,
    const CostsContainer&           costs,
    const PathGeneratorsContainer&  generators,
    ShortcutPathContainer&          shortcut_points,
    const CostCompare&              leq)
{
    typedef typename PathContainer::value_type Point;
    typedef typename CostsContainer::value_type Cost;
    typedef typename PathGeneratorsContainer::value_type PathGenerator;
    typedef typename std::back_insert_iterator<ShortcutPathContainer> OutputIt;

    typedef CallablePathGenerator<Point, Cost, PathGenerator, OutputIt>
    CallablePathGeneratorType;

    std::vector<CallablePathGeneratorType> gens;
    for (const auto& gen : generators) {
        gens.push_back(CallablePathGeneratorType(gen));
    }

    return DivideAndConquerShortcutPath(
            points.begin(), points.end(),
            costs.begin(), costs.end(),
            gens.begin(), gens.end(),
            std::back_inserter(shortcut_points),
            leq);
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

