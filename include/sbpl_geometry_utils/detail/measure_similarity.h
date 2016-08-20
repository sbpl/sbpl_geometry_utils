#ifndef sbpl_stats_PathSimilarityMeasurer_h
#define sbpl_stats_PathSimilarityMeasurer_h

namespace sbpl
{
namespace stats
{

template <typename InputIt, typename CostFunction>
auto dynamic_time_warping(
    InputIt from_s, InputIt to_s,
    InputIt from_t, InputIt to_t,
    const CostFunction& cfun) -> decltype(cfun(*from_s, *from_t))
{
    struct CoordToIndex
    {
        std::size_t width;
        std::size_t height;

        CoordToIndex(std::size_t width, std::size_t height)
            : width(width), height(height) { }

        std::size_t operator()(std::size_t i, std::size_t j) const {
            return i * height + j;
        };
    };

    std::size_t slen = std::distance(from_s, to_s);
    std::size_t tlen = std::distance(from_t, to_t);
    std::size_t matrix_size = (slen + 1) * (tlen + 1);

    CoordToIndex indexer(slen + 1, tlen + 1);

    std::vector<decltype(cfun(*from_s, *from_t))> v(matrix_size);

    v[indexer(0, 0)] = 0;

    std::size_t i = 1, j = 1;
    for (InputIt iit = from_s; iit != to_s; ++iit) {
        j = 1;
        for (InputIt jit = from_t; jit != to_t; ++jit) {
            if (i == 1 && j == 1) {
                auto cost = cfun(*iit, *jit);
                auto c = v[indexer(i - 1, j - 1)];
                v[indexer(i, j)] = cost + c;
            }
            else if (i == 1) {
                auto cost = cfun(*iit, *jit);
                auto b = v[indexer(i, j - 1)];
                v[indexer(i, j)] = cost + b;
            }
            else if (j == 1) {
                auto cost = cfun(*iit, *jit);
                auto a = v[indexer(i - 1, j)];
                v[indexer(i, j)] = cost + a;
            }
            else {
                auto cost = cfun(*iit, *jit);
                auto a = v[indexer(i - 1, j)];
                auto b = v[indexer(i, j - 1)];
                auto c = v[indexer(i - 1, j - 1)];
                v[indexer(i, j)] = cost + std::min(std::min(a, b), c);
            }
            ++j;
        }
        ++i;
    }

    return v[indexer(slen, tlen)];
}

} // namespace stats
} // namespace sbpl

#endif
