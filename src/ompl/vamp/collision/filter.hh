#pragma once

#include <array>
#include <vector>
#include <algorithm>
#include <cstdint>

#include <ompl/vamp/pdqsort.h>

#if defined(__x86_64__)
#include <immintrin.h>
#endif

#include <ompl/vamp/collision/math.hh>

namespace ompl::vamp::collision
{
    // LUTs taken from https://github.com/Forceflow/libmorton

    static constexpr const std::array<uint32_t, 256> MORTON_LUT_X_256 = {
        0x00000000, 0x00000001, 0x00000008, 0x00000009, 0x00000040, 0x00000041, 0x00000048, 0x00000049,
        0x00000200, 0x00000201, 0x00000208, 0x00000209, 0x00000240, 0x00000241, 0x00000248, 0x00000249,
        0x00001000, 0x00001001, 0x00001008, 0x00001009, 0x00001040, 0x00001041, 0x00001048, 0x00001049,
        0x00001200, 0x00001201, 0x00001208, 0x00001209, 0x00001240, 0x00001241, 0x00001248, 0x00001249,
        0x00008000, 0x00008001, 0x00008008, 0x00008009, 0x00008040, 0x00008041, 0x00008048, 0x00008049,
        0x00008200, 0x00008201, 0x00008208, 0x00008209, 0x00008240, 0x00008241, 0x00008248, 0x00008249,
        0x00009000, 0x00009001, 0x00009008, 0x00009009, 0x00009040, 0x00009041, 0x00009048, 0x00009049,
        0x00009200, 0x00009201, 0x00009208, 0x00009209, 0x00009240, 0x00009241, 0x00009248, 0x00009249,
        0x00040000, 0x00040001, 0x00040008, 0x00040009, 0x00040040, 0x00040041, 0x00040048, 0x00040049,
        0x00040200, 0x00040201, 0x00040208, 0x00040209, 0x00040240, 0x00040241, 0x00040248, 0x00040249,
        0x00041000, 0x00041001, 0x00041008, 0x00041009, 0x00041040, 0x00041041, 0x00041048, 0x00041049,
        0x00041200, 0x00041201, 0x00041208, 0x00041209, 0x00041240, 0x00041241, 0x00041248, 0x00041249,
        0x00048000, 0x00048001, 0x00048008, 0x00048009, 0x00048040, 0x00048041, 0x00048048, 0x00048049,
        0x00048200, 0x00048201, 0x00048208, 0x00048209, 0x00048240, 0x00048241, 0x00048248, 0x00048249,
        0x00049000, 0x00049001, 0x00049008, 0x00049009, 0x00049040, 0x00049041, 0x00049048, 0x00049049,
        0x00049200, 0x00049201, 0x00049208, 0x00049209, 0x00049240, 0x00049241, 0x00049248, 0x00049249,
        0x00200000, 0x00200001, 0x00200008, 0x00200009, 0x00200040, 0x00200041, 0x00200048, 0x00200049,
        0x00200200, 0x00200201, 0x00200208, 0x00200209, 0x00200240, 0x00200241, 0x00200248, 0x00200249,
        0x00201000, 0x00201001, 0x00201008, 0x00201009, 0x00201040, 0x00201041, 0x00201048, 0x00201049,
        0x00201200, 0x00201201, 0x00201208, 0x00201209, 0x00201240, 0x00201241, 0x00201248, 0x00201249,
        0x00208000, 0x00208001, 0x00208008, 0x00208009, 0x00208040, 0x00208041, 0x00208048, 0x00208049,
        0x00208200, 0x00208201, 0x00208208, 0x00208209, 0x00208240, 0x00208241, 0x00208248, 0x00208249,
        0x00209000, 0x00209001, 0x00209008, 0x00209009, 0x00209040, 0x00209041, 0x00209048, 0x00209049,
        0x00209200, 0x00209201, 0x00209208, 0x00209209, 0x00209240, 0x00209241, 0x00209248, 0x00209249,
        0x00240000, 0x00240001, 0x00240008, 0x00240009, 0x00240040, 0x00240041, 0x00240048, 0x00240049,
        0x00240200, 0x00240201, 0x00240208, 0x00240209, 0x00240240, 0x00240241, 0x00240248, 0x00240249,
        0x00241000, 0x00241001, 0x00241008, 0x00241009, 0x00241040, 0x00241041, 0x00241048, 0x00241049,
        0x00241200, 0x00241201, 0x00241208, 0x00241209, 0x00241240, 0x00241241, 0x00241248, 0x00241249,
        0x00248000, 0x00248001, 0x00248008, 0x00248009, 0x00248040, 0x00248041, 0x00248048, 0x00248049,
        0x00248200, 0x00248201, 0x00248208, 0x00248209, 0x00248240, 0x00248241, 0x00248248, 0x00248249,
        0x00249000, 0x00249001, 0x00249008, 0x00249009, 0x00249040, 0x00249041, 0x00249048, 0x00249049,
        0x00249200, 0x00249201, 0x00249208, 0x00249209, 0x00249240, 0x00249241, 0x00249248, 0x00249249};

    static constexpr const std::array<uint32_t, 256> MORTON_LUT_Y_256 = {
        0x00000000, 0x00000002, 0x00000010, 0x00000012, 0x00000080, 0x00000082, 0x00000090, 0x00000092,
        0x00000400, 0x00000402, 0x00000410, 0x00000412, 0x00000480, 0x00000482, 0x00000490, 0x00000492,
        0x00002000, 0x00002002, 0x00002010, 0x00002012, 0x00002080, 0x00002082, 0x00002090, 0x00002092,
        0x00002400, 0x00002402, 0x00002410, 0x00002412, 0x00002480, 0x00002482, 0x00002490, 0x00002492,
        0x00010000, 0x00010002, 0x00010010, 0x00010012, 0x00010080, 0x00010082, 0x00010090, 0x00010092,
        0x00010400, 0x00010402, 0x00010410, 0x00010412, 0x00010480, 0x00010482, 0x00010490, 0x00010492,
        0x00012000, 0x00012002, 0x00012010, 0x00012012, 0x00012080, 0x00012082, 0x00012090, 0x00012092,
        0x00012400, 0x00012402, 0x00012410, 0x00012412, 0x00012480, 0x00012482, 0x00012490, 0x00012492,
        0x00080000, 0x00080002, 0x00080010, 0x00080012, 0x00080080, 0x00080082, 0x00080090, 0x00080092,
        0x00080400, 0x00080402, 0x00080410, 0x00080412, 0x00080480, 0x00080482, 0x00080490, 0x00080492,
        0x00082000, 0x00082002, 0x00082010, 0x00082012, 0x00082080, 0x00082082, 0x00082090, 0x00082092,
        0x00082400, 0x00082402, 0x00082410, 0x00082412, 0x00082480, 0x00082482, 0x00082490, 0x00082492,
        0x00090000, 0x00090002, 0x00090010, 0x00090012, 0x00090080, 0x00090082, 0x00090090, 0x00090092,
        0x00090400, 0x00090402, 0x00090410, 0x00090412, 0x00090480, 0x00090482, 0x00090490, 0x00090492,
        0x00092000, 0x00092002, 0x00092010, 0x00092012, 0x00092080, 0x00092082, 0x00092090, 0x00092092,
        0x00092400, 0x00092402, 0x00092410, 0x00092412, 0x00092480, 0x00092482, 0x00092490, 0x00092492,
        0x00400000, 0x00400002, 0x00400010, 0x00400012, 0x00400080, 0x00400082, 0x00400090, 0x00400092,
        0x00400400, 0x00400402, 0x00400410, 0x00400412, 0x00400480, 0x00400482, 0x00400490, 0x00400492,
        0x00402000, 0x00402002, 0x00402010, 0x00402012, 0x00402080, 0x00402082, 0x00402090, 0x00402092,
        0x00402400, 0x00402402, 0x00402410, 0x00402412, 0x00402480, 0x00402482, 0x00402490, 0x00402492,
        0x00410000, 0x00410002, 0x00410010, 0x00410012, 0x00410080, 0x00410082, 0x00410090, 0x00410092,
        0x00410400, 0x00410402, 0x00410410, 0x00410412, 0x00410480, 0x00410482, 0x00410490, 0x00410492,
        0x00412000, 0x00412002, 0x00412010, 0x00412012, 0x00412080, 0x00412082, 0x00412090, 0x00412092,
        0x00412400, 0x00412402, 0x00412410, 0x00412412, 0x00412480, 0x00412482, 0x00412490, 0x00412492,
        0x00480000, 0x00480002, 0x00480010, 0x00480012, 0x00480080, 0x00480082, 0x00480090, 0x00480092,
        0x00480400, 0x00480402, 0x00480410, 0x00480412, 0x00480480, 0x00480482, 0x00480490, 0x00480492,
        0x00482000, 0x00482002, 0x00482010, 0x00482012, 0x00482080, 0x00482082, 0x00482090, 0x00482092,
        0x00482400, 0x00482402, 0x00482410, 0x00482412, 0x00482480, 0x00482482, 0x00482490, 0x00482492,
        0x00490000, 0x00490002, 0x00490010, 0x00490012, 0x00490080, 0x00490082, 0x00490090, 0x00490092,
        0x00490400, 0x00490402, 0x00490410, 0x00490412, 0x00490480, 0x00490482, 0x00490490, 0x00490492,
        0x00492000, 0x00492002, 0x00492010, 0x00492012, 0x00492080, 0x00492082, 0x00492090, 0x00492092,
        0x00492400, 0x00492402, 0x00492410, 0x00492412, 0x00492480, 0x00492482, 0x00492490, 0x00492492};

    static constexpr const std::array<uint32_t, 256> MORTON_LUT_Z_256 = {
        0x00000000, 0x00000004, 0x00000020, 0x00000024, 0x00000100, 0x00000104, 0x00000120, 0x00000124,
        0x00000800, 0x00000804, 0x00000820, 0x00000824, 0x00000900, 0x00000904, 0x00000920, 0x00000924,
        0x00004000, 0x00004004, 0x00004020, 0x00004024, 0x00004100, 0x00004104, 0x00004120, 0x00004124,
        0x00004800, 0x00004804, 0x00004820, 0x00004824, 0x00004900, 0x00004904, 0x00004920, 0x00004924,
        0x00020000, 0x00020004, 0x00020020, 0x00020024, 0x00020100, 0x00020104, 0x00020120, 0x00020124,
        0x00020800, 0x00020804, 0x00020820, 0x00020824, 0x00020900, 0x00020904, 0x00020920, 0x00020924,
        0x00024000, 0x00024004, 0x00024020, 0x00024024, 0x00024100, 0x00024104, 0x00024120, 0x00024124,
        0x00024800, 0x00024804, 0x00024820, 0x00024824, 0x00024900, 0x00024904, 0x00024920, 0x00024924,
        0x00100000, 0x00100004, 0x00100020, 0x00100024, 0x00100100, 0x00100104, 0x00100120, 0x00100124,
        0x00100800, 0x00100804, 0x00100820, 0x00100824, 0x00100900, 0x00100904, 0x00100920, 0x00100924,
        0x00104000, 0x00104004, 0x00104020, 0x00104024, 0x00104100, 0x00104104, 0x00104120, 0x00104124,
        0x00104800, 0x00104804, 0x00104820, 0x00104824, 0x00104900, 0x00104904, 0x00104920, 0x00104924,
        0x00120000, 0x00120004, 0x00120020, 0x00120024, 0x00120100, 0x00120104, 0x00120120, 0x00120124,
        0x00120800, 0x00120804, 0x00120820, 0x00120824, 0x00120900, 0x00120904, 0x00120920, 0x00120924,
        0x00124000, 0x00124004, 0x00124020, 0x00124024, 0x00124100, 0x00124104, 0x00124120, 0x00124124,
        0x00124800, 0x00124804, 0x00124820, 0x00124824, 0x00124900, 0x00124904, 0x00124920, 0x00124924,
        0x00800000, 0x00800004, 0x00800020, 0x00800024, 0x00800100, 0x00800104, 0x00800120, 0x00800124,
        0x00800800, 0x00800804, 0x00800820, 0x00800824, 0x00800900, 0x00800904, 0x00800920, 0x00800924,
        0x00804000, 0x00804004, 0x00804020, 0x00804024, 0x00804100, 0x00804104, 0x00804120, 0x00804124,
        0x00804800, 0x00804804, 0x00804820, 0x00804824, 0x00804900, 0x00804904, 0x00804920, 0x00804924,
        0x00820000, 0x00820004, 0x00820020, 0x00820024, 0x00820100, 0x00820104, 0x00820120, 0x00820124,
        0x00820800, 0x00820804, 0x00820820, 0x00820824, 0x00820900, 0x00820904, 0x00820920, 0x00820924,
        0x00824000, 0x00824004, 0x00824020, 0x00824024, 0x00824100, 0x00824104, 0x00824120, 0x00824124,
        0x00824800, 0x00824804, 0x00824820, 0x00824824, 0x00824900, 0x00824904, 0x00824920, 0x00824924,
        0x00900000, 0x00900004, 0x00900020, 0x00900024, 0x00900100, 0x00900104, 0x00900120, 0x00900124,
        0x00900800, 0x00900804, 0x00900820, 0x00900824, 0x00900900, 0x00900904, 0x00900920, 0x00900924,
        0x00904000, 0x00904004, 0x00904020, 0x00904024, 0x00904100, 0x00904104, 0x00904120, 0x00904124,
        0x00904800, 0x00904804, 0x00904820, 0x00904824, 0x00904900, 0x00904904, 0x00904920, 0x00904924,
        0x00920000, 0x00920004, 0x00920020, 0x00920024, 0x00920100, 0x00920104, 0x00920120, 0x00920124,
        0x00920800, 0x00920804, 0x00920820, 0x00920824, 0x00920900, 0x00920904, 0x00920920, 0x00920924,
        0x00924000, 0x00924004, 0x00924020, 0x00924024, 0x00924100, 0x00924104, 0x00924120, 0x00924124,
        0x00924800, 0x00924804, 0x00924820, 0x00924824, 0x00924900, 0x00924904, 0x00924920, 0x00924924};

    static constexpr const uint32_t MORTON_X_MASK = 0x49249249;
    static constexpr const uint32_t MORTON_Y_MASK = 0x92492492;
    static constexpr const uint32_t MORTON_Z_MASK = 0x24924924;
    static constexpr const uint32_t EBMASK = 0x000000FF;

    constexpr const uint32_t MORTON_FACTOR = 1000;

    inline constexpr auto remap_point(float x, float min, float max) -> uint32_t
    {
        return ((x - min) / (max - min)) * static_cast<float>(MORTON_FACTOR);
    }

    inline auto morton_lut(uint32_t x, uint32_t y, uint32_t z) -> uint32_t
    {
        uint32_t answer = 0;
        for (auto i = 4u; i > 0; --i)
        {
            const auto shift = (i - 1) * 8;
            answer = answer << 24 |
                     (MORTON_LUT_Z_256[(z >> shift) & EBMASK] | MORTON_LUT_Y_256[(y >> shift) & EBMASK] |
                      MORTON_LUT_X_256[(x >> shift) & EBMASK]);
        }

        return answer;
    }

#if defined(__x86_64__)
    inline auto morton_pdep(uint32_t x, uint32_t y, uint32_t z) -> uint32_t
    {
        return _pdep_u32(x, MORTON_X_MASK) | _pdep_u32(y, MORTON_Y_MASK) | _pdep_u32(z, MORTON_Z_MASK);
    }

    constexpr auto morton_encode = morton_pdep;
#else
    constexpr auto morton_encode = morton_lut;
#endif

    // Filter a pointcloud into a smaller representative pointcloud.
    //
    // Inputs
    // - `pc`: the initial pointcloud
    // - `min_dist`: The minimum distance between two points to be considered distinct.
    // - `max_range`: The maximum distance for a point from the origin to be retained from `pc`.
    // - `origin`: The location of the origin.
    // - `workspace_min`: The minimum vertex of the AABB describing the workspace.
    // - `workspace_max`: The maximum vertex of the AABB describing the workspace.
    //
    // The AABB bounded by `workspace_min` and `workspace_max` must contain all points in `pc`.
    //
    // Returns a subset of `pc`, subject to the following conditions:
    // - If there exists `p1`, `p2` in `pc`, such that `d(p1, p2) <= min_dist`, then one of `p1` or `p2` _may_
    //   be removed.
    // - If `d(p, origin) >= max_range`, `p` will be removed.
    template <typename PointCloud>
    auto filter_pointcloud(
        const PointCloud &pc,
        float min_dist,
        float max_range,
        Point origin,
        Point workspace_min,
        Point workspace_max,
        bool cull) -> std::vector<Point>
    {
        if (pc.shape(0) == 0)
        {
            return std::vector<Point>();
        }

        const auto sqdist = min_dist * min_dist;
        const auto sqrange = max_range * max_range;
        auto min = std::min({origin[0] - max_range, origin[1] - max_range, origin[2] - max_range});
        auto max = std::min({origin[0] + max_range, origin[1] + max_range, origin[2] + max_range});

        std::vector<std::pair<uint32_t, uint32_t>> morton;
        morton.resize(pc.shape(0));

        auto hi = 0U;
        // Step 1: filter out any points so far from the origin that we wouldn't collide with them anyway.
        for (auto i = 0U; i < pc.shape(0); ++i)
        {
            if (cull)
            {
                if (sql2_3(pc(i, 0), pc(i, 1), pc(i, 2), origin[0], origin[1], origin[2]) < sqrange and
                    workspace_min[0] <= pc(i, 0) and pc(i, 0) <= workspace_max[0] and
                    workspace_min[1] <= pc(i, 1) and pc(i, 1) <= workspace_max[1] and
                    workspace_min[2] <= pc(i, 2) and pc(i, 2) <= workspace_max[2])
                {
                    morton[hi++].first = i;
                }
            }
            else
            {
                morton[hi++].first = i;
            }
        }

        std::array<unsigned int, 3> coordinates{0, 1, 2};

        // For each space-filling curve...
        do
        {
            // Step 2: Sort points according to their position in the space-filling curve.
            float new_min = max;
            float new_max = min;
            for (auto &h : morton)
            {
                auto c0 = remap_point(pc(h.first, coordinates[0]), min, max);
                auto c1 = remap_point(pc(h.first, coordinates[1]), min, max);
                auto c2 = remap_point(pc(h.first, coordinates[2]), min, max);

                new_min = std::min({new_min, pc(h.first, 0), pc(h.first, 1), pc(h.first, 2)});
                new_max = std::max({new_max, pc(h.first, 0), pc(h.first, 1), pc(h.first, 2)});

                h.second = morton_encode(c0, c1, c2);
            }

            pdqsort_branchless(
                morton.begin(),
                morton.end(),
                [](const auto &a, const auto &b) { return a.second < b.second; });

            std::vector<std::pair<uint32_t, uint32_t>> filtered;
            filtered.reserve(morton.size());
            filtered.emplace_back(morton.front());

            // Step 3: Any adjacent pair of points in the sort which are too close get eliminated.
            for (auto i = 1u; i < morton.size(); ++i)
            {
                const auto h1 = morton[i].first;
                const auto h2 = filtered.back().first;

                if (sql2_3(pc(h1, 0), pc(h1, 1), pc(h1, 2), pc(h2, 0), pc(h2, 1), pc(h2, 2)) > sqdist)
                {
                    filtered.emplace_back(morton[i]);
                }
            }

            morton = std::move(filtered);

            max = (new_max + max) / 2.;
            min = (new_min + min) / 2.;

        } while (std::next_permutation(coordinates.begin(), coordinates.end()));

        std::vector<Point> filtered;
        filtered.reserve(morton.size());

        for (const auto &h : morton)
        {
            filtered.emplace_back(Point{pc(h.first, 0), pc(h.first, 1), pc(h.first, 2)});
        }

        return filtered;
    }

    template <>
    inline auto filter_pointcloud(
        const std::vector<Point> &pc,
        float min_dist,
        float max_range,
        Point origin,
        Point workspace_min,
        Point workspace_max,
        bool cull) -> std::vector<Point>
    {
        struct PointcloudWrapper
        {
            inline auto shape(std::size_t dim) const noexcept -> std::size_t
            {
                if (dim == 0)
                {
                    return pc.size();
                }

                if (dim == 1)
                {
                    return 3;
                }

                return -1;
            }

            inline auto operator()(std::size_t i, std::size_t j) const noexcept -> typename Point::value_type
            {
                return pc[i][j];
            }

            const std::vector<Point> &pc;
        };

        return filter_pointcloud(
            PointcloudWrapper{pc},
            min_dist,
            max_range,
            std::move(origin),
            std::move(workspace_min),
            std::move(workspace_max),
            cull);
    }
}  // namespace ompl::vamp::collision
