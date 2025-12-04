#pragma once

#include <algorithm>
#include <cstdint>
#include <new>
#include <numeric>
#include <limits>
#include <vector>
#include <cmath>
#include <cassert>

#include <ompl/vamp/pdqsort.h>

#include <ompl/vamp/collision/math.hh>
#include <ompl/vamp/vector.hh>

namespace ompl::vamp::collision
{

    // A 3D cell volume.
    struct Volume
    {
        // The lower bound on the volume contained by the cell of this frame.
        Point lower;
        // The upper bound on the volume contained by the cell of this frame.
        Point upper;

        auto extend(const Point &point) noexcept
        {
            for (auto k = 0u; k < 3; k++)
            {
                lower[k] = std::min(lower[k], point[k]);
                upper[k] = std::max(upper[k], point[k]);
            }
        }

        inline auto contained_by_internal_ball(const Point &p, float r) const noexcept -> float
        {
            assert(distsq_to(p) <= std::numeric_limits<float>::epsilon());
            const float d0 = std::max(p[0] - lower[0], upper[0] - p[0]);
            const float d1 = std::max(p[1] - lower[1], upper[1] - p[1]);
            const float d2 = std::max(p[2] - lower[2], upper[2] - p[2]);
            return (d0 * d0 + d1 * d1 + d2 * d2) <= r;
        }

        inline auto distsq_to(const Point &point) const noexcept -> float
        {
            const float d0 = point[0] - std::clamp(point[0], lower[0], upper[0]);
            const float d1 = point[1] - std::clamp(point[1], lower[1], upper[1]);
            const float d2 = point[2] - std::clamp(point[2], lower[2], upper[2]);

            return d0 * d0 + d1 * d1 + d2 * d2;
        }

        inline auto affords(const Point &point, const float max_affordance_l2) const noexcept -> bool
        {
            return distsq_to(point) <= max_affordance_l2;
        }
    };

    struct CAPT
    {
        using FVectorT = FloatVector<>;
        using IVectorT = IntVector<>;

        // A hacky structure used for unrolling recursive tree construction.
        // Emulates a stack frame.
        struct BuildFrame
        {
            // The index of where this frame's region of the points buffer begins.
            uint32_t points_begin;

            // The number of points visible in `points`.
            // Must be greater than 0.
            uint32_t how_many_points;

            // This frame's index in the test buffer.
            uint32_t i;

            // The points which might collide with the cell containing this frame.
            std::vector<uint32_t> afford;

            // The volume contained by this cell.
            Volume volume;

            // The next dimension to branch on.
            uint8_t d;

            // Validate all invariants of this frame.
            inline auto is_valid() const noexcept -> bool
            {
                for (uint8_t k = 0; k < 3; k++)
                {
                    if (volume.lower[d] > volume.upper[d])
                    {
                        return false;
                    }
                }

                return true;
            }
        };

        inline auto median_partition(
            const std::vector<Point> &points,
            std::vector<uint32_t> &argsort,
            const uint32_t begin,
            const uint32_t end,
            const uint8_t k) -> float
        {
            const auto *const points_ptr = points.data();
            pdqsort_branchless(
                argsort.begin() + begin,
                argsort.begin() + end,
                [k, points_ptr](const uint32_t a, const uint32_t b) noexcept -> bool
                { return points_ptr[a][k] < points_ptr[b][k]; });

            const uint32_t how_many = end - begin;
            const uint32_t middle = begin + how_many / 2;
            return (points[argsort[middle - 1]][k] + points[argsort[middle]][k]) / 2.0;
        }

        inline auto subdivide(
            const std::vector<Point> &points,
            std::vector<uint32_t> &argsort,
            const float max_affordance_l1,
            const float max_affordance_l2,
            const float min_affordance_l2,
            BuildFrame frame) noexcept -> void
        {
            assert(frame.how_many_points != 0);
            if (frame.how_many_points == 1)
            {
                const auto &cell_rep = points[argsort[frame.points_begin]];
                Volume aabb = {cell_rep, cell_rep};
                if (std::isfinite(cell_rep[0]))
                {
                    // don't bother including infinities in our affordance buffers
                    aabb_top.extend(cell_rep);

                    // Representative point of the cell comes first
                    std::array<float, FVectorT::num_scalars> xs = {cell_rep[0]};
                    std::array<float, FVectorT::num_scalars> ys = {cell_rep[1]};
                    std::array<float, FVectorT::num_scalars> zs = {cell_rep[2]};

                    uint8_t j = 1;

                    if (!frame.volume.contained_by_internal_ball(cell_rep, min_affordance_l2))
                    {
                        // affordances.reserve(start + frame.afford.size());
                        for (const uint32_t id : frame.afford)
                        {
                            const Point &point = points[id];
                            if (frame.volume.affords(point, max_affordance_l2))
                            {
                                aabb.extend(point);

                                xs[j] = point[0];
                                ys[j] = point[1];
                                zs[j] = point[2];

                                j++;

                                if (j == FVectorT::num_scalars)
                                {
                                    affordances[0].emplace_back(xs);
                                    affordances[1].emplace_back(ys);
                                    affordances[2].emplace_back(zs);
                                    j = 0;
                                }
                            }
                        }
                    }

                    if (j > 0)
                    {
                        for (uint8_t jj = j; jj < FVectorT::num_scalars; jj++)
                        {
                            // ensure that extra garbage does not result in a false collision
                            xs[jj] = std::numeric_limits<float>::infinity();
                            ys[jj] = std::numeric_limits<float>::infinity();
                            zs[jj] = std::numeric_limits<float>::infinity();
                        }

                        affordances[0].emplace_back(xs);
                        affordances[1].emplace_back(ys);
                        affordances[2].emplace_back(zs);
                    }
                }

                aabbs.emplace_back(aabb);
                aff_starts.emplace_back(affordances[0].size());
            }
            else
            {
                const float test = median_partition(
                    points, argsort, frame.points_begin, frame.points_begin + frame.how_many_points, frame.d);
                tests[frame.i] = test;
                assert(test <= frame.volume.upper[frame.d]);
                assert(test >= frame.volume.lower[frame.d]);

                const uint32_t next_width = frame.how_many_points / 2;
                Volume lo_vol = frame.volume;
                Volume hi_vol = frame.volume;
                lo_vol.upper[frame.d] = test;
                hi_vol.lower[frame.d] = test;

                std::vector<uint32_t> hi_afford = std::move(frame.afford);
                std::vector<uint32_t> lo_afford(hi_afford.size(), 0);

                uint32_t hi_len = 0;
                uint32_t lo_len = 0;
                for (const auto idx : hi_afford)
                {
                    if (points[idx][frame.d] <= test + r_max)
                    {
                        lo_afford[lo_len++] = idx;
                    }

                    if (points[idx][frame.d] >= test - r_max)
                    {
                        hi_afford[hi_len++] = idx;
                    }
                }

                uint32_t new_hi_afford = frame.points_begin;
                uint32_t new_lo_afford = frame.points_begin + next_width;
                while (new_hi_afford < frame.points_begin + next_width and
                       points[argsort[new_hi_afford]][frame.d] >= test - r_max and
                       std::isfinite(points[argsort[new_hi_afford]][frame.d]))
                {
                    ++new_hi_afford;
                }

                while (new_lo_afford < frame.points_begin + frame.how_many_points and
                       points[argsort[new_lo_afford]][frame.d] <= test + r_max and
                       std::isfinite(points[argsort[new_lo_afford]][frame.d]))
                {
                    ++new_lo_afford;
                }

                uint32_t num_new_hi = new_hi_afford - frame.points_begin;
                uint32_t num_new_lo = new_lo_afford - (frame.points_begin + next_width);

                hi_afford.resize(hi_len + num_new_hi);
                std::copy(
                    argsort.begin() + frame.points_begin,
                    argsort.begin() + new_hi_afford,
                    hi_afford.begin() + hi_len);
                lo_afford.resize(lo_len + num_new_lo);
                std::copy(
                    argsort.begin() + frame.points_begin + next_width,
                    argsort.begin() + new_lo_afford,
                    lo_afford.begin() + lo_len);

                const uint8_t next_d = (frame.d + 1) % 3;
                subdivide(
                    points,
                    argsort,
                    max_affordance_l1,
                    max_affordance_l2,
                    min_affordance_l2,
                    BuildFrame{
                        frame.points_begin,
                        next_width,
                        2 * frame.i + 1,
                        std::move(lo_afford),
                        std::move(lo_vol),
                        next_d});

                subdivide(
                    points,
                    argsort,
                    max_affordance_l1,
                    max_affordance_l2,
                    min_affordance_l2,
                    BuildFrame{
                        frame.points_begin + next_width,
                        next_width,
                        2 * frame.i + 2,
                        std::move(hi_afford),
                        std::move(hi_vol),
                        next_d});
            }
        }

        // Construct a new affordance tree.
        //
        // Inputs
        // - `points`: buffer filled with 3-dimensional points. All these points will be included in the tree.
        // - `r_min`: The minimum radius that queries to this tree will request, inclusive.
        // - `r_max`: The maximum radius-squared that queries to this tree will request, inclusive.
        // - `r_point`: The radius to associate with each point in the tree.
        CAPT(
            const std::vector<Point> &points,
            const float r_min,
            const float r_max,
            const float r_point) noexcept
          : r_min{r_min}, r_max{r_max}, r_point{r_point}
        {
            const float max_affordance_l1 = r_max + r_point;
            const float max_affordance_l2 = max_affordance_l1 * max_affordance_l1;
            const float min_affordance_l2 = (r_min + r_point) * (r_min + r_point);

            // calculate nlog2 by rounding how_many up to the next power of 2
            nlog2 = 0;
            while ((1u << static_cast<std::size_t>(nlog2)) < points.size())
            {
                nlog2++;
            }

            // Temp buffer to pad out to a power of 2
            // possible optimization: take ownership of passed-in points, then realloc to avoid a memcpy
            const std::size_t pow2_size = (1u << static_cast<std::size_t>(nlog2));
            std::vector<Point> points2 = points;
            points2.reserve(pow2_size);
            points2.insert(
                points2.end(),
                pow2_size - points2.size(),
                Point{
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity()});

            aabb_top = {
                {std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity()},
                {-std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity()}};

            tests.reserve(points2.size());
            tests.insert(tests.end(), points2.size() - 1, std::numeric_limits<float>::quiet_NaN());

            aff_starts.reserve(points2.size() + 1);
            aff_starts.emplace_back(0);

            affordances[0].reserve(points2.size() * 100);
            affordances[1].reserve(points2.size() * 100);
            affordances[2].reserve(points2.size() * 100);

            std::vector<uint32_t> argsort;
            argsort.resize(points2.size());
            std::iota(argsort.begin(), argsort.end(), 0);

            subdivide(
                points2,
                argsort,
                max_affordance_l1,
                max_affordance_l2,
                min_affordance_l2,
                BuildFrame{
                    0u,
                    static_cast<uint32_t>(points2.size()),
                    0u,
                    {},
                    {{-std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity()},
                     {std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::infinity()}},
                    0u});
        }

        //  Test whether a sphere centered at `center` with radius-squared `radius_sq` collides with any
        //  point in this tree.
        //  Returns `true` if in collision and `false` if not.
        [[nodiscard]] auto collides(const Point &center, float r) const noexcept -> bool
        {
            if (aabb_top.distsq_to(center) > r * r)
            {
                return false;
            }

            std::size_t test_idx = 0;
            for (uint8_t i = 0, k = 0; i < nlog2; i++)
            {
                test_idx = 2 * test_idx + 1 + (center[k] >= tests[test_idx]);
                k = (k + 1) % 3;
            }

            const std::size_t z = test_idx - tests.size();

            r += r_point;
            const float radius_sq = r * r;
            if (aabbs[z].distsq_to(center) > radius_sq)
            {
                return false;
            }

            const uint32_t start = aff_starts[z];
            const uint32_t end = aff_starts[z + 1];

            const auto xc = FVectorT::fill(center[0]);
            const auto yc = FVectorT::fill(center[1]);
            const auto zc = FVectorT::fill(center[2]);
            const auto rc = FVectorT::fill(radius_sq);
            for (uint32_t i = start; i < end; i++)
            {
                const auto distsq =
                    sql2_3(affordances[0][i], affordances[1][i], affordances[2][i], xc, yc, zc);
                if (distsq.test_any_less_equal(rc))
                {
                    return true;
                }
            }

            return false;
        }

        // Determine whether any of a set of spheres collides with a point in this tree.
        //
        // Templates
        //
        // - `FVectorT`: type of a vector of floats
        // - `IVectorT`: type of a vector of integer indexes
        //
        // Inputs
        //
        // - `centers`: (x, y, z) struct-of-arrays of the centers of each sphere.
        // - `radii`: SIMD vector of the radii of each sphere.
        auto collides_simd(const std::array<FVectorT, 3> &centers, FVectorT radii) const noexcept -> bool
        {
            // Test against top AABB
            FVectorT inbounds =
                (centers[0] + radii >= aabb_top.lower[0]) & (centers[0] - radii <= aabb_top.upper[0]);

            for (uint8_t k = 1; k < 3; k++)
            {
                inbounds = inbounds & (centers[k] + radii >= aabb_top.lower[k]) &
                           (centers[k] - radii <= aabb_top.upper[k]);
            }

            if (inbounds.none())
            {
                return false;
            }

            FVectorT these_tests = FVectorT::fill(tests[0]);
            FVectorT cmp_results = centers[0].greater_equal(these_tests);
            auto idxs = (cmp_results >> 31U).template as<IVectorT>() + 1;

            // Search downward through the tree, parallel across each point
            for (uint8_t i = 1, k = 1; i < nlog2; i++)
            {
                these_tests = FVectorT::gather(tests.data(), idxs);
                cmp_results = centers[k].greater_equal(these_tests);
                idxs = (idxs << 1U) + (cmp_results >> 31U).template as<IVectorT>() + 1;
                k = (k + 1) % 3;
            }

            const IVectorT zs = idxs - tests.size();

            // Test whether points are in the AABBs
            // NOTE: Now is when we need to add r_point, since these AABBs are really "point volume AABBs" -
            // we can't just test if the query is in the AABB, but rather if it's in the AABB when fattened by
            // the radius of the points the AABB contains
            radii = radii + r_point;
            IVectorT zs6 = zs * 6;
            const float *const aabb_ptr = &aabbs.front().lower.front();

            const auto rc_sq = radii * radii;

            auto d0 = centers[0] -
                      centers[0].clamp(FVectorT::gather(aabb_ptr, zs6), FVectorT::gather(aabb_ptr, zs6 + 3));
            auto d1 =
                centers[1] -
                centers[1].clamp(FVectorT::gather(aabb_ptr, zs6 + 1), FVectorT::gather(aabb_ptr, zs6 + 4));
            auto d2 =
                centers[2] -
                centers[2].clamp(FVectorT::gather(aabb_ptr, zs6 + 2), FVectorT::gather(aabb_ptr, zs6 + 5));

            auto distsq_to = d0 * d0 + d1 * d1 + d2 * d2;
            inbounds = inbounds & (distsq_to <= rc_sq);
            if (inbounds.none())
            {
                return false;
            }

            // Convert the terminal test indices to reference indices for the affordance buffer
            const auto *affdata = reinterpret_cast<const int32_t *>(aff_starts.data());
            const IVectorT starts_v = IVectorT::gather(affdata, zs);
            IVectorT ends_v = inbounds.template as<IVectorT>() & IVectorT::gather(affdata, zs + 1);

            const auto starts = starts_v.to_array();
            const auto ends = ends_v.to_array();

            for (uint8_t j = 0; j < FVectorT::num_scalars; j++)
            {
                const auto xc = centers[0].broadcast(j);
                const auto yc = centers[1].broadcast(j);
                const auto zc = centers[2].broadcast(j);
                const auto rc = rc_sq.broadcast(j);
                for (auto i = starts[j]; i < ends[j]; ++i)
                {
                    const auto distsq =
                        sql2_3(affordances[0][i], affordances[1][i], affordances[2][i], xc, yc, zc);
                    if (distsq.test_any_less_equal(rc))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        auto is_valid() const noexcept -> bool
        {
            /// check relative sizing of tests / aff_starts
            if (tests.size() + 2 != aff_starts.size())
            {
                return false;
            }

            // check that we have a power of 2 number of points
            if (((tests.size() + 1) & tests.size()) != 0)
            {
                return false;
            }

            if (aff_starts.back() != affordances[0].size())
            {
                return false;
            }

            if (aff_starts.front() != 0)
            {
                return false;
            }

            for (std::size_t i = 0; i < aff_starts.size() - 1; i++)
            {
                if (aff_starts[i] > aff_starts[i + 1])
                {
                    return false;
                }

                if (aff_starts[i] == aff_starts[i + 1])
                {
                    continue;
                }
            }

            return true;
        }

        // Destroy the affordance tree, freeing all owned memory.
        ~CAPT() = default;

        template <class T>
        struct AlignedAllocator
        {
            using value_type = T;
            inline static constexpr std::align_val_t alignment{32};

            constexpr AlignedAllocator() noexcept = default;
            constexpr AlignedAllocator(const AlignedAllocator &) noexcept = default;

            template <typename U>
            constexpr AlignedAllocator(const AlignedAllocator<U> &) noexcept
            {
            }

            [[nodiscard]] value_type *allocate(std::size_t num_elements)
            {
                if (num_elements > std::numeric_limits<std::size_t>::max() / sizeof(value_type))
                {
                    throw std::bad_array_new_length();
                }

                const auto num_bytes = num_elements * sizeof(value_type);
                return reinterpret_cast<value_type *>(::operator new[](num_bytes, alignment));
            }

            void deallocate(value_type *allocated_ptr, [[maybe_unused]] std::size_t num_allocated_bytes)
            {
                ::operator delete[](allocated_ptr, alignment);
            }
        };

        // The test buffer for this tree.
        // Contains (2 ^ nlog2) - 1 points.
        std::vector<float, AlignedAllocator<float>> tests;

        //  Indexes for the starts of each affordance buffer in `affordances` for the corresponding
        //  point after the outcome of all the tests.
        //  Contains (2 ^ nlog2) + 1 elements.
        //  The last element is not associated with any element but instead is the length of
        //  `affordances` (in terms of the number of floats).
        //  We use `int32_t` instead of `std::size_t` so that we can use the same registers for integer
        //  operations as we do for floats.
        std::vector<uint32_t, AlignedAllocator<uint32_t>> aff_starts;

        // The combined affordance buffers for the entire tree.
        // At the start of each affordance buffer, we store 6 floats for the corner point of an axis-aligned
        // bounding box. Contains `aff_starts[2 ^ nlog2]` points, or `4 * aff_starts[2 ^ nlog2] + 6 * 2 ^
        // nlog2` float values.
        std::array<std::vector<FVectorT>, 3> affordances;

        // Axis-aligned bounding boxes for the set of afforded points in each cell.
        std::vector<Volume> aabbs;

        // The AABB containing all points.
        Volume aabb_top;

        // The minimum legal radius for a range query (inclusive).
        float r_min;

        // The maximum legal radius for a range query (inclusive).
        float r_max;

        // The offset radius to use for points in the point cloud.
        float r_point;

        // log-base-2 of the number of points in this tree.
        uint8_t nlog2;
    };  // namespace ompl::vamp::collision

}  // namespace ompl::vamp::collision
