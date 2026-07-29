#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ompl::sdf
{
    /// A signed distance value together with its spatial gradient at a query
    /// point. This is exactly the pair a Control Barrier Function needs: the
    /// barrier value h(x) and its derivative dh/dx.
    struct ValueGradient
    {
        double value{0.0};
        Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
    };

    /// The one thing GridSDF needs from an environment: the signed distance from
    /// a workspace point to the nearest obstacle surface (negative inside).
    /// Any obstacle representation that can answer this — analytic primitives, a
    /// mesh/FCL signed-distance query, another distance field — supplies it, so
    /// the cache is completely environment-agnostic.
    using DistanceFn = std::function<double(const Eigen::Vector3d &)>;

    /// A workspace signed distance field cached on a regular voxel grid.
    ///
    /// The grid is baked once, at construction, by sampling a DistanceFn at each
    /// node. After construction the grid is self-contained — it holds no
    /// reference to the source environment. Queries then cost O(1) via trilinear
    /// interpolation, independent of the environment's complexity; the gradient
    /// is the exact derivative of that same interpolant, so the value and the
    /// gradient always describe one consistent field.
    class GridSDF
    {
    public:
        /// Bake \p distanceFn over \p bounds using (approximately) cubic voxels of
        /// edge length \p voxel.
        GridSDF(const DistanceFn &distanceFn, const Eigen::AlignedBox3d &bounds, double voxel)
          : bounds_(bounds), origin_(bounds.min())
        {
            const Eigen::Vector3d extent = bounds.max() - bounds.min();
            for (int d = 0; d < 3; ++d)
            {
                dims_[d] = std::max<int>(2, static_cast<int>(std::ceil(extent[d] / voxel)) + 1);
                spacing_[d] = (extent[d] > 0.0) ? extent[d] / (dims_[d] - 1) : 0.0;
            }

        const std::size_t total =
            static_cast<std::size_t>(dims_[0]) * dims_[1] * dims_[2];

        values_.resize(total);

        for (int k = 0; k < dims_[2]; ++k)
            for (int j = 0; j < dims_[1]; ++j)
                for (int i = 0; i < dims_[0]; ++i)
                    values_[index(i, j, k)] =
                        distanceFn(nodePoint(i, j, k));

        computeLipschitzBound();
        }

        auto distance(const Eigen::Vector3d &p) const -> double
        {
            return interpolate(p).value;
        }

        auto gradient(const Eigen::Vector3d &p) const -> Eigen::Vector3d
        {
            return interpolate(p).gradient;
        }

        auto valueAndGradient(const Eigen::Vector3d &p) const -> ValueGradient
        {
            return interpolate(p);
        }

        auto inBounds(const Eigen::Vector3d &p) const -> bool
        {
            return bounds_.contains(p);
        }

        auto bounds() const -> const Eigen::AlignedBox3d &
        {
            return bounds_;
        }

        auto dimensions() const -> Eigen::Vector3i
        {
            return dims_;
        }

        auto spacing() const -> const Eigen::Vector3d &
        {
            return spacing_;
        }

        /// The largest gradient magnitude anywhere in the interpolated field.
        ///
        /// A true signed distance field is 1-Lipschitz, so its gradient has unit norm
        /// almost everywhere -- but this field is the trilinear interpolant of a *sampled*
        /// distance function, and within one cell the three axis derivatives are bounded by
        /// slopes measured along different edges, so they can each approach 1 without ever
        /// doing so at the same point. That puts the bound above 1. Anything relying on
        /// `|grad d| <= L` for soundness (a Lipschitz bound on how fast a barrier can fall,
        /// say) must use this measured value rather than assume 1.
        auto maxGradientNorm() const -> double
        {
            return maxGradientNorm_;
        }

    private:
        auto index(int i, int j, int k) const -> std::size_t
        {
            return static_cast<std::size_t>(i) +
                   static_cast<std::size_t>(dims_[0]) * (j + static_cast<std::size_t>(dims_[1]) * k);
        }

        auto nodePoint(int i, int j, int k) const -> Eigen::Vector3d
        {
            return origin_ + Eigen::Vector3d(i * spacing_[0], j * spacing_[1], k * spacing_[2]);
        }

        /// An upper bound on `|grad d|` over the interpolated field, measured cell by cell.
        ///
        /// Inside a cell the interpolant's x-derivative is a convex combination (weights
        /// `wy * wz`) of the slopes along the cell's four x-edges, so the largest of those
        /// four slopes bounds it; likewise for y and z. Combining the three per-axis bounds
        /// bounds the gradient norm on that cell, and the max over cells bounds the field.
     void computeLipschitzBound()
        {
            maxGradientNorm_ = 0.0;

            for (int k = 0; k < dims_[2] - 1; ++k)
            {
                for (int j = 0; j < dims_[1] - 1; ++j)
                {
                    for (int i = 0; i < dims_[0] - 1; ++i)
                    {
                        double maxDx = 0.0;
                        double maxDy = 0.0;
                        double maxDz = 0.0;

                        if (spacing_[0] > 0.0)
                        {
                            for (int dk = 0; dk < 2; ++dk)
                            {
                                for (int dj = 0; dj < 2; ++dj)
                                {
                                    const double slope =
                                        std::abs(
                                            values_[index(i + 1, j + dj, k + dk)] -
                                            values_[index(i, j + dj, k + dk)]) /
                                        spacing_[0];

                                    maxDx = std::max(maxDx, slope);
                                }
                            }
                        }

                        if (spacing_[1] > 0.0)
                        {
                            for (int dk = 0; dk < 2; ++dk)
                            {
                                for (int di = 0; di < 2; ++di)
                                {
                                    const double slope =
                                        std::abs(
                                            values_[index(i + di, j + 1, k + dk)] -
                                            values_[index(i + di, j, k + dk)]) /
                                        spacing_[1];

                                    maxDy = std::max(maxDy, slope);
                                }
                            }
                        }

                        if (spacing_[2] > 0.0)
                        {
                            for (int dj = 0; dj < 2; ++dj)
                            {
                                for (int di = 0; di < 2; ++di)
                                {
                                    const double slope =
                                        std::abs(
                                            values_[index(i + di, j + dj, k + 1)] -
                                            values_[index(i + di, j + dj, k)]) /
                                        spacing_[2];

                                    maxDz = std::max(maxDz, slope);
                                }
                            }
                        }

                        const double cellBound =
                            std::sqrt(
                                maxDx * maxDx +
                                maxDy * maxDy +
                                maxDz * maxDz);

                        maxGradientNorm_ =
                            std::max(maxGradientNorm_, cellBound);
                    }
                }
            }
        }

        /// Trilinear interpolation of value and gradient. Points outside the grid
        /// are clamped to the boundary (nearest-node extrapolation).
    auto interpolate(const Eigen::Vector3d &p) const -> ValueGradient
    {
        int i0[3];
        double f[3];
        bool derivativeActive[3];

        for (int d = 0; d < 3; ++d)
        {
            const double raw =
                spacing_[d] > 0.0 ?
                    (p[d] - origin_[d]) / spacing_[d] :
                    0.0;

            derivativeActive[d] =
                raw >= 0.0 &&
                raw <= static_cast<double>(dims_[d] - 1);

            const double c = std::clamp(
                raw,
                0.0,
                static_cast<double>(dims_[d] - 1));

            i0[d] = std::clamp(
                static_cast<int>(std::floor(c)),
                0,
                dims_[d] - 2);

            f[d] = c - static_cast<double>(i0[d]);
        }

        const double wx[2] = {1.0 - f[0], f[0]};
        const double wy[2] = {1.0 - f[1], f[1]};
        const double wz[2] = {1.0 - f[2], f[2]};

        ValueGradient out;

        // Trilinear scalar value.
        for (int dk = 0; dk < 2; ++dk)
        {
            for (int dj = 0; dj < 2; ++dj)
            {
                for (int di = 0; di < 2; ++di)
                {
                    const double weight =
                        wx[di] * wy[dj] * wz[dk];

                    out.value +=
                        weight *
                        values_[index(
                            i0[0] + di,
                            i0[1] + dj,
                            i0[2] + dk)];
                }
            }
        }

        // Exact derivative of the same trilinear scalar field.
        if (spacing_[0] > 0.0 && derivativeActive[0])
        {
            for (int dk = 0; dk < 2; ++dk)
            {
                for (int dj = 0; dj < 2; ++dj)
                {
                    const double v0 =
                        values_[index(
                            i0[0],
                            i0[1] + dj,
                            i0[2] + dk)];

                    const double v1 =
                        values_[index(
                            i0[0] + 1,
                            i0[1] + dj,
                            i0[2] + dk)];

                    out.gradient[0] +=
                        wy[dj] * wz[dk] *
                        (v1 - v0) / spacing_[0];
                }
            }
        }

        if (spacing_[1] > 0.0 && derivativeActive[1])
        {
            for (int dk = 0; dk < 2; ++dk)
            {
                for (int di = 0; di < 2; ++di)
                {
                    const double v0 =
                        values_[index(
                            i0[0] + di,
                            i0[1],
                            i0[2] + dk)];

                    const double v1 =
                        values_[index(
                            i0[0] + di,
                            i0[1] + 1,
                            i0[2] + dk)];

                    out.gradient[1] +=
                        wx[di] * wz[dk] *
                        (v1 - v0) / spacing_[1];
                }
            }
        }

        if (spacing_[2] > 0.0 && derivativeActive[2])
        {
            for (int dj = 0; dj < 2; ++dj)
            {
                for (int di = 0; di < 2; ++di)
                {
                    const double v0 =
                        values_[index(
                            i0[0] + di,
                            i0[1] + dj,
                            i0[2])];

                    const double v1 =
                        values_[index(
                            i0[0] + di,
                            i0[1] + dj,
                            i0[2] + 1)];

                    out.gradient[2] +=
                        wx[di] * wy[dj] *
                        (v1 - v0) / spacing_[2];
                }
            }
        }

        return out;
    }
        Eigen::AlignedBox3d bounds_;
        Eigen::Vector3d origin_;
        Eigen::Vector3i dims_{2, 2, 2};
        Eigen::Vector3d spacing_{0.0, 0.0, 0.0};
        double maxGradientNorm_{0.0};
        std::vector<double> values_;
    };
}  // namespace ompl::sdf
