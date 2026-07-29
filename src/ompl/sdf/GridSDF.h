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
    /// node; node gradients follow from central differences over the baked
    /// values. After construction the grid is self-contained — it holds no
    /// reference to the source environment. Queries then cost O(1) via trilinear
    /// interpolation, independent of the environment's complexity.
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

            const std::size_t total = static_cast<std::size_t>(dims_[0]) * dims_[1] * dims_[2];
            values_.resize(total);
            gradients_.resize(total);

            for (int k = 0; k < dims_[2]; ++k)
                for (int j = 0; j < dims_[1]; ++j)
                    for (int i = 0; i < dims_[0]; ++i)
                        values_[index(i, j, k)] = distanceFn(nodePoint(i, j, k));

            computeGradients();
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
        /// almost everywhere -- but this field is central differences of a *sampled*
        /// distance function, blended trilinearly, and at a kink in the surface the three
        /// components can each approach 1 independently, putting the node gradient's norm
        /// above 1. Anything relying on `|grad d| <= L` for soundness (a Lipschitz bound
        /// on how fast a barrier can fall, say) must use this measured value rather than
        /// assume 1. An interpolated gradient is a convex combination of node gradients,
        /// so bounding the nodes bounds the field.
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

        /// Node gradients by central differences over the baked value grid
        /// (one-sided at the boundaries).
        void computeGradients()
        {
            const std::size_t stride[3] = {
                1u,
                static_cast<std::size_t>(dims_[0]),
                static_cast<std::size_t>(dims_[0]) * dims_[1]};

            for (int k = 0; k < dims_[2]; ++k)
                for (int j = 0; j < dims_[1]; ++j)
                    for (int i = 0; i < dims_[0]; ++i)
                    {
                        const std::size_t idx = index(i, j, k);
                        const int coord[3] = {i, j, k};
                        Eigen::Vector3d g = Eigen::Vector3d::Zero();
                        for (int d = 0; d < 3; ++d)
                        {
                            if (dims_[d] < 2 || spacing_[d] <= 0.0)
                                continue;
                            const std::size_t st = stride[d];
                            if (coord[d] == 0)
                                g[d] = (values_[idx + st] - values_[idx]) / spacing_[d];
                            else if (coord[d] == dims_[d] - 1)
                                g[d] = (values_[idx] - values_[idx - st]) / spacing_[d];
                            else
                                g[d] = (values_[idx + st] - values_[idx - st]) / (2.0 * spacing_[d]);
                        }
                        gradients_[idx] = g;
                        maxGradientNorm_ = std::max(maxGradientNorm_, g.norm());
                    }
        }

        /// Trilinear interpolation of value and gradient. Points outside the grid
        /// are clamped to the boundary (nearest-node extrapolation).
        auto interpolate(const Eigen::Vector3d &p) const -> ValueGradient
        {
            int i0[3];
            double f[3];
            for (int d = 0; d < 3; ++d)
            {
                double c = (spacing_[d] > 0.0) ? (p[d] - origin_[d]) / spacing_[d] : 0.0;
                c = std::clamp(c, 0.0, static_cast<double>(dims_[d] - 1));
                i0[d] = std::clamp(static_cast<int>(std::floor(c)), 0, dims_[d] - 2);
                f[d] = (dims_[d] > 1) ? c - i0[d] : 0.0;
            }

            ValueGradient out;
            for (int dk = 0; dk < 2; ++dk)
                for (int dj = 0; dj < 2; ++dj)
                    for (int di = 0; di < 2; ++di)
                    {
                        const double w =
                            (di ? f[0] : 1.0 - f[0]) * (dj ? f[1] : 1.0 - f[1]) * (dk ? f[2] : 1.0 - f[2]);
                        const std::size_t idx = index(i0[0] + di, i0[1] + dj, i0[2] + dk);
                        out.value += w * values_[idx];
                        out.gradient += w * gradients_[idx];
                    }
            return out;
        }

        Eigen::AlignedBox3d bounds_;
        Eigen::Vector3d origin_;
        Eigen::Vector3i dims_{2, 2, 2};
        Eigen::Vector3d spacing_{0.0, 0.0, 0.0};
        double maxGradientNorm_{0.0};
        std::vector<double> values_;
        std::vector<Eigen::Vector3d> gradients_;
    };
}  // namespace ompl::sdf
