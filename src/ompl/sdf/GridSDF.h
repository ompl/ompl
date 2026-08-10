#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
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

        /// Batched distance query for column-wise points (3 x N).
        void distanceBatch(const Eigen::Ref<const Eigen::Matrix3Xd> &points,
                           Eigen::Ref<Eigen::VectorXd> distances) const
        {
            const Eigen::Index count = points.cols();
            if (distances.size() != count)
                throw std::invalid_argument("ompl::sdf::GridSDF::distanceBatch: distances size mismatch");

            for (Eigen::Index i = 0; i < count; ++i)
                distances[i] = interpolate(points.col(i)).value;
        }

        /// Batched value and gradient query for column-wise points (3 x N).
        void valueGradientBatch(const Eigen::Ref<const Eigen::Matrix3Xd> &points,
                                Eigen::Ref<Eigen::VectorXd> distances,
                                Eigen::Ref<Eigen::Matrix3Xd> gradients) const
        {
            const Eigen::Index count = points.cols();
            if (distances.size() != count)
                throw std::invalid_argument("ompl::sdf::GridSDF::valueGradientBatch: distances size mismatch");
            if (gradients.cols() != count)
                throw std::invalid_argument("ompl::sdf::GridSDF::valueGradientBatch: gradients column mismatch");

            for (Eigen::Index i = 0; i < count; ++i)
            {
                const ValueGradient vg = interpolate(points.col(i));
                distances[i] = vg.value;
                gradients.col(i) = vg.gradient;
            }
        }

        auto inBounds(const Eigen::Vector3d &p) const -> bool
        {
            return bounds_.contains(p);
        }

        /// How far \p p may move in any direction and still be inside the baked box:
        /// the distance to the nearest face, negative when already outside.
        ///
        /// This is not a clearance from anything physical -- the box boundary is not an
        /// obstacle. It exists because a query outside the box is *clamped* and so
        /// over-reports clearance, which is the one direction a barrier cannot tolerate.
        /// Anything that certifies a whole segment of motion from one evaluation has to
        /// bound the excursion by this as well as by the clearance.
        auto boundaryClearance(const Eigen::Vector3d &p) const -> double
        {
            return (p - bounds_.min()).cwiseMin(bounds_.max() - p).minCoeff();
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

        /// Adopt a grid that was baked elsewhere.
        ///
        /// The baking constructor needs a `DistanceFn`, which a scene living in
        /// another process or another language cannot supply. This one takes the
        /// finished node values instead, so the bake can happen wherever the
        /// geometry actually is — a PyBullet scene, an offline tool — while the
        /// query path here stays byte-for-byte the same field.
        ///
        /// \p values holds one signed distance per node, x fastest then y then z,
        /// i.e. index `i + dims.x * (j + dims.y * k)`. Node (i,j,k) sits at
        /// `bounds.min() + (i,j,k) * spacing`, with `spacing = extent / (dims-1)`,
        /// which is the layout the baking constructor produces for a given voxel
        /// size. Use \ref gridDimensions to derive \p dims from a voxel size so the
        /// two agree by construction.
        GridSDF(const Eigen::AlignedBox3d &bounds, const Eigen::Vector3i &dims, std::vector<double> values)
          : bounds_(bounds), origin_(bounds.min()), dims_(dims), values_(std::move(values))
        {
            for (int d = 0; d < 3; ++d)
                if (dims_[d] < 2)
                    throw std::invalid_argument("ompl::sdf::GridSDF: each dimension needs >= 2 nodes");

            const std::size_t expected =
                static_cast<std::size_t>(dims_[0]) * dims_[1] * dims_[2];
            if (values_.size() != expected)
                throw std::invalid_argument("ompl::sdf::GridSDF: value count does not match dimensions");

            const Eigen::Vector3d extent = bounds_.max() - bounds_.min();
            for (int d = 0; d < 3; ++d)
                spacing_[d] = (extent[d] > 0.0) ? extent[d] / (dims_[d] - 1) : 0.0;

            computeLipschitzBound();
        }

        /// Node counts the baking constructor would choose for \p voxel. Exposed so
        /// an external baker can produce a grid this class will accept unchanged.
        static auto gridDimensions(const Eigen::AlignedBox3d &bounds, double voxel) -> Eigen::Vector3i
        {
            const Eigen::Vector3d extent = bounds.max() - bounds.min();
            Eigen::Vector3i dims;
            for (int d = 0; d < 3; ++d)
                dims[d] = std::max<int>(2, static_cast<int>(std::ceil(extent[d] / voxel)) + 1);
            return dims;
        }

        /// Byte layout of \ref save / \ref load. Little-endian, no padding:
        ///
        ///     char     magic[8]   "OMPLSDF1"
        ///     uint32   version    1
        ///     uint32   dims[3]
        ///     float64  min[3], max[3]
        ///     float64  values[dims.x * dims.y * dims.z]
        static constexpr char magic[8] = {'O', 'M', 'P', 'L', 'S', 'D', 'F', '1'};
        static constexpr std::uint32_t formatVersion = 1;

        /// Write the baked grid to \p path. Only the node values are stored; the
        /// gradient is recovered analytically from the interpolant on load, so a
        /// round trip reproduces the field exactly.
        void save(const std::string &path) const
        {
            std::ofstream out(path, std::ios::binary);
            if (!out)
                throw std::runtime_error("ompl::sdf::GridSDF::save: cannot open " + path);

            out.write(magic, sizeof(magic));
            const std::uint32_t version = formatVersion;
            out.write(reinterpret_cast<const char *>(&version), sizeof(version));
            for (int d = 0; d < 3; ++d)
            {
                const std::uint32_t n = static_cast<std::uint32_t>(dims_[d]);
                out.write(reinterpret_cast<const char *>(&n), sizeof(n));
            }
            for (int d = 0; d < 3; ++d)
            {
                const double v = bounds_.min()[d];
                out.write(reinterpret_cast<const char *>(&v), sizeof(v));
            }
            for (int d = 0; d < 3; ++d)
            {
                const double v = bounds_.max()[d];
                out.write(reinterpret_cast<const char *>(&v), sizeof(v));
            }
            out.write(reinterpret_cast<const char *>(values_.data()),
                      static_cast<std::streamsize>(values_.size() * sizeof(double)));
            if (!out)
                throw std::runtime_error("ompl::sdf::GridSDF::save: write failed for " + path);
        }

        /// Read a grid written by \ref save (or by an external baker following the
        /// layout above).
        static auto load(const std::string &path) -> GridSDF
        {
            std::ifstream in(path, std::ios::binary);
            if (!in)
                throw std::runtime_error("ompl::sdf::GridSDF::load: cannot open " + path);

            char tag[sizeof(magic)];
            in.read(tag, sizeof(tag));
            if (!in || std::memcmp(tag, magic, sizeof(magic)) != 0)
                throw std::runtime_error("ompl::sdf::GridSDF::load: not an SDF grid: " + path);

            std::uint32_t version = 0;
            in.read(reinterpret_cast<char *>(&version), sizeof(version));
            if (version != formatVersion)
                throw std::runtime_error("ompl::sdf::GridSDF::load: unsupported version in " + path);

            Eigen::Vector3i dims;
            for (int d = 0; d < 3; ++d)
            {
                std::uint32_t n = 0;
                in.read(reinterpret_cast<char *>(&n), sizeof(n));
                dims[d] = static_cast<int>(n);
            }

            Eigen::Vector3d lower;
            Eigen::Vector3d upper;
            in.read(reinterpret_cast<char *>(lower.data()), 3 * sizeof(double));
            in.read(reinterpret_cast<char *>(upper.data()), 3 * sizeof(double));
            if (!in)
                throw std::runtime_error("ompl::sdf::GridSDF::load: truncated header in " + path);

            const std::size_t total =
                static_cast<std::size_t>(dims[0]) * dims[1] * dims[2];
            std::vector<double> values(total);
            in.read(reinterpret_cast<char *>(values.data()),
                    static_cast<std::streamsize>(total * sizeof(double)));
            if (!in)
                throw std::runtime_error("ompl::sdf::GridSDF::load: truncated values in " + path);

            return GridSDF(Eigen::AlignedBox3d(lower, upper), dims, std::move(values));
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
