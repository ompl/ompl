#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/sdf/GridSDF.h>

namespace ompl::sdf
{
    /// A point-cloud environment you can turn straight into an SDF.
    ///
    /// Load a cloud from a file (`PointCloud::load("scene.ply")`), then bake it
    /// into a GridSDF (`cloud.bake(voxel)`), or do both in one call with
    /// `sdfFromFile(...)`. The distance at a workspace point is the distance to
    /// the nearest cloud point, minus an inflation `radius` (treat each point as
    /// a small sphere). A uniform spatial hash makes nearest-point queries fast
    /// enough to bake a full grid.
    ///
    /// Note on sign: a raw point cloud only samples obstacle *surfaces*, so this
    /// is really an unsigned distance-to-cloud offset by `radius` — it is
    /// negative only within `radius` of a measured point, not throughout a solid
    /// interior. That is exactly what a clearance/CBF filter wants (keep the
    /// robot `radius` away from every observed point); it is not a watertight
    /// inside/outside field (which points alone can't provide without normals).
    class PointCloud
    {
    public:
        explicit PointCloud(std::vector<Eigen::Vector3d> points, double radius = 0.0)
          : points_(std::move(points)), radius_(radius)
        {
            build();
        }

        /// Load a cloud from a .ply (ascii or binary_little_endian) or a
        /// whitespace-separated .xyz/.txt/.pts file (first three columns = x y z).
        static auto load(const std::string &path, double radius = 0.0) -> PointCloud
        {
            const auto dot = path.find_last_of('.');
            std::string ext = (dot == std::string::npos) ? "" : path.substr(dot + 1);
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
            return PointCloud(ext == "ply" ? readPLY(path) : readXYZ(path), radius);
        }

        void setRadius(double radius)
        {
            radius_ = radius;
        }

        auto radius() const -> double
        {
            return radius_;
        }

        auto size() const -> std::size_t
        {
            return points_.size();
        }

        auto points() const -> const std::vector<Eigen::Vector3d> &
        {
            return points_;
        }

        /// Axis-aligned bounding box of the cloud.
        auto aabb() const -> Eigen::AlignedBox3d
        {
            return aabb_;
        }

        /// Signed distance: distance to nearest point minus the inflation radius.
        auto operator()(const Eigen::Vector3d &p) const -> double
        {
            return std::sqrt(nearestSquared(p)) - radius_;
        }

        auto distance(const Eigen::Vector3d &p) const -> double
        {
            return (*this)(p);
        }

        /// Bake over an explicit box.
        auto bake(const Eigen::AlignedBox3d &bounds, double voxel) const -> GridSDF
        {
            return GridSDF(*this, bounds, voxel);
        }

        /// Bake over the cloud's own bounding box, padded on every side.
        auto bake(double voxel, double padding = 0.1) const -> GridSDF
        {
            Eigen::AlignedBox3d b = aabb_;
            b.min().array() -= padding;
            b.max().array() += padding;
            return GridSDF(*this, b, voxel);
        }

    private:
        void build()
        {
            aabb_.setEmpty();
            for (const auto &p : points_)
                aabb_.extend(p);

            origin_ = aabb_.min();
            const Eigen::Vector3d extent = aabb_.sizes();
            const double vol = extent.prod();
            const std::size_t n = std::max<std::size_t>(points_.size(), 1);
            // Aim for ~1 point per cell; fall back gracefully for flat clouds.
            double h = (vol > 0.0) ? std::cbrt(vol / n) : extent.maxCoeff() / 64.0;
            if (!(h > 0.0) || !std::isfinite(h))
                h = 1.0;
            cell_ = std::max(h, extent.maxCoeff() / 512.0 + 1e-9);
            span_ = static_cast<int>(std::ceil(extent.maxCoeff() / cell_)) + 2;

            for (int i = 0; i < static_cast<int>(points_.size()); ++i)
                buckets_[key(cellCoord(points_[i]))].push_back(i);
        }

        auto cellCoord(const Eigen::Vector3d &p) const -> Eigen::Vector3i
        {
            const Eigen::Vector3d c = (p - origin_) / cell_;
            return Eigen::Vector3i(
                static_cast<int>(std::floor(c.x())),
                static_cast<int>(std::floor(c.y())),
                static_cast<int>(std::floor(c.z())));
        }

        static auto key(const Eigen::Vector3i &c) -> std::uint64_t
        {
            constexpr std::int64_t M = 1 << 20;  // supports 2^21 cells / axis
            const std::uint64_t x = static_cast<std::uint64_t>(c.x() + M) & 0x1FFFFF;
            const std::uint64_t y = static_cast<std::uint64_t>(c.y() + M) & 0x1FFFFF;
            const std::uint64_t z = static_cast<std::uint64_t>(c.z() + M) & 0x1FFFFF;
            return x | (y << 21) | (z << 42);
        }

        /// Nearest-point squared distance via expanding cubic shells of cells.
        auto nearestSquared(const Eigen::Vector3d &p) const -> double
        {
            if (points_.empty())
                return std::numeric_limits<double>::max();

            const Eigen::Vector3i c = cellCoord(p);
            double best2 = std::numeric_limits<double>::max();

            for (int R = 0; R <= span_; ++R)
            {
                // Lower bound on any point in shells >= R (p lies inside cell c).
                if (R > 0)
                {
                    const double lb = (R - 1) * cell_;
                    if (best2 < std::numeric_limits<double>::max() && lb * lb >= best2)
                        break;
                }

                for (int dx = -R; dx <= R; ++dx)
                    for (int dy = -R; dy <= R; ++dy)
                        for (int dz = -R; dz <= R; ++dz)
                        {
                            if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != R)
                                continue;  // shell surface only
                            const auto it = buckets_.find(key(c + Eigen::Vector3i(dx, dy, dz)));
                            if (it == buckets_.end())
                                continue;
                            for (int idx : it->second)
                                best2 = std::min(best2, (p - points_[idx]).squaredNorm());
                        }
            }
            return best2;
        }

        // ---- File readers -------------------------------------------------
        static auto readXYZ(const std::string &path) -> std::vector<Eigen::Vector3d>
        {
            std::ifstream in(path);
            if (!in)
                throw std::runtime_error("PointCloud: cannot open " + path);
            std::vector<Eigen::Vector3d> pts;
            std::string line;
            while (std::getline(in, line))
            {
                if (line.empty() || line[0] == '#')
                    continue;
                std::istringstream ss(line);
                double x, y, z;
                if (ss >> x >> y >> z)
                    pts.emplace_back(x, y, z);
            }
            return pts;
        }

        static auto readPLY(const std::string &path) -> std::vector<Eigen::Vector3d>
        {
            std::ifstream in(path, std::ios::binary);
            if (!in)
                throw std::runtime_error("PointCloud: cannot open " + path);

            std::string line, format;
            std::size_t vertexCount = 0;
            bool inVertex = false;
            std::vector<std::pair<std::string, int>> props;  // (name, byte size) for vertex element
            auto typeSize = [](const std::string &t) -> int
            {
                if (t == "char" || t == "uchar" || t == "int8" || t == "uint8")
                    return 1;
                if (t == "short" || t == "ushort" || t == "int16" || t == "uint16")
                    return 2;
                if (t == "int" || t == "uint" || t == "int32" || t == "uint32" || t == "float" || t == "float32")
                    return 4;
                if (t == "double" || t == "float64")
                    return 8;
                return 4;
            };

            while (std::getline(in, line))
            {
                if (!line.empty() && line.back() == '\r')
                    line.pop_back();
                std::istringstream ss(line);
                std::string tok;
                ss >> tok;
                if (tok == "format")
                    ss >> format;
                else if (tok == "element")
                {
                    std::string name;
                    ss >> name;
                    inVertex = (name == "vertex");
                    if (inVertex)
                        ss >> vertexCount;
                }
                else if (tok == "property" && inVertex)
                {
                    std::string t, name;
                    ss >> t;
                    if (t == "list")  // skip list properties (e.g. faces) — not in vertex normally
                    {
                        std::string t2, t3;
                        ss >> t2 >> t3 >> name;
                        props.emplace_back(name, -1);
                    }
                    else
                    {
                        ss >> name;
                        props.emplace_back(name, typeSize(t));
                    }
                }
                else if (tok == "end_header")
                    break;
            }

            int ix = -1, iy = -1, iz = -1;
            for (int i = 0; i < static_cast<int>(props.size()); ++i)
            {
                if (props[i].first == "x")
                    ix = i;
                else if (props[i].first == "y")
                    iy = i;
                else if (props[i].first == "z")
                    iz = i;
            }
            if (ix < 0 || iy < 0 || iz < 0)
                throw std::runtime_error("PointCloud: PLY has no x/y/z vertex properties");

            std::vector<Eigen::Vector3d> pts;
            pts.reserve(vertexCount);

            if (format.rfind("ascii", 0) == 0)
            {
                for (std::size_t v = 0; v < vertexCount && std::getline(in, line); ++v)
                {
                    std::istringstream ss(line);
                    std::vector<double> vals;
                    double d;
                    while (ss >> d)
                        vals.push_back(d);
                    if (static_cast<int>(vals.size()) > std::max({ix, iy, iz}))
                        pts.emplace_back(vals[ix], vals[iy], vals[iz]);
                }
            }
            else  // binary_little_endian
            {
                std::vector<int> off(props.size());
                int stride = 0;
                for (std::size_t i = 0; i < props.size(); ++i)
                {
                    off[i] = stride;
                    stride += std::max(props[i].second, 0);
                }
                std::vector<char> buf(stride);
                auto readAt = [&](const char *base, int o, int sz) -> double
                {
                    if (sz == 4)
                    {
                        float f;
                        std::memcpy(&f, base + o, 4);
                        return f;
                    }
                    if (sz == 8)
                    {
                        double dd;
                        std::memcpy(&dd, base + o, 8);
                        return dd;
                    }
                    return 0.0;  // integer coord types are unusual; treat as 0
                };
                for (std::size_t v = 0; v < vertexCount; ++v)
                {
                    if (!in.read(buf.data(), stride))
                        break;
                    pts.emplace_back(
                        readAt(buf.data(), off[ix], props[ix].second),
                        readAt(buf.data(), off[iy], props[iy].second),
                        readAt(buf.data(), off[iz], props[iz].second));
                }
            }
            return pts;
        }

        std::vector<Eigen::Vector3d> points_;
        double radius_;
        Eigen::AlignedBox3d aabb_;
        Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
        double cell_{1.0};
        int span_{1};
        std::unordered_map<std::uint64_t, std::vector<int>> buckets_;
    };

    /// One-call convenience: load a point-cloud file and return its cached SDF,
    /// baked over the cloud's own (padded) bounding box.
    inline auto sdfFromFile(const std::string &path, double voxel, double radius = 0.0, double padding = 0.1)
        -> GridSDF
    {
        return PointCloud::load(path, radius).bake(voxel, padding);
    }
}  // namespace ompl::sdf
