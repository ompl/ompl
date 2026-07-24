#pragma once

// Mesh / FCL adapter: turn a triangle-mesh environment into an ompl::sdf SDF.
//
// Requires FCL (>= 0.6) and Assimp, and linking against them:
//   pkg-config --cflags --libs fcl assimp
// This header is opt-in — including it pulls in FCL/Assimp; the rest of the SDF
// module has no such dependency.
//
// Distance magnitude comes from FCL's exact primitive-vs-BVH distance query;
// the sign comes from a ray-parity (even/odd crossings) point-in-mesh test over
// the loaded triangles. For a watertight mesh this yields a genuine *filled*
// signed field (negative inside the solid), not just distance-to-surface.

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fcl/fcl.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <ompl/sdf/GridSDF.h>

namespace ompl::sdf
{
    /// A triangle-mesh environment exposed as a DistanceFn (and bakeable into a
    /// GridSDF). Load one or more mesh files; FCL provides the distance, a
    /// ray-parity test provides the inside/outside sign.
    class MeshField
    {
    public:
        using BVH = fcl::BVHModel<fcl::OBBRSS<double>>;

        explicit MeshField(double probeRadius = 1e-4) : probeRadius_(probeRadius)
        {
            // A tiny tetrahedron probe (mesh), centered at the origin. FCL's
            // mesh-vs-mesh distance is reliable, whereas primitive-vs-BVH
            // distance is not; we translate this probe to the query point.
            const double r = probeRadius_ / std::sqrt(3.0);
            const fcl::Vector3<double> t0(r, r, r), t1(r, -r, -r), t2(-r, r, -r), t3(-r, -r, r);
            probe_ = std::make_shared<BVH>();
            probe_->beginModel();
            probe_->addTriangle(t0, t1, t2);
            probe_->addTriangle(t0, t1, t3);
            probe_->addTriangle(t0, t2, t3);
            probe_->addTriangle(t1, t2, t3);
            probe_->endModel();
            probe_->computeLocalAABB();
        }

        /// Load a mesh file (OBJ/STL/PLY/DAE/… via Assimp) into the environment.
        /// An optional world transform places the mesh.
        auto addMesh(const std::string &path, const Eigen::Isometry3d &tf = Eigen::Isometry3d::Identity())
            -> MeshField &
        {
            Assimp::Importer importer;
            const aiScene *scene = importer.ReadFile(
                path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_PreTransformVertices);
            if (scene == nullptr || scene->mNumMeshes == 0)
                throw std::runtime_error("MeshField: failed to load mesh " + path + ": " + importer.GetErrorString());

            for (unsigned m = 0; m < scene->mNumMeshes; ++m)
            {
                const aiMesh *mesh = scene->mMeshes[m];
                std::vector<Eigen::Vector3d> v(mesh->mNumVertices);
                for (unsigned i = 0; i < mesh->mNumVertices; ++i)
                {
                    const auto &a = mesh->mVertices[i];
                    v[i] = tf * Eigen::Vector3d(a.x, a.y, a.z);
                }
                for (unsigned f = 0; f < mesh->mNumFaces; ++f)
                {
                    const aiFace &face = mesh->mFaces[f];
                    if (face.mNumIndices != 3)
                        continue;
                    const Eigen::Vector3d &p0 = v[face.mIndices[0]];
                    const Eigen::Vector3d &p1 = v[face.mIndices[1]];
                    const Eigen::Vector3d &p2 = v[face.mIndices[2]];
                    triangles_.push_back({p0, p1, p2});
                    for (const auto &pt : {p0, p1, p2})
                        aabb_.extend(pt);
                }
            }
            rebuild();
            return *this;
        }

        static auto load(const std::string &path, double probeRadius = 1e-3) -> MeshField
        {
            MeshField f(probeRadius);
            f.addMesh(path);
            return f;
        }

        auto aabb() const -> Eigen::AlignedBox3d
        {
            return aabb_;
        }

        auto triangleCount() const -> std::size_t
        {
            return triangles_.size();
        }

        /// Signed distance: |FCL distance to surface| with sign from a
        /// point-in-mesh parity test (negative inside a watertight mesh).
        auto operator()(const Eigen::Vector3d &p) const -> double
        {
            fcl::Transform3<double> tf = fcl::Transform3<double>::Identity();
            tf.translation() = p;
            fcl::CollisionObject<double> probe(probe_, tf);

            // Unsigned mesh-to-mesh separation (magnitude). We supply the sign
            // ourselves via the inside() parity test.
            fcl::DistanceRequest<double> req;
            req.enable_nearest_points = false;
            req.gjk_solver_type = fcl::GST_INDEP;
            fcl::DistanceResult<double> res;
            fcl::distance(&probe, object_.get(), req, res);

            // Separated: min_distance is the (tiny) probe-to-surface gap ~ point
            // distance. Overlapping the surface: negative sentinel -> ~0.
            const double surface = (res.min_distance < 0.0) ? 0.0 : res.min_distance;
            return inside(p) ? -surface : surface;
        }

        auto distance(const Eigen::Vector3d &p) const -> double
        {
            return (*this)(p);
        }

        auto bake(const Eigen::AlignedBox3d &bounds, double voxel) const -> GridSDF
        {
            return GridSDF(*this, bounds, voxel);
        }

        auto bake(double voxel, double padding = 0.1) const -> GridSDF
        {
            Eigen::AlignedBox3d b = aabb_;
            b.min().array() -= padding;
            b.max().array() += padding;
            return GridSDF(*this, b, voxel);
        }

    private:
        void rebuild()
        {
            auto geom = std::make_shared<BVH>();
            geom->beginModel();
            for (const auto &t : triangles_)
                geom->addTriangle(t[0], t[1], t[2]);
            geom->endModel();
            geom->computeLocalAABB();
            object_ = std::make_shared<fcl::CollisionObject<double>>(geom, fcl::Transform3<double>::Identity());
        }

        /// Point-in-mesh via Möller–Trumbore ray casting with parity counting.
        /// Uses a non-axis-aligned ray to avoid coplanar/edge degeneracies.
        auto inside(const Eigen::Vector3d &p) const -> bool
        {
            const Eigen::Vector3d dir = Eigen::Vector3d(0.5773503, 0.5773503, 0.5773503);
            int crossings = 0;
            constexpr double eps = 1e-9;
            for (const auto &t : triangles_)
            {
                const Eigen::Vector3d e1 = t[1] - t[0];
                const Eigen::Vector3d e2 = t[2] - t[0];
                const Eigen::Vector3d pv = dir.cross(e2);
                const double det = e1.dot(pv);
                if (std::abs(det) < eps)
                    continue;
                const double inv = 1.0 / det;
                const Eigen::Vector3d tv = p - t[0];
                const double u = tv.dot(pv) * inv;
                if (u < 0.0 || u > 1.0)
                    continue;
                const Eigen::Vector3d qv = tv.cross(e1);
                const double v = dir.dot(qv) * inv;
                if (v < 0.0 || u + v > 1.0)
                    continue;
                const double s = e2.dot(qv) * inv;
                if (s > eps)
                    ++crossings;
            }
            return (crossings & 1) != 0;
        }

        double probeRadius_;
        std::shared_ptr<BVH> probe_;
        std::shared_ptr<fcl::CollisionObject<double>> object_;
        std::vector<std::array<Eigen::Vector3d, 3>> triangles_;
        Eigen::AlignedBox3d aabb_;
    };

    /// One-call convenience: load a mesh file and return its cached SDF, baked
    /// over the mesh's own (padded) bounding box.
    inline auto sdfFromMeshFile(const std::string &path, double voxel, double padding = 0.1, double probeRadius = 1e-3)
        -> GridSDF
    {
        return MeshField::load(path, probeRadius).bake(voxel, padding);
    }
}  // namespace ompl::sdf
