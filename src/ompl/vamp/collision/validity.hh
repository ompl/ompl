#pragma once

#include <ompl/vamp/collision/shapes.hh>
#include <ompl/vamp/collision/environment.hh>
#include <ompl/vamp/collision/sphere_sphere.hh>
#include <ompl/vamp/collision/sphere_capsule.hh>
#include <ompl/vamp/collision/sphere_cuboid.hh>
#include <ompl/vamp/collision/sphere_heightfield.hh>
#include <ompl/vamp/collision/math.hh>

namespace ompl::vamp
{
    template <
        typename VectorDataT,
        typename ArgT1,
        typename ArgT2,
        typename ArgT3,
        typename ArgT4,
        typename ArgT5,
        typename ArgT6,
        typename ArgT7,
        typename ArgT8>
    inline constexpr auto sphere_sphere_self_collision(
        ArgT1 ax_,
        ArgT2 ay_,
        ArgT3 az_,
        ArgT4 ar_,
        ArgT5 bx_,
        ArgT6 by_,
        ArgT7 bz_,
        ArgT8 br_) noexcept -> bool
    {
        auto ax = static_cast<VectorDataT>(ax_);
        auto ay = static_cast<VectorDataT>(ay_);
        auto az = static_cast<VectorDataT>(az_);
        auto ar = static_cast<VectorDataT>(ar_);
        auto bx = static_cast<VectorDataT>(bx_);
        auto by = static_cast<VectorDataT>(by_);
        auto bz = static_cast<VectorDataT>(bz_);
        auto br = static_cast<VectorDataT>(br_);

        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        return not collision::sphere_sphere_sql2(ax, ay, az, ar, bx, by, bz, br).test_zero();
    }

    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline constexpr auto sphere_environment_in_collision(
        const collision::Environment<DataT> &e,  //
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> bool
    {
        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        const auto max_extent = collision::sqrt(collision::dot_3(sx, sy, sz, sx, sy, sz)) + sr;

        for (const auto &es : e.spheres)
        {
            const auto diff = es.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_sphere_sql2(es, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        for (const auto &ec : e.capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        for (const auto &ec : e.z_aligned_capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        const auto rsq = sr * sr;
        for (const auto &ec : e.cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                return true;
            }
        }

        for (const auto &ec : e.z_aligned_cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                return true;
            }
        }

        for (const auto &eh : e.heightfields)
        {
            if (not collision::sphere_heightfield(eh, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        const std::array<DataT, 3> positions = {sx, sy, sz};
        for (const auto &pc : e.pointclouds)
        {
            if (pc.collides_simd(positions, sr))
            {
                return true;
            }
        }

        return false;
    }

    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline auto sphere_environment_get_collisions(
        const collision::Environment<DataT> &e,  //
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> std::vector<std::string>
    {
        std::vector<std::string> objects;

        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        const auto max_extent = collision::sqrt(collision::dot_3(sx, sy, sz, sx, sy, sz)) + sr;

        for (const auto &es : e.spheres)
        {
            const auto diff = es.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_sphere_sql2(es, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(es.name);
            }
        }

        for (const auto &ec : e.capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        for (const auto &ec : e.z_aligned_capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        const auto rsq = sr * sr;
        for (const auto &ec : e.cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        for (const auto &ec : e.z_aligned_cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        for (const auto &eh : e.heightfields)
        {
            if (not collision::sphere_heightfield(eh, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(eh.name);
            }
        }

        return objects;
    }

    template <typename DataT>
    inline constexpr auto attachment_environment_collision(const collision::Environment<DataT> &e) noexcept
        -> bool
    {
        for (const auto &s : e.attachments->posed_spheres)
        {
            // HACK: The radius needs to be a float, and unfortunately the spheres assume homogeneous
            // DataT for storage
            // TODO: Fix the sphere representation to allow to store float radii even with vector
            // centers
            if (sphere_environment_in_collision(e, s.x, s.y, s.z, s.r[{0, 0}]))
            {
                return true;
            }
        }

        return false;
    }

    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline constexpr auto attachment_sphere_collision(
        const collision::Environment<DataT> &e,
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> bool
    {
        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        for (const auto &att_s : e.attachments->posed_spheres)
        {
            if (not collision::sphere_sphere_sql2(sx, sy, sz, sr, att_s.x, att_s.y, att_s.z, att_s.r)
                        .test_zero())
            {
                return true;
            }
        }

        return false;
    }
}  // namespace ompl::vamp
