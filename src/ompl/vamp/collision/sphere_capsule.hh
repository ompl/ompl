#pragma once

#include <ompl/vamp/collision/shapes.hh>
#include <ompl/vamp/collision/math.hh>

namespace ompl::vamp::collision
{
    template <typename DataT>
    inline constexpr auto sphere_capsule(
        const Capsule<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r) noexcept -> DataT
    {
        auto dot = dot_3(x - c.x1, y - c.y1, z - c.z1, c.xv, c.yv, c.zv);
        auto cdf = (dot * c.rdv).clamp(0.F, 1.F);

        auto sum = sql2_3(x, y, z, c.x1 + c.xv * cdf, c.y1 + c.yv * cdf, c.z1 + c.zv * cdf);
        auto rs = r + c.r;
        return sum - rs * rs;
    }

    template <typename DataT>
    inline constexpr auto sphere_capsule(const Capsule<DataT> &c, const Sphere<DataT> &s) noexcept -> DataT
    {
        return sphere_capsule(c, s.x, s.y, s.z, s.r);
    }

    template <typename DataT>
    inline constexpr auto sphere_z_aligned_capsule(
        const Capsule<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r) noexcept -> DataT
    {
        auto dot = (z - c.z1) * c.zv;
        auto cdf = (dot * c.rdv).clamp(0.F, 1.F);

        auto sum = sql2_3(x, y, z, c.x1, c.y1, c.z1 + c.zv * cdf);
        auto rs = r + c.r;
        return sum - rs * rs;
    }

    template <typename DataT>
    inline constexpr auto sphere_z_aligned_capsule(const Capsule<DataT> &c, const Sphere<DataT> &s) noexcept
        -> DataT
    {
        return sphere_z_aligned_capsule(c, s.x, s.y, s.z, s.r);
    }
}  // namespace ompl::vamp::collision
