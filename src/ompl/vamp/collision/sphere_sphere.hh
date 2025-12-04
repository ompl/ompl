#pragma once

#include <ompl/vamp/collision/shapes.hh>
#include <ompl/vamp/collision/math.hh>

namespace ompl::vamp::collision
{

    template <typename DataT>
    inline constexpr auto sphere_sphere_sql2(
        const DataT &ax,
        const DataT &ay,
        const DataT &az,
        const DataT &ar,
        const DataT &bx,
        const DataT &by,
        const DataT &bz,
        const DataT &br) noexcept -> DataT
    {
        auto sum = sql2_3(ax, ay, az, bx, by, bz);
        auto rs = ar + br;
        return sum - rs * rs;
    }

    template <typename DataT>
    inline constexpr auto sphere_sphere_sql2(
        const Sphere<DataT> &a,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r) noexcept -> DataT
    {
        return sphere_sphere_sql2(a.x, a.y, a.z, a.r, x, y, z, r);
    }

    template <typename DataT>
    inline constexpr auto sphere_sphere_sql2(const Sphere<DataT> &a, const Sphere<DataT> &b) noexcept -> DataT
    {
        return sphere_sphere_sql2(a, b.x, b.y, b.z, b.r);
    }

    template <typename DataT>
    inline constexpr auto sphere_sphere_l2(const Sphere<DataT> &a, const Sphere<DataT> &b) noexcept -> DataT
    {
        auto sum = sql2_3(a.x, a.y, a.z, b.x, b.y, b.z).sqrt();
        return sum - (a.r + b.r);
    }
}  // namespace ompl::vamp::collision
