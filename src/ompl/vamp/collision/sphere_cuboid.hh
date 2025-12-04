#pragma once

#include <ompl/vamp/collision/shapes.hh>
#include <ompl/vamp/collision/math.hh>

namespace ompl::vamp::collision
{
    template <typename DataT>
    inline constexpr auto sphere_cuboid(
        const Cuboid<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &rsq) noexcept -> DataT
    {
        auto xs = x - c.x;
        auto ys = y - c.y;
        auto zs = z - c.z;

        auto a1 = (dot_3(c.axis_1_x, c.axis_1_y, c.axis_1_z, xs, ys, zs).abs() - c.axis_1_r).max(0.);
        auto a2 = (dot_3(c.axis_2_x, c.axis_2_y, c.axis_2_z, xs, ys, zs).abs() - c.axis_2_r).max(0.);
        auto a3 = (dot_3(c.axis_3_x, c.axis_3_y, c.axis_3_z, xs, ys, zs).abs() - c.axis_3_r).max(0.);

        auto sum = dot_3(a1, a2, a3, a1, a2, a3);
        return sum - rsq;
    }

    template <typename DataT>
    inline constexpr auto sphere_cuboid(const Cuboid<DataT> &c, const Sphere<DataT> &s) noexcept -> DataT
    {
        return sphere_cuboid(c, s.x, s.y, s.z, s.r * s.r);
    }

    template <typename DataT>
    inline constexpr auto sphere_z_aligned_cuboid(
        const Cuboid<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &rsq) noexcept -> DataT
    {
        auto xs = x - c.x;
        auto ys = y - c.y;
        auto zs = z - c.z;

        auto a1 = (dot_2(c.axis_1_x, c.axis_1_y, xs, ys).abs() - c.axis_1_r).max(0.);
        auto a2 = (dot_2(c.axis_2_x, c.axis_2_y, xs, ys).abs() - c.axis_2_r).max(0.);
        auto a3 = (zs.abs() - c.axis_3_r).max(0.);

        auto sum = dot_3(a1, a2, a3, a1, a2, a3);
        return sum - rsq;
    }

    template <typename DataT>
    inline constexpr auto sphere_z_aligned_cuboid(const Cuboid<DataT> &c, const Sphere<DataT> &s) noexcept
        -> DataT
    {
        return sphere_z_aligned_cuboid(c, s.x, s.y, s.z, s.r * s.r);
    }
}  // namespace ompl::vamp::collision
