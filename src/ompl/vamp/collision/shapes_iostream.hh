#pragma once

#include <iostream>
#include <ompl/vamp/collision/shapes.hh>

namespace ompl::vamp::collision
{
    template <typename DataT>
    auto operator<<(std::ostream &o, const Cuboid<DataT> &c) noexcept -> std::ostream &
    {
        o << "(x: " << c.x << ", \n\ty: " << c.y << ", \n\tz: " << c.z   //
          << ", \n\ta1x: " << c.axis_1_x << ", \n\ta1y: " << c.axis_1_y  //
          << ", \n\ta1z: " << c.axis_1_z << ", \n\ta1r: " << c.axis_1_r  //
          << ", \n\ta2x: " << c.axis_2_x << ", \n\ta2y: " << c.axis_2_y  //
          << ", \n\ta2z: " << c.axis_2_z << ", \n\ta2r: " << c.axis_2_r  //
          << ", \n\ta3x: " << c.axis_3_x << ", \n\ta3y: " << c.axis_3_y  //
          << ", \n\ta3z: " << c.axis_3_z << ", \n\ta3r: " << c.axis_3_r << ")";
        return o;
    }

    template <typename DataT>
    auto operator<<(std::ostream &o, const Cylinder<DataT> &c) noexcept -> std::ostream &
    {
        o << "(x1: " << c.x1 << ", \n\ty1: " << c.y1 << ", \n\tz1: " << c.z1       //
          << ", \n\txv: " << c.xv << ", \n\tyv: " << c.yv << ", \n\tzv: " << c.zv  //
          << ", \n\tr: " << c.r << ")";
        return o;
    }

    template <typename DataT>
    auto operator<<(std::ostream &o, const Sphere<DataT> &s) noexcept -> std::ostream &
    {
        o << "(x: " << s.x << ", \n\ty: " << s.y << ", \n\tz: " << s.z  //
          << ", \n\tr: " << s.r << ")";
        return o;
    }
}  // namespace ompl::vamp::collision
