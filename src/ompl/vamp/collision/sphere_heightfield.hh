#pragma once

#include <ompl/vamp/collision/shapes.hh>
#include <ompl/vamp/collision/math.hh>

namespace ompl::vamp::collision
{
    template <typename DataT>
    inline constexpr auto sphere_heightfield(
        const HeightField<DataT> &a,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r) noexcept -> DataT
    {
        using IndexT = IntVector<DataT::num_scalars_per_row, DataT::num_rows>;
        auto xo = a.x - x;
        auto yo = a.y - y;

        auto xs = (a.xs * xo + a.xd2).clamp(0.F, static_cast<float>(a.xd)).floor();
        auto ys = (a.ys * yo + a.yd2).clamp(0.F, static_cast<float>(a.yd)).floor();

        auto index = ys * a.xd + xs;
        IndexT indices = index.template to<IndexT>();

        auto zh = DataT::gather(a.data.data(), indices);
        auto zhs = a.zs * zh + a.z;

        return z - r - zhs;
    }
}  // namespace ompl::vamp::collision
