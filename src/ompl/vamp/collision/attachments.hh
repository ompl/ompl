#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <type_traits>
#include <ompl/vamp/collision/shapes.hh>
#include <vector>

namespace ompl::vamp::collision
{
    template <typename DataT>
    struct Attachment
    {
        Attachment(const Eigen::Transform<DataT, 3, Eigen::Isometry> &tf) noexcept : tf(std::move(tf))
        {
        }

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Eigen::Transform<float, 3, Eigen::Isometry> &tf) noexcept
          : Attachment(tf.cast<DataT>())
        {
        }

        Attachment(const Attachment &) = default;

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Attachment<float> &o) noexcept : Attachment(o.tf)
        {
            spheres.reserve(o.spheres.size());
            for (const auto &sphere : o.spheres)
            {
                spheres.emplace_back(sphere);
            }
        }

        std::vector<Sphere<DataT>> spheres;
        // HACK: To get around passing the environment as const but needing to re-pose the
        // attachments
        mutable std::vector<Sphere<DataT>> posed_spheres;
        Eigen::Transform<DataT, 3, Eigen::Isometry> tf;

        inline void pose(const Eigen::Transform<DataT, 3, Eigen::Isometry> &p_tf) const noexcept
        {
            const auto &n_tf = p_tf * tf;

            posed_spheres.resize(spheres.size());
            for (auto i = 0U; i < spheres.size(); ++i)
            {
                const auto &s = spheres[i];
                Eigen::Matrix<DataT, 3, 1> sp(s.x, s.y, s.z);
                auto tfs = n_tf * sp;
                posed_spheres[i] = Sphere<DataT>(tfs[0], tfs[1], tfs[2], s.r);
            }
        }
    };
}  // namespace ompl::vamp::collision
