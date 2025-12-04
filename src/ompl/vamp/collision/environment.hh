#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <optional>
#include <ompl/vamp/collision/shapes.hh>
#include <ompl/vamp/collision/capt.hh>
#include <ompl/vamp/collision/attachments.hh>

namespace ompl::vamp::collision
{
    template <typename DataT>
    struct Environment
    {
        std::vector<Sphere<DataT>> spheres;
        std::vector<Capsule<DataT>> capsules;
        std::vector<Capsule<DataT>> z_aligned_capsules;
        std::vector<Cylinder<DataT>> cylinders;
        std::vector<Cuboid<DataT>> cuboids;
        std::vector<Cuboid<DataT>> z_aligned_cuboids;
        std::vector<HeightField<DataT>> heightfields;
        std::vector<CAPT> pointclouds;
        std::optional<Attachment<DataT>> attachments;

        Environment() = default;

        template <typename OtherDataT>
        explicit Environment(const Environment<OtherDataT> &other)
          : spheres(other.spheres.begin(), other.spheres.end())
          , capsules(other.capsules.begin(), other.capsules.end())
          , z_aligned_capsules(other.z_aligned_capsules.begin(), other.z_aligned_capsules.end())
          , cylinders(other.cylinders.begin(), other.cylinders.end())
          , cuboids(other.cuboids.begin(), other.cuboids.end())
          , z_aligned_cuboids(other.z_aligned_cuboids.begin(), other.z_aligned_cuboids.end())
          , heightfields(other.heightfields.begin(), other.heightfields.end())
          , pointclouds(other.pointclouds.begin(), other.pointclouds.end())
          , attachments(other.template clone_attachments<DataT>())
        {
        }

        inline auto sort()
        {
            std::sort(
                spheres.begin(),
                spheres.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                capsules.begin(),
                capsules.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                z_aligned_capsules.begin(),
                z_aligned_capsules.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                cylinders.begin(),
                cylinders.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                cuboids.begin(),
                cuboids.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                z_aligned_cuboids.begin(),
                z_aligned_cuboids.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
        }

    private:
        template <typename OtherDataT>
        friend struct Environment;

        template <typename OtherDataT>
        inline auto clone_attachments() const noexcept -> std::optional<Attachment<OtherDataT>>
        {
            if (attachments)
            {
                return Attachment<OtherDataT>(*attachments);
            }

            return std::nullopt;
        }
    };

    template <typename DataT>
    inline auto set_attachment_pose(
        const Environment<DataT> &e,
        const Eigen::Transform<DataT, 3, Eigen::Isometry> &p_tf) noexcept
    {
        e.attachments->pose(p_tf);
    }

}  // namespace ompl::vamp::collision
