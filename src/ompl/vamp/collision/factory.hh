#pragma once

#include <array>
#include <vector>
#include <ompl/vamp/vector.hh>
#include <ompl/vamp/collision/shapes.hh>
#include <ompl/vamp/collision/environment.hh>
#include <ompl/vamp/collision/math.hh>

#include <Eigen/Dense>

namespace ompl::vamp::collision::factory
{
    using ConstEigenRef = const Eigen::Ref<const Eigen::Vector3f> &;
    using ConstEigenRotationRef = const Eigen::Quaternionf &;
    using ConstArrayRef = const std::array<float, 3> &;

    namespace cuboid
    {
        /**
         * Constuct a cuboid given its:
           - Center coordinate
           - Orientation about that center given in Euler XYZ angles
           - Half-extents (radii) from that center along each of the XYZ axes.
         */
        inline static auto flat(
            float center_x,
            float center_y,
            float center_z,
            float rho,
            float theta,
            float phi,
            float half_extent_x,
            float half_extent_y,
            float half_extent_z) noexcept -> collision::Cuboid<float>
        {
            const auto rotation_matrix = Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitZ()) *
                                         Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()) *
                                         Eigen::AngleAxisf(rho, Eigen::Vector3f::UnitX());

            const auto axis_1 = rotation_matrix * Eigen::Vector3f::UnitX();
            const auto axis_2 = rotation_matrix * Eigen::Vector3f::UnitY();
            const auto axis_3 = rotation_matrix * Eigen::Vector3f::UnitZ();

            return collision::Cuboid<float>(
                center_x,
                center_y,
                center_z,
                axis_1[0],
                axis_1[1],
                axis_1[2],
                axis_2[0],
                axis_2[1],
                axis_2[2],
                axis_3[0],
                axis_3[1],
                axis_3[2],
                half_extent_x,
                half_extent_y,
                half_extent_z);
        }

        inline static auto
        eigen(ConstEigenRef center, ConstEigenRef euler_xyz, ConstEigenRef half_extents) noexcept
            -> collision::Cuboid<float>
        {
            return flat(
                center[0],
                center[1],
                center[2],
                euler_xyz[0],
                euler_xyz[1],
                euler_xyz[2],
                half_extents[0],
                half_extents[1],
                half_extents[2]);
        };

        inline static auto
        eigen_rot(ConstEigenRef center, ConstEigenRotationRef rotation, ConstEigenRef half_extents) noexcept
            -> collision::Cuboid<float>
        {
            auto euler = rotation.toRotationMatrix().eulerAngles(0, 1, 2);
            return eigen(center, euler, half_extents);
        };

        inline static auto
        array(ConstArrayRef center, ConstArrayRef euler_xyz, ConstArrayRef half_extents) noexcept
            -> collision::Cuboid<float>
        {
            return flat(
                center[0],
                center[1],
                center[2],
                euler_xyz[0],
                euler_xyz[1],
                euler_xyz[2],
                half_extents[0],
                half_extents[1],
                half_extents[2]);
        };
    }  // namespace cuboid

    namespace cylinder
    {

        namespace endpoints
        {
            /**
               Construct a cylinder given two end points (the center of the "caps" of the cylinder)
               and the radius around the line segment connecting these two points.
             */
            inline static auto
            flat(float x1, float y1, float z1, float x2, float y2, float z2, float radius) noexcept
                -> collision::Cylinder<float>
            {
                auto x_v = x2 - x1;
                auto y_v = y2 - y1;
                auto z_v = z2 - z1;
                auto dot = dot_3(x_v, y_v, z_v, x_v, y_v, z_v);

                return collision::Cylinder<float>(
                    x1, y1, z1, x_v, y_v, z_v, radius, static_cast<float>(1.0 / dot));
            }

            inline static auto eigen(ConstEigenRef endpoint1, ConstEigenRef endpoint2, float radius) noexcept
                -> collision::Cylinder<float>
            {
                return flat(
                    endpoint1[0],
                    endpoint1[1],
                    endpoint1[2],
                    endpoint2[0],
                    endpoint2[1],
                    endpoint2[2],
                    radius);
            }

            inline static auto array(ConstArrayRef endpoint1, ConstArrayRef endpoint2, float radius) noexcept
                -> collision::Cylinder<float>
            {
                return flat(
                    endpoint1[0],
                    endpoint1[1],
                    endpoint1[2],
                    endpoint2[0],
                    endpoint2[1],
                    endpoint2[2],
                    radius);
            }
        }  // namespace endpoints

        namespace center
        {
            /**
               Construct a cylinder from a center point and an Euler XYZ orientation about that center.
               End points are each length / 2 away from the center along the local Z axis.
               Radius is the radius around the line segment connecting the points.
             */
            inline static auto flat(
                float center_x,
                float center_y,
                float center_z,
                float rho,
                float theta,
                float phi,
                float radius,
                float length) noexcept -> collision::Cylinder<float>
            {
                const auto rotation_matrix = Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitZ()) *
                                             Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()) *
                                             Eigen::AngleAxisf(rho, Eigen::Vector3f::UnitX());

                const auto tf_matrix = Eigen::Translation3f(center_x, center_y, center_z) * rotation_matrix;

                const auto p1 = tf_matrix * Eigen::Vector3f(0, 0, length / 2);
                const auto p2 = tf_matrix * Eigen::Vector3f(0, 0, -length / 2);

                return endpoints::flat(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], radius);
            }

            inline static auto
            eigen(ConstEigenRef center, ConstEigenRef euler_xyz, float radius, float length) noexcept
                -> collision::Cylinder<float>
            {
                return flat(
                    center[0],
                    center[1],
                    center[2],
                    euler_xyz[0],
                    euler_xyz[1],
                    euler_xyz[2],
                    radius,
                    length);
            }

            inline static auto eigen_rot(
                ConstEigenRef center,
                ConstEigenRotationRef rotation,
                float radius,
                float length) noexcept -> collision::Cylinder<float>
            {
                auto euler = rotation.toRotationMatrix().eulerAngles(0, 1, 2);
                return eigen(center, euler, radius, length);
            }

            inline static auto
            array(ConstArrayRef center, ConstArrayRef euler_xyz, float radius, float length) noexcept
                -> collision::Cylinder<float>
            {
                return flat(
                    center[0],
                    center[1],
                    center[2],
                    euler_xyz[0],
                    euler_xyz[1],
                    euler_xyz[2],
                    radius,
                    length);
            }
        }  // namespace center
    }  // namespace cylinder

    namespace capsule
    {

        namespace endpoints
        {
            /**
               Construct a capsule given two end points (the center of the "caps" of the capsule)
               and the radius around the line segment connecting these two points.
             */
            inline static auto
            flat(float x1, float y1, float z1, float x2, float y2, float z2, float radius) noexcept
                -> collision::Capsule<float>
            {
                auto x_v = x2 - x1;
                auto y_v = y2 - y1;
                auto z_v = z2 - z1;
                auto dot = dot_3(x_v, y_v, z_v, x_v, y_v, z_v);

                return collision::Capsule<float>(
                    x1, y1, z1, x_v, y_v, z_v, radius, static_cast<float>(1.0 / dot));
            }

            inline static auto eigen(ConstEigenRef endpoint1, ConstEigenRef endpoint2, float radius) noexcept
                -> collision::Capsule<float>
            {
                return flat(
                    endpoint1[0],
                    endpoint1[1],
                    endpoint1[2],
                    endpoint2[0],
                    endpoint2[1],
                    endpoint2[2],
                    radius);
            }

            inline static auto array(ConstArrayRef endpoint1, ConstArrayRef endpoint2, float radius) noexcept
                -> collision::Capsule<float>
            {
                return flat(
                    endpoint1[0],
                    endpoint1[1],
                    endpoint1[2],
                    endpoint2[0],
                    endpoint2[1],
                    endpoint2[2],
                    radius);
            }
        }  // namespace endpoints

        namespace center
        {
            /**
               Construct a capsule from a center point and an Euler XYZ orientation about that center.
               End points are each length / 2 away from the center along the local Z axis.
               Radius is the radius around the line segment connecting the points.
             */
            inline static auto flat(
                float center_x,
                float center_y,
                float center_z,
                float rho,
                float theta,
                float phi,
                float radius,
                float length) noexcept -> collision::Capsule<float>
            {
                const auto rotation_matrix = Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitZ()) *
                                             Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()) *
                                             Eigen::AngleAxisf(rho, Eigen::Vector3f::UnitX());

                const auto tf_matrix = Eigen::Translation3f(center_x, center_y, center_z) * rotation_matrix;

                const auto p1 = tf_matrix * Eigen::Vector3f(0, 0, length / 2);
                const auto p2 = tf_matrix * Eigen::Vector3f(0, 0, -length / 2);

                return endpoints::flat(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], radius);
            }

            inline static auto
            eigen(ConstEigenRef center, ConstEigenRef euler_xyz, float radius, float length) noexcept
                -> collision::Capsule<float>
            {
                return flat(
                    center[0],
                    center[1],
                    center[2],
                    euler_xyz[0],
                    euler_xyz[1],
                    euler_xyz[2],
                    radius,
                    length);
            }

            inline static auto eigen_rot(
                ConstEigenRef center,
                ConstEigenRotationRef rotation,
                float radius,
                float length) noexcept -> collision::Capsule<float>
            {
                auto euler = rotation.toRotationMatrix().eulerAngles(0, 1, 2);
                return eigen(center, euler, radius, length);
            }

            inline static auto
            array(ConstArrayRef center, ConstArrayRef euler_xyz, float radius, float length) noexcept
                -> collision::Capsule<float>
            {
                return flat(
                    center[0],
                    center[1],
                    center[2],
                    euler_xyz[0],
                    euler_xyz[1],
                    euler_xyz[2],
                    radius,
                    length);
            }
        }  // namespace center
    }  // namespace capsule

    namespace sphere
    {
        inline static auto flat(float center_x, float center_y, float center_z, float radius) noexcept
            -> collision::Sphere<float>
        {
            return collision::Sphere<float>(center_x, center_y, center_z, radius);
        }

        inline static auto eigen(ConstEigenRef center, float radius) noexcept -> collision::Sphere<float>
        {
            return flat(center[0], center[1], center[2], radius);
        }

        inline static auto array(ConstArrayRef center, float radius) noexcept -> collision::Sphere<float>
        {
            return flat(center[0], center[1], center[2], radius);
        }
    }  // namespace sphere

    namespace heightfield
    {
        inline static auto flat(
            float center_x,
            float center_y,
            float center_z,
            float scale_x,
            float scale_y,
            float scale_z,
            std::size_t num_cells_x,
            std::size_t num_cells_y,
            const std::vector<float> &data) noexcept -> collision::HeightField<float>
        {
            return collision::HeightField(
                center_x,
                center_y,
                center_z,
                1.F / scale_x,
                1.F / scale_y,
                1.F / scale_z,
                num_cells_x,
                num_cells_y,
                data);
        }

        inline static auto eigen(
            ConstEigenRef center,
            ConstEigenRef scale,
            const std::array<std::size_t, 2> &dimensions,
            const std::vector<float> &data) noexcept -> collision::HeightField<float>
        {
            return flat(
                center[0],
                center[1],
                center[2],
                scale[0],
                scale[1],
                scale[2],
                dimensions[0],
                dimensions[1],
                data);
        }

        inline static auto array(
            ConstArrayRef center,
            ConstArrayRef scale,
            const std::array<std::size_t, 2> &dimensions,
            const std::vector<float> &data) noexcept -> collision::HeightField<float>
        {
            return flat(
                center[0],
                center[1],
                center[2],
                scale[0],
                scale[1],
                scale[2],
                dimensions[0],
                dimensions[1],
                data);
        }
    }  // namespace heightfield
}  // namespace ompl::vamp::collision::factory
