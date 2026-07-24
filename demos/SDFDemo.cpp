// demos/SDFDemo.cpp
//
// Minimal signed distance field (SDF) service: hand GridSDF a distance function
// for your environment, and it caches the field (value + gradient) on a voxel
// grid for fast O(1) queries. The gradient it returns is the direction of
// increasing clearance (away from the nearest obstacle) — the pair a Control
// Barrier Function needs. GridSDF is environment-agnostic: it only sees the
// distance function, never the geometry.

#include <algorithm>
#include <cstdio>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/sdf/GridSDF.h>

int main()
{
    // Example environment: two spheres. Signed distance is the min over them.
    const auto sceneDistance = [](const Eigen::Vector3d &p)
    {
        const double d0 = (p - Eigen::Vector3d(0.5, 0.0, 0.0)).norm() - 0.2;
        const double d1 = (p - Eigen::Vector3d(-0.4, 0.3, 0.0)).norm() - 0.15;
        return std::min(d0, d1);
    };

    const Eigen::AlignedBox3d bounds(Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d(1.0, 1.0, 1.0));
    std::printf("Baking GridSDF...\n");
    const ompl::sdf::GridSDF grid(sceneDistance, bounds, /*voxel=*/0.05);

    const Eigen::Vector3i dims = grid.dimensions();
    std::printf("Grid: %d x %d x %d nodes\n\n", dims.x(), dims.y(), dims.z());

    std::printf("   x        d(x)      grad (unit)\n");
    for (double x = -0.9; x <= 0.9; x += 0.1)
    {
        const Eigen::Vector3d p(x, 0.0, 0.0);
        const ompl::sdf::ValueGradient vg = grid.valueAndGradient(p);
        const Eigen::Vector3d g = vg.gradient.normalized();
        std::printf("%6.2f  %9.4f   (%5.2f, %5.2f, %5.2f)\n", x, vg.value, g.x(), g.y(), g.z());
    }

    return 0;
}
