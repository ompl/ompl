// demos/UR5SphereDemo.cpp
//
// The robot-side half of an SDF-CBF: a UR5 whose links are approximated by 40
// spheres, with closed-form forward kinematics and exact sphere-center
// Jacobians. Together with ompl::sdf::GridSDF this gives the barrier
//
//     h_i(q) = d(p_i(q)) - r_i,      dh_i/dq = grad d(p_i)^T (dp_i/dq)
//
// for every sphere i — the constraint rows a CBF-QP steering step needs.
//
// Run with a path argument to also dump a joint sweep as JSON, which
// scripts/ur5_sphere_viz.py overlays on the real UR5 meshes in PyBullet:
//
//     ./build/demos/demo_UR5Sphere /tmp/ur5_spheres.json
//     python scripts/ur5_sphere_viz.py /tmp/ur5_spheres.json --gui

#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/robots/UR5.h>
#include <ompl/sdf/GridSDF.h>

using UR5 = ompl::robots::UR5;

namespace
{
    // A few configurations to look at: zero, a folded pose, and a reach.
    std::vector<UR5::Configuration> sampleConfigurations()
    {
        std::vector<UR5::Configuration> configs;
        configs.emplace_back(UR5::Configuration::Zero());
        configs.emplace_back((UR5::Configuration() << 0.0, -1.2, 1.8, -0.6, 1.57, 0.0).finished());
        configs.emplace_back((UR5::Configuration() << 1.0, -0.5, 0.7, 0.3, -1.0, 0.5).finished());
        return configs;
    }

    // A joint sweep, for the PyBullet overlay to animate: each joint rotates
    // through its range in turn, from the folded pose.
    std::vector<UR5::Configuration> sweep(std::size_t stepsPerJoint)
    {
        const UR5::Configuration home = (UR5::Configuration() << 0.0, -1.2, 1.8, -0.6, 1.57, 0.0).finished();
        const UR5::Configuration lower = UR5::lowerBounds();
        const UR5::Configuration upper = UR5::upperBounds();

        std::vector<UR5::Configuration> configs;
        for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(UR5::nJoints); ++j)
            for (std::size_t s = 0; s < stepsPerJoint; ++s)
            {
                // Sweep joint j over its range while the rest hold at home.
                const double t = static_cast<double>(s) / static_cast<double>(stepsPerJoint - 1);
                UR5::Configuration q = home;
                q[j] = lower[j] + t * (upper[j] - lower[j]);
                configs.push_back(q);
            }
        return configs;
    }

    void writeJson(const std::string &path, const UR5 &robot, const std::vector<UR5::Configuration> &configs)
    {
        std::FILE *out = std::fopen(path.c_str(), "w");
        if (out == nullptr)
        {
            std::printf("could not open %s for writing\n", path.c_str());
            return;
        }

        std::fprintf(out, "{\n  \"radii\": [");
        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
            std::fprintf(out, "%s%.9g", i ? ", " : "", UR5::spheres()[i].radius);

        std::fprintf(out, "],\n  \"links\": [");
        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
            std::fprintf(out, "%s\"%s\"", i ? ", " : "", UR5::spheres()[i].link);

        std::fprintf(out, "],\n  \"frames\": [");
        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
            std::fprintf(out, "%s%zu", i ? ", " : "", UR5::spheres()[i].frame);

        std::fprintf(out, "],\n  \"configs\": [\n");
        for (std::size_t c = 0; c < configs.size(); ++c)
        {
            UR5::SphereCenters centers;
            UR5::sphereCenters(robot.kinematics(configs[c]), centers);

            std::fprintf(out, "    {\"q\": [");
            for (Eigen::Index j = 0; j < configs[c].size(); ++j)
                std::fprintf(out, "%s%.9g", j ? ", " : "", configs[c][j]);
            std::fprintf(out, "], \"centers\": [");
            for (Eigen::Index i = 0; i < centers.cols(); ++i)
                std::fprintf(out, "%s[%.9g, %.9g, %.9g]", i ? ", " : "", centers(0, i), centers(1, i),
                             centers(2, i));
            std::fprintf(out, "]}%s\n", c + 1 < configs.size() ? "," : "");
        }
        std::fprintf(out, "  ]\n}\n");
        std::fclose(out);
        std::printf("wrote %zu configurations to %s\n", configs.size(), path.c_str());
    }
}  // namespace

int main(int argc, char **argv)
{
    const UR5 robot;  // default base pose: VAMP's 0.9144 m pedestal

    std::printf("UR5: %zu joints, %zu collision spheres over %zu frames\n\n", UR5::nJoints, UR5::nSpheres,
                UR5::nFrames);

    // How the sphere model covers the arm.
    std::printf("sphere model (frame, radius, source link)\n");
    const char *previous = "";
    std::size_t count = 0;
    for (std::size_t i = 0; i <= UR5::nSpheres; ++i)
    {
        const bool end = (i == UR5::nSpheres);
        const char *link = end ? "" : UR5::spheres()[i].link;
        if (!end && std::string(link) == previous)
        {
            ++count;
            continue;
        }
        if (count > 0)
            std::printf("  %-36s x%zu (frame %zu, r = %.3f m)\n", previous, count,
                        UR5::spheres()[i - 1].frame, UR5::spheres()[i - 1].radius);
        previous = link;
        count = 1;
    }

    // Forward kinematics at a few configurations.
    for (const UR5::Configuration &q : sampleConfigurations())
    {
        const UR5::Kinematics kin = robot.kinematics(q);
        const Eigen::Vector3d flange = UR5::endEffectorPose(kin).translation();
        UR5::SphereCenters centers;
        UR5::sphereCenters(kin, centers);

        std::printf("\nq = [");
        for (Eigen::Index j = 0; j < q.size(); ++j)
            std::printf("%s%5.2f", j ? ", " : "", q[j]);
        std::printf("]\n  flange       (%6.3f, %6.3f, %6.3f)\n", flange.x(), flange.y(), flange.z());
        std::printf("  sphere 0     (%6.3f, %6.3f, %6.3f)   base, never moves\n", centers(0, 0), centers(1, 0),
                    centers(2, 0));
        std::printf("  sphere 39    (%6.3f, %6.3f, %6.3f)   gripper fingertip\n", centers(0, 39),
                    centers(1, 39), centers(2, 39));
    }

    // Preview of the CBF hookup: one spherical obstacle in the workspace, and
    // the barrier value and gradient it induces on the tightest sphere.
    std::printf("\n--- barrier against one spherical obstacle at (0.3, 0.3, 1.1), r = 0.2 ---\n");
    const auto obstacle = [](const Eigen::Vector3d &p)
    { return (p - Eigen::Vector3d(0.3, 0.3, 1.1)).norm() - 0.2; };
    const Eigen::AlignedBox3d bounds(Eigen::Vector3d(-1.0, -1.0, 0.4), Eigen::Vector3d(1.0, 1.0, 2.0));
    const ompl::sdf::GridSDF field(obstacle, bounds, /*voxel=*/0.05);

    for (const UR5::Configuration &q : sampleConfigurations())
    {
        const UR5::Kinematics kin = robot.kinematics(q);
        UR5::SphereCenters centers;
        UR5::sphereCenters(kin, centers);

        // Tightest barrier over all spheres: h_i = d(p_i) - r_i.
        std::size_t worst = 0;
        double worstValue = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
        {
            const double h = field.distance(centers.col(static_cast<Eigen::Index>(i))) - UR5::radii()[i];
            if (h < worstValue)
            {
                worstValue = h;
                worst = i;
            }
        }

        const Eigen::Vector3d gradient = field.gradient(centers.col(static_cast<Eigen::Index>(worst)));
        const UR5::Configuration dh = UR5::barrierGradient(kin, worst, gradient);

        std::printf("  q = [");
        for (Eigen::Index j = 0; j < q.size(); ++j)
            std::printf("%s%5.2f", j ? ", " : "", q[j]);
        std::printf("]  min h = %7.4f m on sphere %2zu (%s)\n", worstValue, worst,
                    UR5::spheres()[worst].link);
        std::printf("      dh/dq = [");
        for (Eigen::Index j = 0; j < dh.size(); ++j)
            std::printf("%s%6.3f", j ? ", " : "", dh[j]);
        std::printf("]%s\n", worstValue < 0.0 ? "   (in collision)" : "");
    }

    if (argc > 1)
    {
        std::printf("\n");
        writeJson(argv[1], robot, sweep(/*stepsPerJoint=*/40));
    }
    else
        std::printf("\npass an output path to dump a joint sweep for scripts/ur5_sphere_viz.py\n");

    return 0;
}
