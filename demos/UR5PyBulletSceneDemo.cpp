// demos/UR5PyBulletSceneDemo.cpp
//
// Runs CBF steering on a workspace baked outside OMPL.
//
// The scenes come from a PyBullet setup (ur5_experiments/), which exports two
// files per environment: a signed distance grid in GridSDF's node layout, and a
// problem file holding the start configuration, goal configurations and joint
// limits. Nothing here knows what PyBullet is -- it loads a grid and plans.
//
//     ./build/demos/demo_UR5PyBulletScene <scene.problem> [seconds] [out.path]
//     ./build/demos/demo_UR5PyBulletScene <scene.problem> --probe   < points.txt
//
// `--probe` reads "x y z" triples on stdin and prints the field value and
// gradient at each, so the Python baker can check that the grid it wrote is the
// field this side reads. That check is the whole point of a shared byte layout:
// a sign flip or an axis-order mistake in the exporter is otherwise invisible
// until the planner drives the arm through a wall.
//
// The goals are *not* validated here. The exporter picks configurations that
// maximise the barrier value precisely because being collision-free is not
// enough -- with a 0.06 m margin over a sphere model that under-covers the
// links, a mesh-free configuration inside a shelf bay is routinely infeasible.
// A goal with h < 0 is unreachable by construction: the filter would have to
// violate the barrier to arrive.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/cbf/CBFControlFilter.h>
#include <ompl/cbf/ClearanceBarrier.h>
#include <ompl/cbf/FilteredMotionValidator.h>
#include <ompl/cbf/FilteredStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/robots/UR5.h>
#include <ompl/sdf/GridSDF.h>
#include <ompl/util/Time.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using Barrier = ompl::cbf::ClearanceBarrier;
using Filter = ompl::cbf::CBFControlFilter;
using UR5 = ompl::robots::UR5;
namespace sdf = ompl::sdf;

namespace
{
    constexpr int dimension = static_cast<int>(UR5::nJoints);

    /// One goal from the problem file: a configuration plus what it was for.
    struct Goal
    {
        int index{0};
        std::string label;
        UR5::Configuration configuration;
        double exportedBarrier{0.0};  ///< h the exporter measured, for cross-checking
        Eigen::Vector3d target{Eigen::Vector3d::Zero()};
    };

    /// The exported scene: everything needed to set up the planning problem.
    struct Scene
    {
        std::string name;
        std::string gridPath;
        double voxel{0.02};
        double margin{Barrier::defaultMargin};
        UR5::Configuration start{UR5::Configuration::Zero()};
        UR5::Configuration lower{UR5::Configuration::Zero()};
        UR5::Configuration upper{UR5::Configuration::Zero()};
        std::vector<Goal> goals;
    };

    std::string directoryOf(const std::string &path)
    {
        const std::size_t slash = path.find_last_of('/');
        return slash == std::string::npos ? std::string(".") : path.substr(0, slash);
    }

    Scene readScene(const std::string &path)
    {
        std::ifstream in(path);
        if (!in)
            throw std::runtime_error("cannot open problem file " + path);

        Scene scene;
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream fields(line);
            std::string key;
            fields >> key;

            if (key == "env")
                fields >> scene.name;
            else if (key == "grid")
            {
                std::string file;
                fields >> file;
                scene.gridPath = directoryOf(path) + "/" + file;
            }
            else if (key == "voxel")
                fields >> scene.voxel;
            else if (key == "margin")
                fields >> scene.margin;
            else if (key == "start")
                for (int j = 0; j < dimension; ++j)
                    fields >> scene.start[j];
            else if (key == "limits_lower")
                for (int j = 0; j < dimension; ++j)
                    fields >> scene.lower[j];
            else if (key == "limits_upper")
                for (int j = 0; j < dimension; ++j)
                    fields >> scene.upper[j];
            else if (key == "goal")
            {
                Goal goal;
                fields >> goal.index >> goal.label;
                for (int j = 0; j < dimension; ++j)
                    fields >> goal.configuration[j];
                fields >> goal.exportedBarrier;
                for (int j = 0; j < 3; ++j)
                    fields >> goal.target[j];
                scene.goals.push_back(goal);
            }
        }
        if (scene.gridPath.empty())
            throw std::runtime_error("problem file names no grid: " + path);
        return scene;
    }

    /// Print field values on stdin points, for the exporter to diff against.
    int probe(const sdf::GridSDF &field)
    {
        const Eigen::Vector3i dims = field.dimensions();
        std::printf("# dims %d %d %d\n", dims[0], dims[1], dims[2]);
        std::printf("# spacing %.9f %.9f %.9f\n", field.spacing()[0], field.spacing()[1],
                    field.spacing()[2]);
        std::printf("# bounds %.9f %.9f %.9f %.9f %.9f %.9f\n", field.bounds().min()[0],
                    field.bounds().min()[1], field.bounds().min()[2], field.bounds().max()[0],
                    field.bounds().max()[1], field.bounds().max()[2]);
        std::printf("# maxGradientNorm %.9f\n", field.maxGradientNorm());

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        while (std::scanf("%lf %lf %lf", &x, &y, &z) == 3)
        {
            const sdf::ValueGradient vg = field.valueAndGradient(Eigen::Vector3d(x, y, z));
            std::printf("%.9f %.9f %.9f %.9f\n", vg.value, vg.gradient[0], vg.gradient[1],
                        vg.gradient[2]);
        }
        return 0;
    }

    /// Worst sphere-sphere overlap between links that are not neighbours.
    ///
    /// `ClearanceBarrier` is robot-versus-environment: nothing in `h` refers to
    /// one part of the arm meeting another. Dropping the state validity checker
    /// therefore drops the only thing that was checking self-collision, and the
    /// filter has no reason not to fold the arm through itself on the way to a
    /// goal -- which, measured against the real meshes in PyBullet, it does.
    ///
    /// Reported here so the gap is visible without leaving OMPL. Returns the most
    /// negative `|p_i - p_j| - (r_i + r_j)`; >= 0 means clear.
    ///
    /// Spheres on the same frame, or on frames one joint apart, are skipped: those
    /// are rigidly attached or share an axis, so they touch by construction. This
    /// is the same rule the PyBullet side applies to link pairs.
    double worstSelfOverlap(const UR5 &robot, const UR5::Configuration &q)
    {
        const UR5::SphereCenters centers = robot.sphereCenters(q);
        const auto &spheres = UR5::spheres();

        double worst = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
            for (std::size_t j = i + 1; j < UR5::nSpheres; ++j)
            {
                const std::size_t a = spheres[i].frame;
                const std::size_t b = spheres[j].frame;
                const std::size_t gap = a > b ? a - b : b - a;
                if (gap < 2)
                    continue;
                const double distance = (centers.col(i) - centers.col(j)).norm() -
                                        (spheres[i].radius + spheres[j].radius);
                worst = std::min(worst, distance);
            }
        return worst;
    }

    struct Result
    {
        bool solved{false};
        double seconds{0.0};
        std::size_t waypoints{0};
        std::size_t auditedStates{0};
        std::size_t unsafeStates{0};
        double minBarrier{std::numeric_limits<double>::infinity()};
        double minSelfOverlap{std::numeric_limits<double>::infinity()};
        std::size_t selfColliding{0};
        double goalDistance{0.0};
        std::size_t vertices{0};
        std::size_t blocked{0};
        std::size_t filtered{0};
        std::size_t steps{0};
    };

    Result plan(const Scene &scene, const sdf::GridSDF &field, const Goal &goal, double timeLimit,
                double stepSize, double range, std::vector<UR5::Configuration> *path)
    {
        const UR5 robot;
        const Barrier audit(robot, field, scene.margin);
        // The filter guards a *larger* margin than the one audited: enforcing
        // h >= 0 on the interpolated field only delivers h >= -O(voxel), so the
        // buffer is what turns "the QP was satisfied" into "the audit passes".
        const Barrier guard =
            Barrier::guarding(robot, field, scene.margin, Barrier::interpolationBuffer(field));

        Filter::Parameters parameters;
        const Filter filter(guard, parameters);

        auto space = std::make_shared<ompl::cbf::FilteredStateSpace>(filter, stepSize,
                                                                    UR5::velocityLimits());
        ob::RealVectorBounds bounds(dimension);
        for (int j = 0; j < dimension; ++j)
        {
            bounds.setLow(j, scene.lower[j]);
            bounds.setHigh(j, scene.upper[j]);
        }
        space->setBounds(bounds);

        auto si = std::make_shared<ob::SpaceInformation>(space);
        // The rollout certifies every step it emits, so there is nothing left for a
        // validity checker to do -- see README_CBF_USAGE.md on removing it.
        si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
        si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
        si->setup();

        ob::ScopedState<ob::RealVectorStateSpace> start(space);
        ob::ScopedState<ob::RealVectorStateSpace> target(space);
        for (int j = 0; j < dimension; ++j)
        {
            start->values[j] = scene.start[j];
            target->values[j] = goal.configuration[j];
        }

        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        // A rollout cannot be asked to land exactly on a state beside an obstacle,
        // so the goal is a ball, matching the control-space demo's tolerance.
        pdef->setStartAndGoalStates(start, target, 0.35);

        auto planner = std::make_shared<og::RRTConnect>(si);
        planner->setRange(range);
        planner->setProblemDefinition(pdef);
        planner->setup();

        const ompl::time::point begin = ompl::time::now();
        const ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeLimit));

        Result result;
        result.seconds = ompl::time::seconds(ompl::time::now() - begin);
        result.solved = (status == ob::PlannerStatus::EXACT_SOLUTION);

        const ompl::cbf::FilteredStateSpace::Statistics stats = space->statistics();
        result.steps = stats.steps;
        result.filtered = stats.filtered;
        result.blocked = stats.blocked;

        ob::PlannerData data(si);
        planner->getPlannerData(data);
        result.vertices = data.numVertices();

        if (pdef->hasSolution())
        {
            auto solution = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
            result.waypoints = solution->getStateCount();

            // interpolate() runs the space's interpolate, which *is* the CBF rollout,
            // so this reconstructs the executed motion rather than a straight-line
            // stand-in -- the only honest thing to audit.
            solution->interpolate();
            result.auditedStates = solution->getStateCount();
            for (std::size_t i = 0; i < solution->getStateCount(); ++i)
            {
                const UR5::Configuration q =
                    ompl::cbf::FilteredStateSpace::configurationOf(solution->getState(i));
                const double h = audit.worstValue(q);
                result.minBarrier = std::min(result.minBarrier, h);
                if (h < 0.0)
                    ++result.unsafeStates;

                const double own = worstSelfOverlap(robot, q);
                result.minSelfOverlap = std::min(result.minSelfOverlap, own);
                if (own < 0.0)
                    ++result.selfColliding;
                if (path != nullptr)
                    path->push_back(q);
            }
            const UR5::Configuration last = ompl::cbf::FilteredStateSpace::configurationOf(
                solution->getState(solution->getStateCount() - 1));
            result.goalDistance = (last - goal.configuration).norm();
        }
        return result;
    }
}  // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::printf("usage: %s <scene.problem> [seconds] [out.path]\n", argv[0]);
        std::printf("       %s <scene.problem> --probe   < points.txt\n", argv[0]);
        return 1;
    }

    const std::string problemPath = argv[1];
    const bool probeMode = (argc > 2 && std::string(argv[2]) == "--probe");
    const double timeLimit = (argc > 2 && !probeMode) ? std::atof(argv[2]) : 10.0;
    const std::string outPath = (argc > 3 && !probeMode) ? argv[3] : std::string();

    const Scene scene = readScene(problemPath);
    const sdf::GridSDF field = sdf::GridSDF::load(scene.gridPath);

    if (probeMode)
        return probe(field);

    const UR5 robot;
    const Barrier barrier(robot, field, scene.margin);
    const double buffer = Barrier::interpolationBuffer(field);
    const double stepSize = 0.05;
    const double range = 1.5;

    std::printf("scene %s   grid %s\n", scene.name.c_str(), scene.gridPath.c_str());
    const Eigen::Vector3i dims = field.dimensions();
    std::printf("field %dx%dx%d nodes, voxel %.4f m, maxGradientNorm %.4f\n", dims[0], dims[1],
                dims[2], scene.voxel, field.maxGradientNorm());
    std::printf("margin %.4f, interpolation buffer %.4f, guarded margin %.4f\n", scene.margin,
                buffer, scene.margin + buffer);

    // Confirm the two sides agree about the start before planning: a mismatch here
    // means the grid and the sphere model disagree about where the world is, and
    // every number after it would be measuring the wrong thing.
    const double startBarrier = barrier.worstValue(scene.start);
    std::printf("start barrier h = %+.4f m\n", startBarrier);
    if (startBarrier < 0.0)
    {
        std::printf("  start is already in violation; the filter cannot recover from this\n");
        return 2;
    }

    std::printf("\n%-28s %7s %8s %6s %8s %8s %9s %7s %8s %9s\n", "goal", "solved", "seconds",
                "wpts", "audited", "unsafe", "min h", "blocked", "selfcoll", "min self");

    std::vector<UR5::Configuration> combined;
    std::size_t solvedCount = 0;
    std::size_t unsafeTotal = 0;
    std::size_t selfTotal = 0;
    for (const Goal &goal : scene.goals)
    {
        const double exported = barrier.worstValue(goal.configuration);
        std::vector<UR5::Configuration> path;
        const Result r = plan(scene, field, goal, timeLimit, stepSize, range, &path);
        solvedCount += r.solved ? 1 : 0;
        unsafeTotal += r.unsafeStates;
        selfTotal += r.selfColliding;

        std::printf("%-28s %7s %8.3f %6zu %8zu %8zu %+9.4f %7zu %8zu %+9.4f\n", goal.label.c_str(),
                    r.solved ? "yes" : "no", r.seconds, r.waypoints, r.auditedStates,
                    r.unsafeStates, r.solved ? r.minBarrier : 0.0, r.blocked, r.selfColliding,
                    r.solved ? r.minSelfOverlap : 0.0);

        // The exporter measured h with its own copy of the sphere model; if that
        // disagrees with this one the two pipelines have drifted apart.
        if (std::abs(exported - goal.exportedBarrier) > 5e-3)
            std::printf("    ! barrier mismatch at goal: exporter %+.4f, here %+.4f\n",
                        goal.exportedBarrier, exported);

        if (!outPath.empty() && r.solved)
        {
            combined.push_back(scene.start);
            for (const UR5::Configuration &q : path)
                combined.push_back(q);
        }
    }

    std::printf("\nsolved %zu/%zu goals, %zu audited states below the audited margin\n", solvedCount,
                scene.goals.size(), unsafeTotal);
    if (selfTotal > 0)
        std::printf("%zu audited states self-collide: the barrier does not model the arm "
                    "against itself, so nothing was checking it\n",
                    selfTotal);

    if (!outPath.empty())
    {
        std::ofstream out(outPath);
        if (!out)
        {
            std::printf("cannot write %s\n", outPath.c_str());
            return 1;
        }
        out << "# joint-space path for " << scene.name << ", one configuration per line\n";
        for (const UR5::Configuration &q : combined)
        {
            for (int j = 0; j < dimension; ++j)
                out << (j ? " " : "") << q[j];
            out << "\n";
        }
        std::printf("wrote %s (%zu configurations)\n", outPath.c_str(), combined.size());
    }
    return solvedCount == scene.goals.size() ? 0 : 3;
}
