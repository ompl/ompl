// demos/UR5MBMBenchmark.cpp
//
// The CBF rollout against ordinary collision-checked planning on a *standard* problem
// set: MotionBenchMaker's UR5 scenes, 689 problems over 7 scenes, as shipped by VAMP.
//
//     ./scripts/mbm_to_scenes.py /path/to/vamp/resources/ur5/problems.json scenes.txt
//     ./build/demos/demo_UR5MBMBenchmark scenes.txt [perScene] [seconds] [voxel] [stepSize] [range]
//
// Why this and not UR5CBFPlanningDemo's scene: that one is a pair of spheres placed to
// block the direct sweep, and `geometric::RRTConnect` solves it in six vertices. A
// method whose claim is "no wasted edges" cannot show anything on a problem with no
// wasted edges. These scenes are cluttered, externally defined, and widely reported, so
// they can neither be tuned to flatter the filter nor dismissed as a strawman.
//
// Every obstacle in the set is a box or a cylinder, both of which have an exact
// closed-form signed distance -- so the field the barrier reads is exact up to the grid,
// with no mesh, no FCL, and no sign ambiguity.
//
// ### What is compared
//
// - `rrtconnect`: stock geometric RRTConnect, straight-line edges, the SDF behind an
//   ordinary StateValidityChecker. The bar.
// - `cbf-rrtc`: the same planner over `cbf::FilteredStateSpace`, so every edge is a CBF
//   rollout and every intermediate state is certified as it is produced. No state
//   validity checker at all.
//
// Both see the same field, the same margin, the same start and goal.
//
// ### Feasibility is reported, not assumed
//
// MotionBenchMaker calls a problem valid when the *mesh* robot is collision free. Our
// robot is 40 spheres that do not enclose those meshes (see ClearanceBarrier) and the
// barrier adds a margin on top, so a problem can be valid upstream and still have its
// start or goal inside our margin. Those are counted and excluded rather than scored as
// failures -- and the count is itself a result, because a margin that rules out most of
// a standard benchmark is a finding about the margin.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/cbf/CBFControlFilter.h>
#include <ompl/cbf/FilteredMotionValidator.h>
#include <ompl/cbf/FilteredStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Time.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using Barrier = ompl::cbf::ClearanceBarrier;
using Filter = ompl::cbf::CBFControlFilter;
using Space = ompl::cbf::FilteredStateSpace;
using UR5 = ompl::robots::UR5;
namespace sdf = ompl::sdf;

namespace
{
    constexpr int dimension = 6;

    /// Joint-space spacing both rows are audited at, in radians. Finer than the rollout
    /// step so the audit is not merely re-reading the filter's own decisions.
    constexpr double auditResolution = 0.02;

    /// One obstacle: a box (`halfExtents`) or a cylinder (`radius`, `halfLength` about
    /// the local z axis), posed in the world.
    struct Obstacle
    {
        enum class Kind
        {
            Box,
            Cylinder
        };

        Kind kind{Kind::Box};
        Eigen::Vector3d halfExtents{Eigen::Vector3d::Zero()};
        double radius{0.0};
        double halfLength{0.0};
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};

        /// Exact signed distance. Both formulas are the standard ones: reduce to the
        /// box's local frame, take the per-axis overshoot, then the norm of its positive
        /// part outside and the largest (negative) component inside.
        double distance(const Eigen::Vector3d &p) const
        {
            const Eigen::Vector3d q = rotation.transpose() * (p - position);
            if (kind == Kind::Box)
            {
                const Eigen::Vector3d d = q.cwiseAbs() - halfExtents;
                return d.cwiseMax(0.0).norm() + std::min(d.maxCoeff(), 0.0);
            }
            const Eigen::Vector2d d(q.head<2>().norm() - radius, std::abs(q.z()) - halfLength);
            return d.cwiseMax(0.0).norm() + std::min(d.maxCoeff(), 0.0);
        }
    };

    struct Problem
    {
        std::string scene;
        int index{0};
        UR5::Configuration start{UR5::Configuration::Zero()};
        UR5::Configuration goal{UR5::Configuration::Zero()};
        std::vector<Obstacle> obstacles;

        /// A union of solids is the min of their distances, which is still exact.
        sdf::DistanceFn field() const
        {
            const std::vector<Obstacle> solids = obstacles;
            return [solids](const Eigen::Vector3d &p)
            {
                double distance = std::numeric_limits<double>::infinity();
                for (const Obstacle &solid : solids)
                    distance = std::min(distance, solid.distance(p));
                return distance;
            };
        }
    };

    Eigen::Matrix3d rotationOf(double x, double y, double z, double w)
    {
        return Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
    }

    std::vector<Problem> readProblems(const std::string &path)
    {
        std::ifstream in(path);
        if (!in)
            throw ompl::Exception("cannot open " + path +
                                  " -- generate it with scripts/mbm_to_scenes.py");

        std::vector<Problem> problems;
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream fields(line);
            std::string tag;
            fields >> tag;

            if (tag == "problem")
            {
                problems.emplace_back();
                fields >> problems.back().scene >> problems.back().index;
            }
            else if (problems.empty())
            {
                continue;
            }
            else if (tag == "start" || tag == "goal")
            {
                UR5::Configuration q;
                for (int j = 0; j < dimension; ++j)
                    fields >> q[j];
                (tag == "start" ? problems.back().start : problems.back().goal) = q;
            }
            else if (tag == "box" || tag == "cyl")
            {
                Obstacle obstacle;
                double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
                if (tag == "box")
                {
                    obstacle.kind = Obstacle::Kind::Box;
                    fields >> obstacle.halfExtents[0] >> obstacle.halfExtents[1] >>
                        obstacle.halfExtents[2];
                }
                else
                {
                    obstacle.kind = Obstacle::Kind::Cylinder;
                    fields >> obstacle.radius >> obstacle.halfLength;
                }
                fields >> obstacle.position[0] >> obstacle.position[1] >> obstacle.position[2] >>
                    qx >> qy >> qz >> qw;
                obstacle.rotation = rotationOf(qx, qy, qz, qw);
                problems.back().obstacles.push_back(obstacle);
            }
        }
        return problems;
    }

    ob::RealVectorBounds jointBounds()
    {
        ob::RealVectorBounds bounds(dimension);
        for (int j = 0; j < dimension; ++j)
        {
            bounds.setLow(j, UR5::lowerBounds()[j]);
            bounds.setHigh(j, UR5::upperBounds()[j]);
        }
        return bounds;
    }

    struct Result
    {
        bool solved{false};
        double seconds{0.0};
        std::size_t evaluations{0};  ///< collision checks, or filter calls
        std::size_t vertices{0};
        std::size_t unsafeStates{0};
        std::size_t auditedStates{0};
        double minClearance{std::numeric_limits<double>::infinity()};
    };

    /// Audit a solution the way the CBF rows must be audited: densify through the
    /// space's own interpolate() -- which for FilteredStateSpace is the rollout, so this
    /// reconstructs the executed motion -- then evaluate the *unbuffered* barrier at
    /// every state.
    void audit(const og::PathGeometric &solution, const Barrier &barrier, Result &result)
    {
        og::PathGeometric path(solution);
        // The same joint-space resolution the baseline is audited at. PathGeometric's
        // argument-free interpolate() would use longestValidSegmentFraction instead,
        // which on this space is 0.31 rad -- it would sample the rollout an order of
        // magnitude more coarsely than the baseline's audit and make this row look clean
        // by not looking. Because the space's interpolate() *is* the rollout, asking for
        // more states samples the executed motion itself, not a straight-line stand-in.
        path.interpolate(static_cast<unsigned int>(path.length() / auditResolution));
        result.auditedStates = path.getStateCount();
        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            const double h = barrier.worstValue(Space::configurationOf(path.getState(i)));
            result.minClearance = std::min(result.minClearance, h);
            if (h < 0.0)
                ++result.unsafeStates;
        }
    }

    /// The bar: straight-line edges, the same SDF behind an ordinary validity checker.
    Result runCollisionChecked(const Problem &problem, const Barrier &barrier, double range,
                               double timeLimit, double segmentFraction)
    {
        auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
        space->setBounds(jointBounds());
        if (segmentFraction > 0.0)
            space->setLongestValidSegmentFraction(segmentFraction);
        auto si = std::make_shared<ob::SpaceInformation>(space);

        std::size_t checks = 0;
        si->setStateValidityChecker([&barrier, &checks](const ob::State *state)
                                    {
                                        ++checks;
                                        return barrier.isSafe(Space::configurationOf(state));
                                    });
        si->setup();

        ob::ScopedState<> start(space), goal(space);
        for (int j = 0; j < dimension; ++j)
        {
            start[j] = problem.start[j];
            goal[j] = problem.goal[j];
        }
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal, 0.05);

        auto planner = std::make_shared<og::RRTConnect>(si);
        planner->setRange(range);
        planner->setProblemDefinition(pdef);
        planner->setup();

        Result result;
        const ompl::time::point begin = ompl::time::now();
        const ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeLimit));
        result.seconds = ompl::time::seconds(ompl::time::now() - begin);
        result.solved = (status == ob::PlannerStatus::EXACT_SOLUTION);
        result.evaluations = checks;

        ob::PlannerData data(si);
        planner->getPlannerData(data);
        result.vertices = data.numVertices();

        if (result.solved)
        {
            // Audited at the same resolution the rollout uses, so the clearance columns
            // mean the same thing in both rows.
            og::PathGeometric path(*std::static_pointer_cast<og::PathGeometric>(
                pdef->getSolutionPath()));
            path.interpolate(static_cast<unsigned int>(path.length() / auditResolution));
            result.auditedStates = path.getStateCount();
            for (std::size_t i = 0; i < path.getStateCount(); ++i)
            {
                const double h = barrier.worstValue(Space::configurationOf(path.getState(i)));
                result.minClearance = std::min(result.minClearance, h);
                if (h < 0.0)
                    ++result.unsafeStates;
            }
        }
        return result;
    }

    /// The CBF rollout as the state space's interpolate(), with no collision checking
    /// anywhere: the barrier certifies each step as it is produced.
    Result runFiltered(const Problem &problem, const Barrier &audited, const Filter &filter,
                       double stepSize, double range, double timeLimit)
    {
        auto space = std::make_shared<Space>(filter, stepSize, UR5::velocityLimits());
        space->setBounds(jointBounds());

        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
        si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
        si->setup();

        ob::ScopedState<> start(space), goal(space);
        for (int j = 0; j < dimension; ++j)
        {
            start[j] = problem.start[j];
            goal[j] = problem.goal[j];
        }
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        // A rollout cannot be asked to land on an exact state next to an obstacle, so the
        // goal gets a tolerance. Kept small enough that "solved" still means solved.
        pdef->setStartAndGoalStates(start, goal, 0.1);

        auto planner = std::make_shared<og::RRTConnect>(si);
        planner->setRange(range);
        planner->setProblemDefinition(pdef);
        planner->setup();

        Result result;
        const ompl::time::point begin = ompl::time::now();
        const ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeLimit));
        result.seconds = ompl::time::seconds(ompl::time::now() - begin);
        result.solved = (status == ob::PlannerStatus::EXACT_SOLUTION);
        result.evaluations = space->statistics().steps;

        ob::PlannerData data(si);
        planner->getPlannerData(data);
        result.vertices = data.numVertices();

        if (result.solved)
            audit(*std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath()), audited,
                  result);
        return result;
    }

    struct Tally
    {
        int attempted{0};
        int skipped{0};  ///< start or goal inside our margin
        /// min(start, goal) sphere clearance at zero margin, per problem. VAMP validated
        /// this set against the same 40 spheres with no margin, so these should all be
        /// positive; how far above zero they sit is what decides which margins are
        /// affordable on a standard benchmark.
        std::vector<double> endpointClearance;
        int solved[2]{0, 0};
        std::vector<double> seconds[2];
        std::vector<double> evaluations[2];
        std::vector<double> vertices[2];
        std::size_t unsafe[2]{0, 0};
        std::size_t audited[2]{0, 0};
        double worstClearance[2]{std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity()};

        void add(int row, const Result &result)
        {
            solved[row] += result.solved ? 1 : 0;
            seconds[row].push_back(result.seconds);
            evaluations[row].push_back(static_cast<double>(result.evaluations));
            vertices[row].push_back(static_cast<double>(result.vertices));
            unsafe[row] += result.unsafeStates;
            audited[row] += result.auditedStates;
            if (result.solved)
                worstClearance[row] = std::min(worstClearance[row], result.minClearance);
        }
    };

    double median(std::vector<double> values)
    {
        if (values.empty())
            return 0.0;
        std::sort(values.begin(), values.end());
        return values[values.size() / 2];
    }

    void reportRow(const char *label, const Tally &tally, int row)
    {
        const int scored = tally.attempted - tally.skipped;
        std::printf("  %-11s %3d/%-4d %9.2f %10.0f %8.0f", label, tally.solved[row], scored,
                    1e3 * median(tally.seconds[row]), median(tally.evaluations[row]),
                    median(tally.vertices[row]));
        if (tally.solved[row] > 0)
            std::printf(" %10.4f %6zu/%-7zu\n", tally.worstClearance[row], tally.unsafe[row],
                        tally.audited[row]);
        else
            std::printf(" %10s %14s\n", "-", "-");
    }
}  // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::printf("usage: %s scenes.txt [perScene] [seconds] [voxel] [stepSize] [range]\n\n"
                    "Generate scenes.txt with scripts/mbm_to_scenes.py.\n",
                    argv[0]);
        return 1;
    }

    const std::string path = argv[1];
    const int perScene = argc > 2 ? std::atoi(argv[2]) : 10;
    const double timeLimit = argc > 3 ? std::atof(argv[3]) : 5.0;
    const double voxel = argc > 4 ? std::atof(argv[4]) : 0.03;
    const double stepSize = argc > 5 ? std::atof(argv[5]) : 0.05;
    const double range = argc > 6 ? std::atof(argv[6]) : 2.0;
    // The audited margin, and the extra the filter guards on top of it. MotionBenchMaker
    // endpoints are grasp poses sitting ~8 mm off the shelf, so the defaults
    // (0.06 + one voxel) rule out most of the set -- these exist to find out what does
    // fit, with the unsafe column as the check on having shrunk them too far.
    const double margin = argc > 7 ? std::atof(argv[7]) : Barrier::defaultMargin;
    const double buffer = argc > 8 ? std::atof(argv[8]) : -1.0;
    // Resolution the baseline checks its straight-line edges at, as a fraction of the
    // state space's maximum extent. OMPL defaults to 0.01, which on this space is 0.31
    // rad between samples -- far coarser than the audit, so the baseline is scored unsafe
    // at the default. Tightening this is what makes the comparison like for like: both
    // rows then have to be audit-clean, and the cost of being so is the thing to compare.
    const double segmentFraction = argc > 9 ? std::atof(argv[9]) : -1.0;

    ompl::RNG::setSeed(1);
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    const std::vector<Problem> problems = readProblems(path);
    const UR5 robot;

    Filter::Parameters parameters;
    parameters.gamma = 0.4;
    parameters.maxSpeed = UR5::velocityLimits();
    parameters.respectJointLimits = true;

    std::printf("\nMotionBenchMaker UR5, %d problems loaded, up to %d per scene\n",
                static_cast<int>(problems.size()), perScene);
    std::printf("voxel %.3f m, margin %.4f m + %.4f m filter buffer, stepSize %.3f s, "
                "range %.2f rad, %.1f s limit\n\n",
                voxel, margin,
                buffer < 0.0 ? Barrier::interpolationBuffer(
                                   sdf::GridSDF(problems.front().field(),
                                                UR5::reachableBounds(), voxel))
                             : buffer,
                stepSize, range, timeLimit);
    std::printf("baseline segment: %s\n\n",
                segmentFraction > 0.0 ? "tightened" : "OMPL default (0.01 of extent)");
    std::printf("  %-11s %8s %9s %10s %8s %10s %14s\n", "planner", "solved", "ms",
                "evals", "vertices", "worst clr", "unsafe/audited");

    std::map<std::string, Tally> tallies;
    std::map<std::string, int> seen;
    Tally overall;

    for (const Problem &problem : problems)
    {
        if (seen[problem.scene]++ >= perScene)
            continue;

        const sdf::GridSDF field(problem.field(), UR5::reachableBounds(), voxel);
        // The barrier the assertions use, and the thicker one the filter guards so that
        // auditing against the first one passes. See ClearanceBarrier::guarding().
        const Barrier audited(robot, field, margin);
        const Barrier guard = buffer < 0.0 ? Barrier::guarding(robot, field, margin)
                                           : Barrier::guarding(robot, field, margin, buffer);
        const Filter filter(guard, parameters);

        Tally &tally = tallies[problem.scene];
        ++tally.attempted;
        ++overall.attempted;

        const Barrier bare(robot, field, 0.0);
        const double endpoints =
            std::min(bare.worstValue(problem.start), bare.worstValue(problem.goal));
        tally.endpointClearance.push_back(endpoints);
        overall.endpointClearance.push_back(endpoints);

        // Valid upstream does not mean feasible here: our spheres do not enclose the
        // meshes MotionBenchMaker checked, and the margin sits on top of that.
        if (!audited.isSafe(problem.start) || !audited.isSafe(problem.goal))
        {
            ++tally.skipped;
            ++overall.skipped;
            continue;
        }

        const Result checked =
            runCollisionChecked(problem, audited, range, timeLimit, segmentFraction);
        const Result rolled = runFiltered(problem, audited, filter, stepSize, range, timeLimit);
        tally.add(0, checked);
        tally.add(1, rolled);
        overall.add(0, checked);
        overall.add(1, rolled);
    }

    for (const auto &entry : tallies)
    {
        const Tally &tally = entry.second;
        std::printf("\n%s  (%d problems, %d skipped: start or goal inside the margin)\n",
                    entry.first.c_str(), tally.attempted, tally.skipped);
        reportRow("rrtconnect", tally, 0);
        reportRow("cbf-rrtc", tally, 1);
    }

    // The feasibility question, since it decides how much of the benchmark is usable.
    std::printf("\nEndpoint clearance at zero margin -- min(start, goal) over all spheres.\n");
    std::printf("  %-20s %8s %9s %9s %9s   %s\n", "scene", "min", "median", "max",
                "affordable", "problems by margin");
    for (const auto &entry : tallies)
    {
        std::vector<double> clearances = entry.second.endpointClearance;
        if (clearances.empty())
            continue;
        std::sort(clearances.begin(), clearances.end());
        const auto share = [&clearances](double margin)
        {
            std::size_t n = 0;
            for (const double c : clearances)
                n += (c >= margin) ? 1 : 0;
            return 100.0 * static_cast<double>(n) / static_cast<double>(clearances.size());
        };
        std::printf("  %-20s %8.4f %9.4f %9.4f %9.4f   0:%3.0f%% 0.02:%3.0f%% 0.04:%3.0f%% "
                    "0.06:%3.0f%% 0.09:%3.0f%%\n",
                    entry.first.c_str(), clearances.front(), median(clearances), clearances.back(),
                    clearances.front(), share(0.0), share(0.02), share(0.04), share(0.06),
                    share(0.09));
    }

    std::printf("\nall scenes  (%d problems, %d skipped)\n", overall.attempted, overall.skipped);
    reportRow("rrtconnect", overall, 0);
    reportRow("cbf-rrtc", overall, 1);
    std::printf("\n\"evals\" is collision checks for rrtconnect and filter calls for cbf-rrtc.\n"
                "\"unsafe/audited\" evaluates the unbuffered barrier at every state of the\n"
                "densified solution -- for cbf-rrtc that densification is the rollout itself,\n"
                "so it is the motion that would actually be executed. Non-zero unsafe\n"
                "invalidates a row however fast it was.\n");
    return 0;
}
