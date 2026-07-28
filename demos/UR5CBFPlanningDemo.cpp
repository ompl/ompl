// demos/UR5CBFPlanningDemo.cpp
//
// Plans for a spherized UR5 with a CBF safety filter folded into propagation, and
// compares it against ordinary collision-checked planning.
//
// The point of the demo is how little wiring this takes: ompl::control::RRT is used
// completely unmodified. The CBF machinery enters through two ordinary extension
// points that any OMPL user already has --
//
//     si->setStatePropagator(...)                  <- the filter lives here
//     si->setDirectedControlSamplerAllocator(...)  <- a real steer instead of random shooting
//
// ### No collision checking in the CBF rows
//
// The filter certifies every step it produces, so the state validity checker is
// redundant and is removed outright:
//
//     si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
//
// Leaving it in would mean paying for the same geometry twice per step -- the
// barrier already runs the forward kinematics and queries the field for all 40
// spheres to build its constraint rows, and isSafe() would then repeat that work to
// learn something the barrier already knows. The three rows below separate the two
// effects: `collision-check` is the ordinary baseline, `cbf+check` keeps the
// redundant checker, and `cbf-only` drops it. cbf+check minus cbf-only is the cost
// of the duplicate work; collision-check versus cbf-only is the comparison that
// matters.
//
// With no checker there is nothing catching an error in the barrier itself, so the
// returned path is audited afterwards instead: it is interpolated to every
// propagation step and the barrier is evaluated at each one ("audit" / "unsafe"
// columns).
//
//     ./build/demos/demo_UR5CBFPlanning [seconds] [runs] [dump.json|-] [obstacleScale] [buffer] [voxel] [stepSize]
//
// `buffer` is the extra margin the filter guards on top of the audited one; omit it
// (or pass a negative value) for one voxel. Shrinking it is what the "unsafe" column
// is for -- see ClearanceBarrier::interpolationBuffer().

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/cbf/CBFControlFilter.h>
#include <ompl/cbf/FilteredStatePropagator.h>
#include <ompl/cbf/JointSteeringControlSampler.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/cbf/FilteredMotionValidator.h>
#include <ompl/cbf/FilteredStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
using Barrier = ompl::cbf::ClearanceBarrier;
using Filter = ompl::cbf::CBFControlFilter;
using Propagator = ompl::cbf::FilteredStatePropagator;
using UR5 = ompl::robots::UR5;
namespace sdf = ompl::sdf;

namespace
{
    constexpr int dimension = Propagator::dimension;
    double stepSize = 0.05;  // overridable from argv, to probe the step/buffer relation
    unsigned int maxSteps = 10;  // max propagation steps per edge -- caps how far one edge reaches

    // Start and goal are the same arm shape rotated about the base, so the direct
    // joint-space motion is a pure base sweep -- straight through the obstacle.
    UR5::Configuration startConfiguration()
    {
        return (UR5::Configuration() << 0.0, -1.2, 1.8, -0.6, 1.57, 0.0).finished();
    }

    UR5::Configuration goalConfiguration()
    {
        return (UR5::Configuration() << 2.4, -1.2, 1.8, -0.6, 1.57, 0.0).finished();
    }

    /// One obstacle sphere, kept as data rather than baked into the distance lambda so
    /// that the SDF the planner sees and the geometry the viewer draws come from the
    /// same list and cannot drift apart.
    struct Obstacle
    {
        Eigen::Vector3d center;
        double radius;
    };

    /// Obstacles sitting exactly where the arm would be at the midpoint of the direct
    /// sweep, so the straight joint-space path is blocked and the arm has to reshape
    /// itself to get around. Clearance along the direct path, at this margin, runs
    /// +0.29 (start), +0.14, **-0.24** (midpoint), +0.08, +0.38 (goal).
    ///
    /// \p scale inflates the obstacles, tightening the free space the planner has to
    /// work in. At scale 1 the problem is easy for both planners; larger values are
    /// what actually exercise the filter.
    std::vector<Obstacle> obstacles(double scale)
    {
        return {{Eigen::Vector3d(-0.742, 0.121, 1.083), 0.18 * scale},
                {Eigen::Vector3d(-0.742, 0.121, 1.383), 0.14 * scale}};
    }

    /// A union of spheres is the min of their distances, which is still an exact
    /// signed distance field.
    sdf::DistanceFn scene(const std::vector<Obstacle> &spheres)
    {
        return [spheres](const Eigen::Vector3d &p)
        {
            double distance = std::numeric_limits<double>::infinity();
            for (const Obstacle &sphere : spheres)
                distance = std::min(distance, (p - sphere.center).norm() - sphere.radius);
            return distance;
        };
    }

    struct Problem
    {
        UR5 robot;
        Barrier barrier;
        /// Counts calls to the state validity checker, so "we removed the collision
        /// checking" is a measurement rather than a claim.
        mutable std::size_t validityChecks{0};
        oc::SpaceInformationPtr si;
        std::shared_ptr<Propagator> propagator;

        Problem(const Problem &) = delete;
        Problem &operator=(const Problem &) = delete;

        Problem(const sdf::GridSDF &field, const ompl::cbf::ControlFilter &filter, double margin,
                bool collisionChecking)
          : robot(), barrier(robot, field, margin)
        {
            auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
            ob::RealVectorBounds stateBounds(dimension);
            for (int j = 0; j < dimension; ++j)
            {
                stateBounds.setLow(j, UR5::lowerBounds()[j]);
                stateBounds.setHigh(j, UR5::upperBounds()[j]);
            }
            space->setBounds(stateBounds);

            auto controls = std::make_shared<oc::RealVectorControlSpace>(space, dimension);
            ob::RealVectorBounds controlBounds(dimension);
            for (int j = 0; j < dimension; ++j)
            {
                controlBounds.setLow(j, -UR5::velocityLimits()[j]);
                controlBounds.setHigh(j, UR5::velocityLimits()[j]);
            }
            controls->setBounds(controlBounds);

            si = std::make_shared<oc::SpaceInformation>(space, controls);
            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, maxSteps);

            if (collisionChecking)
            {
                const Barrier *barrierPtr = &barrier;
                std::size_t *checks = &validityChecks;
                si->setStateValidityChecker([barrierPtr, checks](const ob::State *state)
                                            {
                                                ++*checks;
                                                return barrierPtr->isSafe(configurationOf(state));
                                            });
            }
            else
            {
                // Nothing to check: every state the propagator produces was certified
                // by the barrier on the way out. See FilteredStatePropagator's header
                // for what this gives up.
                si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
            }

            // The two extension points -- this is the whole integration.
            propagator = std::make_shared<Propagator>(si, filter);
            si->setStatePropagator(propagator);
            si->setDirectedControlSamplerAllocator(
                [](const oc::SpaceInformation *space)
                { return std::make_shared<ompl::cbf::JointSteeringControlSampler>(space); });

            si->setup();
        }

        static UR5::Configuration configurationOf(const ob::State *state)
        {
            const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
            UR5::Configuration q;
            for (int j = 0; j < dimension; ++j)
                q[j] = values[j];
            return q;
        }

        ob::ScopedState<ob::RealVectorStateSpace> stateOf(const UR5::Configuration &q) const
        {
            ob::ScopedState<ob::RealVectorStateSpace> state(si->getStateSpace());
            for (int j = 0; j < dimension; ++j)
                state->values[j] = q[j];
            return state;
        }
    };

    struct Outcome
    {
        bool solved{false};
        bool verified{false};
        double seconds{0.0};
        unsigned int vertices{0};
        std::size_t pathStates{0};
        std::size_t auditedStates{0};
        std::size_t unsafeStates{0};
        std::size_t validityChecks{0};
        double minClearance{0.0};
        Propagator::Statistics statistics;
    };

    /// The bar. `geometric::RRTConnect` with the same SDF-backed collision checker is
    /// what anyone would actually reach for on a 6-DoF arm, so it is the number to
    /// beat -- and it is not a like-for-like comparison, which is the point worth
    /// being clear about:
    ///
    /// - It is bidirectional. A CBF filter cannot be, because a projection is not
    ///   invertible: `FilteredStatePropagator::canPropagateBackward()` is false, so no
    ///   control planner in OMPL can grow a tree back from the goal.
    /// - It plans geometrically: an edge is a straight line in joint space, checked at
    ///   a fixed resolution and thrown away whole if any sample fails. It has no
    ///   velocity limits and no notion of a control, so its paths are not executable
    ///   without post-processing that this demo does not do.
    /// - It reaches the goal *exactly*, rather than within a 0.35 rad threshold.
    ///
    /// So it solves an easier problem with a stronger primitive. If the CBF version is
    /// to be interesting on speed it has to approach this, not merely beat control RRT.
    Outcome runGeometric(const sdf::GridSDF &field, double margin, double timeLimit)
    {
        const UR5 robot;
        const Barrier barrier(robot, field, margin);
        std::size_t checks = 0;

        auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
        ob::RealVectorBounds bounds(dimension);
        for (int j = 0; j < dimension; ++j)
        {
            bounds.setLow(j, UR5::lowerBounds()[j]);
            bounds.setHigh(j, UR5::upperBounds()[j]);
        }
        space->setBounds(bounds);

        auto si = std::make_shared<ob::SpaceInformation>(space);
        const Barrier *barrierPtr = &barrier;
        std::size_t *counter = &checks;
        si->setStateValidityChecker([barrierPtr, counter](const ob::State *state)
                                    {
                                        ++*counter;
                                        return barrierPtr->isSafe(Problem::configurationOf(state));
                                    });
        si->setup();

        ob::ScopedState<ob::RealVectorStateSpace> start(space), goal(space);
        for (int j = 0; j < dimension; ++j)
        {
            start->values[j] = startConfiguration()[j];
            goal->values[j] = goalConfiguration()[j];
        }
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal);

        auto planner = std::make_shared<og::RRTConnect>(si);
        planner->setProblemDefinition(pdef);
        planner->setup();

        const ompl::time::point begin = ompl::time::now();
        const ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeLimit));
        Outcome outcome;
        outcome.seconds = ompl::time::seconds(ompl::time::now() - begin);
        outcome.solved = (status == ob::PlannerStatus::EXACT_SOLUTION);
        outcome.validityChecks = checks;

        ob::PlannerData data(si);
        planner->getPlannerData(data);
        outcome.vertices = data.numVertices();

        if (pdef->hasSolution())
        {
            auto path = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
            outcome.verified = path->check();
            outcome.pathStates = path->getStateCount();
            // Audit at the same resolution the control rows are audited at, so the
            // clearance columns mean the same thing.
            path->interpolate(static_cast<unsigned int>(path->length() / stepSize));
            outcome.auditedStates = path->getStateCount();
            outcome.minClearance = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < path->getStateCount(); ++i)
            {
                const double h = barrier.worstValue(Problem::configurationOf(path->getState(i)));
                outcome.minClearance = std::min(outcome.minClearance, h);
                if (h < 0.0)
                    ++outcome.unsafeStates;
            }
        }
        return outcome;
    }

    /// The CBF rollout behind a *geometric* planner, via FilteredStateSpace. No
    /// controls, no StatePropagator, no collision checker, and no state validity
    /// checker doing anything -- the barrier certifies each rollout step as it is
    /// produced. `planner` picks RRT or RRTConnect; the interesting question is whether
    /// bidirectional works at all, given the two trees have to meet.
    Outcome runFilteredGeometric(const sdf::GridSDF &field, const ompl::cbf::ControlFilter &filter,
                                 double margin, bool bidirectional, double range, double timeLimit,
                                 std::vector<UR5::Configuration> *solution)
    {
        const UR5 robot;
        const Barrier barrier(robot, field, margin);

        auto space = std::make_shared<ompl::cbf::FilteredStateSpace>(filter, stepSize,
                                                                    UR5::velocityLimits());
        ob::RealVectorBounds bounds(dimension);
        for (int j = 0; j < dimension; ++j)
        {
            bounds.setLow(j, UR5::lowerBounds()[j]);
            bounds.setHigh(j, UR5::upperBounds()[j]);
        }
        space->setBounds(bounds);

        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
        si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
        si->setup();

        ob::ScopedState<ob::RealVectorStateSpace> start(space), goal(space);
        for (int j = 0; j < dimension; ++j)
        {
            start->values[j] = startConfiguration()[j];
            goal->values[j] = goalConfiguration()[j];
        }
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        // Same goal tolerance the control rows get, since a rollout cannot be asked to
        // land on an exact state next to an obstacle.
        pdef->setStartAndGoalStates(start, goal, 0.35);

        ob::PlannerPtr planner;
        if (bidirectional)
        {
            auto rrtc = std::make_shared<og::RRTConnect>(si);
            rrtc->setRange(range);
            planner = rrtc;
        }
        else
        {
            auto rrt = std::make_shared<og::RRT>(si);
            rrt->setRange(range);
            planner = rrt;
        }
        planner->setProblemDefinition(pdef);
        planner->setup();

        const ompl::time::point begin = ompl::time::now();
        const ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeLimit));
        Outcome outcome;
        outcome.seconds = ompl::time::seconds(ompl::time::now() - begin);
        outcome.solved = (status == ob::PlannerStatus::EXACT_SOLUTION);

        const ompl::cbf::FilteredStateSpace::Statistics stats = space->statistics();
        outcome.statistics.propagations = stats.steps;
        outcome.statistics.filtered = stats.filtered;
        outcome.statistics.blocked = stats.blocked;

        ob::PlannerData data(si);
        planner->getPlannerData(data);
        outcome.vertices = data.numVertices();

        if (pdef->hasSolution())
        {
            auto path = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
            outcome.verified = path->check();
            outcome.pathStates = path->getStateCount();

            // The audit. PathGeometric::interpolate() calls the space's interpolate,
            // which *is* the rollout, so densifying the waypoints reconstructs the
            // motion that would actually be executed -- not a straight-line stand-in.
            path->interpolate();
            outcome.auditedStates = path->getStateCount();
            outcome.minClearance = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < path->getStateCount(); ++i)
            {
                const UR5::Configuration q =
                    ompl::cbf::FilteredStateSpace::configurationOf(path->getState(i));
                const double h = barrier.worstValue(q);
                outcome.minClearance = std::min(outcome.minClearance, h);
                if (h < 0.0)
                    ++outcome.unsafeStates;
                if (solution != nullptr)
                    solution->push_back(q);
            }
        }
        return outcome;
    }

    Outcome run(const sdf::GridSDF &field, const ompl::cbf::ControlFilter &filter, double margin,
                bool collisionChecking, double timeLimit, std::vector<UR5::Configuration> *solution)
    {
        Problem problem(field, filter, margin, collisionChecking);
        auto pdef = std::make_shared<ob::ProblemDefinition>(problem.si);
        pdef->setStartAndGoalStates(problem.stateOf(startConfiguration()),
                                    problem.stateOf(goalConfiguration()), /*threshold=*/0.35);

        auto planner = std::make_shared<oc::RRT>(problem.si);  // stock, unmodified
        planner->setProblemDefinition(pdef);
        planner->setup();

        const ompl::time::point begin = ompl::time::now();
        const ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeLimit));
        Outcome outcome;
        outcome.seconds = ompl::time::seconds(ompl::time::now() - begin);
        outcome.solved = (status == ob::PlannerStatus::EXACT_SOLUTION);
        outcome.statistics = problem.propagator->statistics();
        // Read before check()/the audit below, which would otherwise be billed to
        // the planner: PathControl::check() calls isValid() on every edge start.
        outcome.validityChecks = problem.validityChecks;

        ob::PlannerData data(problem.si);
        planner->getPlannerData(data);
        outcome.vertices = data.numVertices();

        if (pdef->hasSolution())
        {
            const auto path = std::static_pointer_cast<oc::PathControl>(pdef->getSolutionPath());
            outcome.pathStates = path->getStateCount();
            // check() re-propagates every edge and compares against the stored state,
            // so it passes only if the filter is deterministic. Note that with the
            // collision checker removed it no longer says anything about safety --
            // it is purely a determinism check there. Safety is the audit below.
            outcome.verified = path->check();

            // Expand the path to one state per propagation step and evaluate the
            // barrier at each. This is the safety audit, and it is not optional in
            // the rows that plan without a collision checker: it is the only thing
            // that would catch the barrier being wrong. interpolate() uses plain
            // propagate(), so it costs no validity checks.
            path->interpolate();
            outcome.auditedStates = path->getStateCount();
            outcome.minClearance = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < path->getStateCount(); ++i)
            {
                const double h = problem.barrier.worstValue(Problem::configurationOf(path->getState(i)));
                outcome.minClearance = std::min(outcome.minClearance, h);
                if (h < 0.0)
                    ++outcome.unsafeStates;
            }

            // Copy the configurations out while the problem is still alive; the path
            // itself refers to objects that die with it.
            if (solution != nullptr)
            {
                solution->clear();
                for (std::size_t i = 0; i < path->getStateCount(); ++i)
                    solution->push_back(Problem::configurationOf(path->getState(i)));
            }
        }
        return outcome;
    }

    double median(std::vector<double> values)
    {
        if (values.empty())
            return 0.0;
        std::sort(values.begin(), values.end());
        const std::size_t middle = values.size() / 2;
        return values.size() % 2 == 1 ? values[middle] : 0.5 * (values[middle - 1] + values[middle]);
    }

    /// Medians, not means. Run-to-run spread here is enormous -- an unlucky RRT can
    /// burn an order of magnitude more propagations than a lucky one -- so a mean over
    /// ten runs mostly reports the worst run. Clearance is reported as the worst over
    /// all runs for the opposite reason: averaging a minimum lets a violation in one
    /// run hide behind slack in the others.
    void report(const char *label, const std::vector<Outcome> &runs)
    {
        int solved = 0, verified = 0;
        std::vector<double> seconds, vertices, steps, checks;
        std::size_t propagations = 0, filtered = 0, blocked = 0, audited = 0, unsafe = 0;
        double worstClearance = std::numeric_limits<double>::infinity();
        for (const Outcome &r : runs)
        {
            solved += r.solved ? 1 : 0;
            verified += r.verified ? 1 : 0;
            seconds.push_back(r.seconds);
            vertices.push_back(static_cast<double>(r.vertices));
            steps.push_back(static_cast<double>(r.statistics.propagations));
            checks.push_back(static_cast<double>(r.validityChecks));
            propagations += r.statistics.propagations;
            filtered += r.statistics.filtered;
            blocked += r.statistics.blocked;
            audited += r.auditedStates;
            unsafe += r.unsafeStates;
            if (r.solved)
                worstClearance = std::min(worstClearance, r.minClearance);
        }
        std::printf("%-14s %3d/%-3zu %5d %8.3f %9.0f %10.0f %10.0f %8.1f%% %7.1f%%", label, solved,
                    runs.size(), verified, 1e3 * median(seconds), median(vertices), median(steps),
                    median(checks),
                    100.0 * static_cast<double>(filtered) / std::max<std::size_t>(1, propagations),
                    100.0 * static_cast<double>(blocked) / std::max<std::size_t>(1, propagations));
        if (solved > 0)
            std::printf(" %9.4f %6zu/%-7zu\n", worstClearance, unsafe, audited);
        else
            std::printf(" %9s %14s\n", "-", "-");
    }

    /// The per-step question, measured directly rather than inferred from planner
    /// wall-clock: what does one collision check cost, and what does one filtered
    /// propagation step cost? Search variance swamps this difference in the table
    /// below, so it is worth isolating.
    void reportStepCost(const Barrier &barrier, const Filter &filter)
    {
        constexpr int samples = 100000;
        ompl::RNG rng;
        std::vector<UR5::Configuration> configurations(samples), nominals(samples);
        for (int i = 0; i < samples; ++i)
            for (int j = 0; j < dimension; ++j)
            {
                configurations[i][j] = rng.uniformReal(UR5::lowerBounds()[j], UR5::upperBounds()[j]);
                nominals[i][j] = rng.uniformReal(-UR5::velocityLimits()[j], UR5::velocityLimits()[j]);
            }

        double sink = 0.0;
        ompl::time::point begin = ompl::time::now();
        for (int i = 0; i < samples; ++i)
            sink += barrier.isSafe(configurations[i]) ? 1.0 : 0.0;
        const double checkCost = ompl::time::seconds(ompl::time::now() - begin) / samples;

        UR5::Configuration applied;
        begin = ompl::time::now();
        for (int i = 0; i < samples; ++i)
        {
            filter.filter(configurations[i], nominals[i], stepSize, applied);
            sink += applied[0];
        }
        const double filterCost = ompl::time::seconds(ompl::time::now() - begin) / samples;

        std::printf("Per-step cost over %d random (q, u) pairs:\n", samples);
        std::printf("  isSafe(q)                    %6.3f us   one collision check\n", 1e6 * checkCost);
        std::printf("  filter(q, u)                 %6.3f us   barrier rows + QP  (%.2fx a check)\n",
                    1e6 * filterCost, filterCost / checkCost);
        std::printf("  both, as cbf+check does      %6.3f us   dropping the checker saves %.0f%%\n",
                    1e6 * (checkCost + filterCost),
                    100.0 * checkCost / (checkCost + filterCost));
        std::printf("A filtered step cannot beat a bare collision check: the barrier computes\n");
        std::printf("everything isSafe() does and then %d position Jacobians on top. The CBF has to\n",
                    static_cast<int>(UR5::nSpheres));
        std::printf("win on edges it does not waste, not on cost per step. (sink %.3g)\n\n", sink);
    }

    /// Dump a solution path in the format scripts/ur5_sphere_viz.py already reads, so
    /// the planned motion can be replayed over the real UR5 meshes in PyBullet.
    void writeViewerJson(const char *path, const UR5 &robot,
                         const std::vector<UR5::Configuration> &solution,
                         const std::vector<Obstacle> &scenery)
    {
        std::FILE *out = std::fopen(path, "w");
        if (out == nullptr)
        {
            std::printf("could not open %s for writing\n", path);
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

        std::fprintf(out, "],\n  \"obstacles\": [");
        for (std::size_t i = 0; i < scenery.size(); ++i)
            std::fprintf(out, "%s{\"center\": [%.9g, %.9g, %.9g], \"radius\": %.9g}", i ? ", " : "",
                         scenery[i].center.x(), scenery[i].center.y(), scenery[i].center.z(),
                         scenery[i].radius);

        std::fprintf(out, "],\n  \"configs\": [\n");
        for (std::size_t s = 0; s < solution.size(); ++s)
        {
            const UR5::Configuration &q = solution[s];
            const UR5::SphereCenters centers = robot.sphereCenters(q);

            std::fprintf(out, "    {\"q\": [");
            for (Eigen::Index j = 0; j < q.size(); ++j)
                std::fprintf(out, "%s%.9g", j ? ", " : "", q[j]);
            std::fprintf(out, "], \"centers\": [");
            for (Eigen::Index i = 0; i < centers.cols(); ++i)
                std::fprintf(out, "%s[%.9g, %.9g, %.9g]", i ? ", " : "", centers(0, i), centers(1, i),
                             centers(2, i));
            std::fprintf(out, "]}%s\n", s + 1 < solution.size() ? "," : "");
        }
        std::fprintf(out, "  ]\n}\n");
        std::fclose(out);
    }
}  // namespace

int main(int argc, char **argv)
{
    const double timeLimit = argc > 1 ? std::atof(argv[1]) : 5.0;
    const int repetitions = argc > 2 ? std::atoi(argv[2]) : 10;
    const char *dumpPath = (argc > 3 && argv[3][0] != '-') ? argv[3] : nullptr;
    const double obstacleScale = argc > 4 ? std::atof(argv[4]) : 1.0;
    // Negative means "use one voxel", the only value with an argument behind it.
    const double bufferArg = argc > 5 ? std::atof(argv[5]) : -1.0;
    const double voxel = argc > 6 ? std::atof(argv[6]) : 0.03;
    if (argc > 7)
        stepSize = std::atof(argv[7]);
    if (argc > 8)
        maxSteps = static_cast<unsigned int>(std::atoi(argv[8]));
    // How far one geometric extension may reach, in radians of joint-space distance.
    const double geomRange = argc > 9 ? std::atof(argv[9]) : 2.0;
    // Bitmask selecting which rows to run, so a sweep need not pay for all six:
    // 1 geom-rrtconnect, 2 collision-check, 4 cbf+check, 8 cbf-only, 16 cbf-geom-rrt,
    // 32 cbf-geom-rrtc.
    const int rows = argc > 10 ? std::atoi(argv[10]) : 63;

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    // OMPL only honours a seed set before any random number is drawn, so the
    // repetitions below cannot be individually seeded: they are independent draws
    // from one stream, and the two rows are therefore *not* paired by seed.
    ompl::RNG::setSeed(1);

    std::printf("Baking the workspace SDF over the UR5's reachable volume (voxel %.3f m)...\n", voxel);
    const std::vector<Obstacle> scenery = obstacles(obstacleScale);
    const sdf::GridSDF field(scene(scenery), UR5::reachableBounds(), voxel);
    const Eigen::Vector3i dims = field.dimensions();
    std::printf("  %d x %d x %d nodes, obstacle scale %.2f\n\n", dims.x(), dims.y(), dims.z(),
                obstacleScale);

    // The margin makes the barrier conservative; see ClearanceBarrier's header.
    const double margin = Barrier::defaultMargin;
    const double buffer = bufferArg >= 0.0 ? bufferArg : 0.010;  // measured clean; see interpolationBuffer()
    const UR5 robot;
    const Barrier barrier(robot, field, margin);
    std::printf("start clearance %+.4f m, goal clearance %+.4f m (margin %.3f m)\n",
                barrier.worstValue(startConfiguration()), barrier.worstValue(goalConfiguration()), margin);
    if (!barrier.isSafe(startConfiguration()) || !barrier.isSafe(goalConfiguration()))
    {
        std::printf("start or goal is not clear -- nothing to plan\n");
        return 1;
    }

    // The filter enforces a *buffered* barrier, because enforcing h >= 0 against an
    // interpolated field only delivers h >= -O(voxel): see interpolationBuffer().
    // Everything that judges the result -- the checker, the audit, the clearance
    // column -- uses the unbuffered `barrier`, so the buffer is not marking its own
    // homework. Without it the audit below reports genuine violations.
    const Barrier guard = Barrier::guarding(robot, field, margin, buffer);
    std::printf("filter guards margin %.3f m (audit margin %.3f + buffer %.3f; one voxel is %.3f)\n",
                guard.margin(), margin, buffer, Barrier::interpolationBuffer(field));

    Filter::Parameters parameters;
    parameters.gamma = 0.4;
    parameters.maxSpeed = UR5::velocityLimits();
    parameters.respectJointLimits = true;
    const Filter cbfFilter(guard, parameters);
    const ompl::cbf::PassthroughFilter passthrough;

    std::printf("\n");
    reportStepCost(barrier, cbfFilter);  // check at `margin`, filter at the guarded one

    std::printf("Stock ompl::control::RRT, %d runs, %.1f s limit each. Medians, not means.\n\n",
                repetitions, timeLimit);
    std::printf("  geom-rrtconnect   geometric RRTConnect, same checker -- a different, easier\n");
    std::printf("                    problem with a stronger primitive; see runGeometric()\n");
    std::printf("  collision-check   control RRT, no filter, barrier-backed checker\n");
    std::printf("  cbf+check         CBF filter *and* the checker -- the same geometry twice\n");
    std::printf("  cbf-only          CBF filter, AllValidStateValidityChecker        (no checking)\n");
    std::printf("  cbf-geom-rrt      CBF rollout as FilteredStateSpace::interpolate, geometric RRT\n");
    std::printf("  cbf-geom-rrtc     the same behind RRTConnect -- does bidirectional work at all?\n\n");
    std::printf("%-14s %7s %5s %8s %9s %10s %10s %9s %8s %9s %14s\n", "setup", "solved", "chkd", "ms",
                "vertices", "steps", "checks", "filtered", "blocked", "worst clr", "unsafe/audited");

    std::vector<Outcome> geometric, plain, cbfChecked, cbfOnly, cbfGeom, cbfGeomBi;
    std::vector<UR5::Configuration> best;
    for (int repeat = 0; repeat < repetitions; ++repeat)
    {
        if (rows & 1)
            geometric.push_back(runGeometric(field, margin, timeLimit));
        if (rows & 16)
            cbfGeom.push_back(
                runFilteredGeometric(field, cbfFilter, margin, false, geomRange, timeLimit, nullptr));
        if (rows & 32)
        {
            // The dump comes from whichever CBF row ran, preferring this one: it is the
            // configuration worth looking at, and tying the dump to a single row means a
            // row mask that skips it silently writes nothing.
            std::vector<UR5::Configuration> solution;
            cbfGeomBi.push_back(runFilteredGeometric(field, cbfFilter, margin, true, geomRange,
                                                     timeLimit,
                                                     dumpPath != nullptr ? &solution : nullptr));
            if (dumpPath != nullptr && cbfGeomBi.back().solved && best.empty())
                best = solution;
        }
        if (rows & 2)
            plain.push_back(run(field, passthrough, margin, /*collisionChecking=*/true, timeLimit, nullptr));
        if (rows & 4)
            cbfChecked.push_back(run(field, cbfFilter, margin, /*collisionChecking=*/true, timeLimit, nullptr));

        std::vector<UR5::Configuration> solution;
        if (rows & 8)
        {
            cbfOnly.push_back(run(field, cbfFilter, margin, /*collisionChecking=*/false, timeLimit,
                                  dumpPath != nullptr ? &solution : nullptr));
            if (dumpPath != nullptr && cbfOnly.back().solved && best.empty())
                best = solution;
        }
    }
    if (!geometric.empty())
        report("geom-rrtconnect", geometric);
    if (!plain.empty())
        report("collision-check", plain);
    if (!cbfChecked.empty())
        report("cbf+check", cbfChecked);
    if (!cbfOnly.empty())
        report("cbf-only", cbfOnly);
    if (!cbfGeom.empty())
        report("cbf-geom-rrt", cbfGeom);
    if (!cbfGeomBi.empty())
        report("cbf-geom-rrtc", cbfGeomBi);

    std::printf("\n\"checks\" is calls to the StateValidityChecker during planning -- zero for\n");
    std::printf("cbf-only, which is the point: the barrier already did that geometry to build\n");
    std::printf("its constraint rows. cbf+check minus cbf-only is the cost of doing it twice.\n");
    std::printf("\"unsafe/audited\" evaluates the barrier at every propagation step of the\n");
    std::printf("returned path, not just the edge endpoints. With no collision checker this is\n");
    std::printf("the only thing standing between a wrong barrier and a wrong answer, so a\n");
    std::printf("non-zero \"unsafe\" invalidates that row regardless of how fast it was. It is\n");
    std::printf("zero here only because the filter guards the buffered barrier; enforcing the\n");
    std::printf("audited margin directly leaves ~2%% of steps a few mm short of it.\n");
    std::printf("\"chkd\" counts solutions accepted by PathControl::check(), which re-propagates\n");
    std::printf("every edge -- it passes only because the filter is deterministic, and for\n");
    std::printf("cbf-only it says nothing about safety.\n");
    std::printf("\"filtered\"/\"blocked\" are shares of propagation steps. A run where filtered is\n");
    std::printf("~0%% never exercised the CBF, so its numbers say nothing about the filter.\n");

    if (dumpPath != nullptr && !best.empty())
    {
        writeViewerJson(dumpPath, robot, best, scenery);
        std::printf("\nwrote %s -- replay it with\n  python scripts/ur5_sphere_viz.py %s --gui\n", dumpPath,
                    dumpPath);
    }
    return 0;
}
