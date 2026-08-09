#define BOOST_TEST_MODULE FilteredStateSpaceTest
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/cbf/CBFControlFilter.h>
#include <ompl/cbf/ExecutedPath.h>
#include <ompl/cbf/FilteredMotionValidator.h>
#include <ompl/cbf/FilteredStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using Barrier = ompl::cbf::ClearanceBarrier;
using Filter = ompl::cbf::CBFControlFilter;
using Space = ompl::cbf::FilteredStateSpace;
using UR5 = ompl::robots::UR5;
namespace sdf = ompl::sdf;

namespace
{
    constexpr double stepSize = 0.05;
    constexpr double voxel = 0.03;
    constexpr int dim = Space::dimension;

    sdf::DistanceFn sphereField(const Eigen::Vector3d &center, double radius)
    {
        return [center, radius](const Eigen::Vector3d &p) { return (p - center).norm() - radius; };
    }

    /// An obstacle the arm has to work around.
    const sdf::GridSDF &obstacleField()
    {
        static const sdf::GridSDF field(sphereField(Eigen::Vector3d(0.3, 0.3, 1.1), 0.2),
                                        UR5::reachableBounds(), voxel);
        return field;
    }

    /// An obstacle nowhere near the robot, so the filter never engages and the space
    /// must behave exactly like a RealVectorStateSpace.
    const sdf::GridSDF &emptyField()
    {
        static const sdf::GridSDF field(sphereField(Eigen::Vector3d(8.0, 8.0, 8.0), 0.1),
                                        UR5::reachableBounds(), voxel);
        return field;
    }

    /// A field swallowing the base sphere, which no joint can move. The filter has
    /// nothing safe to offer anywhere, so every rollout is stillborn.
    const sdf::GridSDF &corneringField()
    {
        static const sdf::GridSDF field(
            sphereField(UR5().sphereCenters(UR5::Configuration::Zero()).col(0), 0.5),
            UR5::reachableBounds(), voxel);
        return field;
    }

    Filter::Parameters filterParameters()
    {
        Filter::Parameters p;
        p.gamma = 0.4;
        p.maxSpeed = UR5::velocityLimits();
        p.respectJointLimits = true;
        return p;
    }

    std::shared_ptr<Space> makeSpace(const ompl::cbf::ControlFilter &filter)
    {
        auto space = std::make_shared<Space>(filter, stepSize, UR5::velocityLimits());
        ob::RealVectorBounds bounds(dim);
        for (int j = 0; j < dim; ++j)
        {
            bounds.setLow(j, UR5::lowerBounds()[j]);
            bounds.setHigh(j, UR5::upperBounds()[j]);
        }
        space->setBounds(bounds);
        return space;
    }

    UR5::Configuration configuration(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        return (UR5::Configuration() << q0, q1, q2, q3, q4, q5).finished();
    }

    bool straightLineBlocked(const Barrier &barrier, const UR5::Configuration &from,
                             const UR5::Configuration &to)
    {
        constexpr int samples = 40;
        for (int i = 0; i <= samples; ++i)
            if (!barrier.isSafe(from + (to - from) * (static_cast<double>(i) / samples)))
                return true;
        return false;
    }

    /// A target on the far side of the obstacle: clear itself, but with the straight
    /// joint-space line to it passing through the obstacle, so reaching it requires
    /// going around. Searched rather than hardcoded — the arm's azimuth runs opposite to
    /// q0 and the clearance profile along a base sweep is not obvious by inspection, so a
    /// hand-picked sweep tends to land either short of the obstacle or inside it.
    ///
    /// Of the candidates, the *roomiest* is chosen rather than the first. The first clear
    /// sweep past the obstacle hugs its boundary, and a CBF filter actively pushes away
    /// from there — a goal sitting in the one place the method avoids makes for a test
    /// about goal tolerances, not about planning.
    ///
    /// Roomiest means roomiest *from the obstacle*, so the ranking reads the world
    /// clearances rather than `worstValue()`. It has to: this sweep turns joint 0 and
    /// nothing else, and joint 0 carries the whole arm rigidly, so it cannot change the
    /// distance between any two spheres on it. Ranking by `worstValue()` would compare
    /// candidates on a self-collision clearance that is the same constant at every one of
    /// them, and pick an arbitrary sweep — as it happens, one hugging the obstacle.
    UR5::Configuration pastObstacle(const Barrier &barrier, const UR5::Configuration &from)
    {
        const Barrier::Evaluation evaluation = barrier.evaluate(from);
        const double sign =
            evaluation.rows(static_cast<Eigen::Index>(evaluation.worst), 0) > 0.0 ? -1.0 : 1.0;

        UR5::Configuration best = from;
        double roomiest = -std::numeric_limits<double>::infinity();
        for (double sweep = 0.4; sweep <= 3.0; sweep += 0.05)
        {
            UR5::Configuration to = from;
            to[0] += sign * sweep;
            const double clearance = barrier.values(to).head<Barrier::nSpheres>().minCoeff();
            if (clearance > roomiest && barrier.isSafe(to) && straightLineBlocked(barrier, from, to))
            {
                roomiest = clearance;
                best = to;
            }
        }
        if (roomiest == -std::numeric_limits<double>::infinity())
            BOOST_FAIL("no target found that is clear but unreachable in a straight line");
        return best;
    }

    /// A target whose straight line is blocked but which the filter still *arrives* at,
    /// by a visibly bent path. The ledger tests need all three: several steps to record,
    /// an arrival for the record to be keyed on, and a bend — a straight edge cannot tell
    /// per-step interpolation apart from lerping the whole thing, so it would pass
    /// against a broken implementation.
    ///
    /// `pastObstacle()` can no longer supply one, and the reason is worth stating. It
    /// sweeps joint 0 alone, and getting round this obstacle that way needs the arm to
    /// fold down towards its own base — which the self-collision rows now forbid, rightly:
    /// spheres 0 and 2 reach 23 mm of overlap somewhere along that route. Every sweep in
    /// that family now slides along the obstacle instead of arriving.
    ///
    /// So this one moves all six joints. It was found by search and is pinned here rather
    /// than re-searched, because a search seeded off the RNG makes the test's subject
    /// depend on RNG behaviour. The properties the search was for are asserted at each
    /// use, so if one stops holding the test says which.
    UR5::Configuration deflectedArrival()
    {
        return configuration(-0.96, -1.85, 0.40, -2.84, -2.68, 1.90);
    }
}  // namespace

// The property that makes this a drop-in state space: with nothing in the way, the
// rollout *is* linear interpolation. The nominal control re-aims at the target every
// step, which in free space reproduces exactly |b - a| / N of travel per step, so the
// full rollout lands on b to floating-point precision rather than merely near it.
BOOST_AUTO_TEST_CASE(FreeSpaceRolloutIsExactlyLinearInterpolation)
{
    const UR5 robot;
    const Barrier barrier(robot, emptyField(), 0.0);
    const Filter filter(barrier, filterParameters());
    auto space = makeSpace(filter);

    const UR5::Configuration a = configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0);
    const UR5::Configuration b = configuration(1.4, -0.7, 1.2, -0.2, 1.20, 0.5);

    BOOST_CHECK_EQUAL(space->roll(a, b, 1.0).filtered, 0u);
    BOOST_CHECK(space->roll(a, b, 1.0).reachedTarget);
    BOOST_CHECK_LE((space->roll(a, b, 1.0).end - b).norm(), 1e-9);

    // Intermediate fractions match the straight line to within the quantisation of
    // one step, which is all the discretisation allows.
    const unsigned int total = space->horizonSteps(a, b);
    const double stepTravel = (b - a).norm() / total;
    for (const double t : {0.25, 0.5, 0.75})
    {
        const UR5::Configuration rolled = space->roll(a, b, t).end;
        BOOST_CHECK_LE((rolled - (a + t * (b - a))).norm(), stepTravel);
    }
}

// The horizon is the time the straight-line motion needs at the per-joint speed
// limit, so the slowest joint sets it.
BOOST_AUTO_TEST_CASE(HorizonIsSetByTheSlowestJoint)
{
    const UR5 robot;
    const Barrier barrier(robot, emptyField(), 0.0);
    const Filter filter(barrier, filterParameters());
    auto space = makeSpace(filter);

    const UR5::Configuration a = UR5::Configuration::Zero();
    UR5::Configuration b = UR5::Configuration::Zero();
    b[2] = 1.0;  // 1 rad at 0.5 rad/s = 2 s = 40 steps of 0.05 s
    BOOST_CHECK_EQUAL(space->horizonSteps(a, b), 40u);

    b[4] = 0.5;  // a shorter move on another joint must not change the horizon
    BOOST_CHECK_EQUAL(space->horizonSteps(a, b), 40u);

    BOOST_CHECK_EQUAL(space->horizonSteps(a, a), 0u);
}

// The value proposition, in one test: aim along a line that is blocked, and the rollout
// leaves the line rather than dying on it. A straight-line steer here has nothing to
// offer the planner but a rejection.
//
// Whether it ultimately arrives is deliberately not asserted -- if the CBF manages to
// work all the way around, so much the better. What must hold is that it departs from
// the blocked line, gets somewhere, and is clear at every step.
BOOST_AUTO_TEST_CASE(RolloutLeavesABlockedLineInsteadOfDyingOnIt)
{
    const UR5 robot;
    const Barrier truth(robot, obstacleField(), 0.0);
    // The filter guards a buffered barrier; the assertions use the unbuffered one. See
    // ClearanceBarrier::guarding() -- enforcing h >= 0 does not deliver h >= 0.
    const Barrier guard = Barrier::guarding(robot, obstacleField(), 0.0);
    const Filter filter(guard, filterParameters());
    auto space = makeSpace(filter);

    const UR5::Configuration a = UR5::Configuration::Zero();
    const UR5::Configuration b = pastObstacle(truth, a);
    BOOST_REQUIRE(straightLineBlocked(truth, a, b));

    const Space::Rollout rollout = space->roll(a, b, 1.0);
    BOOST_CHECK_GT(rollout.filtered, 0u);
    BOOST_CHECK_GT((rollout.end - a).norm(), 0.1);

    // Every state along the way is clear, and somewhere along it the rollout is well
    // off the straight line it was aimed down. Read from the rollout's own waypoints:
    // those *are* the states it passed through, and asking roll() for each fraction
    // separately would re-derive the same trajectory from scratch every time.
    const unsigned int total = space->horizonSteps(a, b);
    BOOST_REQUIRE_EQUAL(rollout.waypoints.size(), rollout.steps + 1u);
    BOOST_CHECK_EQUAL((rollout.waypoints.front() - a).norm(), 0.0);
    BOOST_CHECK_EQUAL((rollout.waypoints.back() - rollout.end).norm(), 0.0);

    double deviation = 0.0;
    for (std::size_t i = 0; i < rollout.waypoints.size(); ++i)
    {
        const double t = static_cast<double>(i) / total;
        BOOST_CHECK(truth.isSafe(rollout.waypoints[i]));
        deviation = std::max(deviation, (rollout.waypoints[i] - (a + t * (b - a))).norm());
    }
    BOOST_CHECK_GT(deviation, 0.05);
}

// roll() is a pure function of its inputs. Less load-bearing than it used to be -- the
// executed motion is now kept rather than re-derived -- but the fallback path in
// FilteredMotionValidator still rolls, and a rollout that drifted between identical calls
// would make the ledger's contents depend on call order.
BOOST_AUTO_TEST_CASE(RolloutIsDeterministic)
{
    const UR5 robot;
    const Barrier truth(robot, obstacleField(), 0.0);
    const Barrier guard = Barrier::guarding(robot, obstacleField(), 0.0);
    const Filter filter(guard, filterParameters());
    auto space = makeSpace(filter);

    const UR5::Configuration a = UR5::Configuration::Zero();
    const Space::Rollout first = space->roll(a, pastObstacle(truth, a), 1.0);
    const UR5::Configuration b = pastObstacle(truth, a);
    for (int repeat = 0; repeat < 4; ++repeat)
    {
        const Space::Rollout again = space->roll(a, b, 1.0);
        BOOST_CHECK_EQUAL((again.end - first.end).norm(), 0.0);
        BOOST_REQUIRE_EQUAL(again.waypoints.size(), first.waypoints.size());
        for (std::size_t i = 0; i < again.waypoints.size(); ++i)
            BOOST_CHECK_EQUAL((again.waypoints[i] - first.waypoints[i]).norm(), 0.0);
    }
}

// A rollout with nothing safe available anywhere stops on its first step, and it does so
// again next time -- the filter sees the identical state and nominal control. The refusal
// is re-paid now (one filter call), because a rollout that went nowhere is not an edge and
// is deliberately not recorded.
BOOST_AUTO_TEST_CASE(ABlockedRolloutStopsAtTheBlock)
{
    const UR5 robot;
    const Barrier barrier(robot, corneringField(), 0.0);
    const Filter filter(barrier, filterParameters());
    auto space = makeSpace(filter);

    const UR5::Configuration a = UR5::Configuration::Zero();
    const UR5::Configuration b = configuration(1.5, 0.0, 0.0, 0.0, 0.0, 0.0);

    const Space::Rollout first = space->roll(a, b, 0.5);
    BOOST_REQUIRE_EQUAL(first.steps, 0u);
    BOOST_REQUIRE_EQUAL(first.blocked, 1u);
    BOOST_CHECK_EQUAL(first.waypoints.size(), 1u);
    const std::size_t calls = space->statistics().steps;

    const Space::Rollout second = space->roll(a, b, 1.0);
    BOOST_CHECK_EQUAL(second.steps, 0u);
    BOOST_CHECK_EQUAL(second.blocked, 1u);
    BOOST_CHECK_EQUAL(space->statistics().steps, calls + 1);
    BOOST_CHECK_EQUAL(space->statistics().blocked, 2u);

    // Nothing was produced, so there is nothing to keep.
    BOOST_CHECK_EQUAL(space->ledgerEdges(), 0u);
}

// Densifying a recorded edge costs no filter calls at all: the trajectory is on file, so
// PathGeometric::interpolate() is reading it back rather than recomputing it.
//
// Note what has to happen first. An edge enters the ledger when the planner adopts it --
// here, when the validator accepts the motion. Densifying an edge nobody ever validated is
// still O(n^2) filter calls, which is why executedPath() is the supported way to expand a
// solution.
BOOST_AUTO_TEST_CASE(DensifyingARecordedEdgeCostsNoFilterCalls)
{
    const UR5 robot;
    const Barrier barrier(robot, emptyField(), 0.0);
    const Filter filter(barrier, filterParameters());
    auto space = makeSpace(filter);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
    si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
    si->setup();

    const UR5::Configuration a = configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0);
    const UR5::Configuration b = configuration(1.4, -0.7, 1.2, -0.2, 1.20, 0.5);

    ob::ScopedState<> from(space), to(space);
    Space::setState(from.get(), a);
    Space::setState(to.get(), b);

    og::PathGeometric path(si);
    path.append(from.get());
    path.append(to.get());

    BOOST_REQUIRE(si->checkMotion(from.get(), to.get()));
    BOOST_REQUIRE_EQUAL(space->ledgerEdges(), 1u);

    space->resetStatistics();
    path.interpolate();

    BOOST_CHECK_GT(path.getStateCount(), 10u);
    BOOST_CHECK_EQUAL(space->statistics().steps, 0u);
    BOOST_CHECK_GT(space->statistics().served, 0u);
}

// The headline of keeping the trajectory: a tree planner rolls to produce a state and then
// asks the validator about the state it produced, and the answer is already in hand. This
// used to hold only when the filter never engaged -- a deflected extension had to be rolled
// a second time to find out whether it could be reproduced from its endpoints alone. Now
// nothing needs reproducing, so both cases cost zero extra filter calls.
BOOST_AUTO_TEST_CASE(AnExtensionIsValidatedWithoutASecondRollout)
{
    const UR5 robot;
    const UR5::Configuration a = configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0);
    const UR5::Configuration b = configuration(1.4, -0.7, 1.2, -0.2, 1.20, 0.5);

    const Barrier clear(robot, emptyField(), 0.0);
    const Filter clearFilter(clear, filterParameters());
    const Barrier truth(robot, obstacleField(), 0.0);
    const Barrier guard = Barrier::guarding(robot, obstacleField(), 0.0);
    const Filter deflectingFilter(guard, filterParameters());

    for (const bool unobstructed : {true, false})
    {
        auto space = makeSpace(unobstructed ? clearFilter : deflectingFilter);
        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
        si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
        si->setup();

        ob::ScopedState<> from(space), to(space), reached(space);
        Space::setState(from.get(), a);
        Space::setState(to.get(), unobstructed ? b : pastObstacle(truth, a));

        // What RRT.cpp:144 does, followed by what RRT.cpp:148 does.
        space->interpolate(from.get(), to.get(), 0.5, reached.get());
        const std::size_t rolled = space->statistics().steps;
        BOOST_REQUIRE_GT((Space::configurationOf(reached.get()) - a).norm(), 1e-9);
        if (!unobstructed)
            BOOST_REQUIRE_GT(space->statistics().filtered, 0u);

        BOOST_CHECK(si->checkMotion(from.get(), reached.get()));
        BOOST_CHECK_EQUAL(space->statistics().steps, rolled);
        BOOST_CHECK_EQUAL(space->ledgerEdges(), 1u);

        // And it stays answered: a later re-query, which is what
        // PathGeometric::check() does, is a lookup rather than a rollout.
        BOOST_CHECK(si->checkMotion(from.get(), reached.get()));
        BOOST_CHECK_EQUAL(space->statistics().steps, rolled);
    }
}

// RRTConnect grows a tree from the goal as well, and validates its edges backwards:
// `si_->isValid(dstate) && si_->checkMotion(dstate, nmotion->state)` (RRTConnect.cpp:143).
// A re-rolling validator could never answer that -- rolling from the deflected endpoint
// back toward the node is a different motion -- so deflected goal-tree extensions were
// rejected outright. A recorded polyline visits the same states either way round, so it
// answers in both directions.
BOOST_AUTO_TEST_CASE(AReversedEdgeIsAnsweredFromTheLedger)
{
    const UR5 robot;
    const Barrier truth(robot, obstacleField(), 0.0);
    const Barrier guard = Barrier::guarding(robot, obstacleField(), 0.0);
    const Filter filter(guard, filterParameters());
    auto space = makeSpace(filter);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
    si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
    si->setup();

    const UR5::Configuration a = UR5::Configuration::Zero();
    ob::ScopedState<> from(space), to(space), reached(space);
    Space::setState(from.get(), a);
    // `deflectedArrival()`, not `pastObstacle()`, for the reason given at its definition:
    // the sweep-joint-0 family needs the arm to fold towards its own base, which the self
    // rows forbid, so those rollouts slide along the obstacle instead of arriving. Without
    // an arrival there is no keyed record, and this test's subject is the record.
    Space::setState(to.get(), deflectedArrival());

    space->interpolate(from.get(), to.get(), 0.5, reached.get());
    const std::size_t rolled = space->statistics().steps;
    BOOST_REQUIRE_GT(space->statistics().filtered, 0u);
    BOOST_REQUIRE_GT((Space::configurationOf(reached.get()) - a).norm(), 1e-9);

    // The goal-tree question, asked the way RRTConnect asks it.
    BOOST_CHECK(si->checkMotion(reached.get(), from.get()));
    BOOST_CHECK_EQUAL(space->statistics().steps, rolled);

    const Space::EdgeRecord record = space->recordedEdge(Space::configurationOf(reached.get()), a);
    BOOST_REQUIRE(static_cast<bool>(record));
    BOOST_CHECK(record.reversed());
    // Query order: the walk now starts at the deflected endpoint and finishes at `a`.
    BOOST_CHECK_EQUAL((record[0] - Space::configurationOf(reached.get())).norm(), 0.0);
    BOOST_CHECK_EQUAL((record.at(1.0) - a).norm(), 0.0);
}

// Between two waypoints the control was held constant, so the executed motion there is a
// straight line and sampling inside a step is exact rather than invented. This is what the
// old rollout could not do: it quantised every fraction to a whole step, so asking for a
// finer resolution silently handed back step boundaries.
BOOST_AUTO_TEST_CASE(StatesInsideAStepAreOnTheExecutedSegment)
{
    const UR5 robot;
    const Barrier truth(robot, obstacleField(), 0.0);
    const Barrier guard = Barrier::guarding(robot, obstacleField(), 0.0);
    const Filter filter(guard, filterParameters());
    auto space = makeSpace(filter);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
    si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
    si->setup();

    const UR5::Configuration a = UR5::Configuration::Zero();
    const UR5::Configuration b = deflectedArrival();
    BOOST_REQUIRE(truth.isSafe(b));
    BOOST_REQUIRE(straightLineBlocked(truth, a, b));

    ob::ScopedState<> from(space), to(space), reached(space), out(space);
    Space::setState(from.get(), a);
    Space::setState(to.get(), b);
    space->interpolate(from.get(), to.get(), 1.0, reached.get());
    BOOST_REQUIRE(si->checkMotion(from.get(), reached.get()));

    const Space::EdgeRecord record =
        space->recordedEdge(a, Space::configurationOf(reached.get()));
    BOOST_REQUIRE(static_cast<bool>(record));
    const std::size_t steps = record.size() - 1;
    BOOST_REQUIRE_GT(steps, 4u);

    // Bent, so that per-step interpolation is distinguishable from lerping the edge.
    double deviation = 0.0;
    for (std::size_t i = 0; i <= steps; ++i)
        deviation = std::max(deviation,
                             (record[i] - (a + (static_cast<double>(i) / steps) * (b - a))).norm());
    BOOST_REQUIRE_GT(deviation, 0.05);

    const std::size_t before = space->statistics().steps;
    for (std::size_t i = 0; i < steps; ++i)
    {
        const double t = (static_cast<double>(i) + 0.5) / static_cast<double>(steps);
        space->interpolate(from.get(), reached.get(), t, out.get());
        const UR5::Configuration midpoint = 0.5 * (record[i] + record[i + 1]);
        BOOST_CHECK_LE((Space::configurationOf(out.get()) - midpoint).norm(), 1e-12);
    }
    // None of that cost a filter call.
    BOOST_CHECK_EQUAL(space->statistics().steps, before);
}

// executedPath() hands back the motion the planner actually found, edge by edge.
BOOST_AUTO_TEST_CASE(ExecutedPathReplaysTheRecordedWaypoints)
{
    const UR5 robot;
    const Barrier truth(robot, obstacleField(), 0.0);
    const Barrier guard = Barrier::guarding(robot, obstacleField(), 0.0);
    const Filter filter(guard, filterParameters());
    auto space = makeSpace(filter);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
    si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
    si->setup();

    const UR5::Configuration a = UR5::Configuration::Zero();
    const UR5::Configuration b = deflectedArrival();
    BOOST_REQUIRE(truth.isSafe(b));
    BOOST_REQUIRE(straightLineBlocked(truth, a, b));

    ob::ScopedState<> from(space), to(space), reached(space);
    Space::setState(from.get(), a);
    Space::setState(to.get(), b);
    space->interpolate(from.get(), to.get(), 1.0, reached.get());
    BOOST_REQUIRE(si->checkMotion(from.get(), reached.get()));

    const Space::EdgeRecord record =
        space->recordedEdge(a, Space::configurationOf(reached.get()));
    BOOST_REQUIRE(static_cast<bool>(record));
    std::vector<UR5::Configuration> expected;
    for (std::size_t i = 0; i < record.size(); ++i)
        expected.push_back(record[i]);

    og::PathGeometric sparse(si);
    sparse.append(from.get());
    sparse.append(reached.get());

    std::size_t misses = 1;
    const std::size_t before = space->statistics().steps;
    const og::PathGeometric dense = ompl::cbf::executedPath(sparse, 0.0, &misses);

    BOOST_CHECK_EQUAL(misses, 0u);
    BOOST_CHECK_EQUAL(space->statistics().steps, before);
    BOOST_REQUIRE_EQUAL(dense.getStateCount(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i)
        BOOST_CHECK_EQUAL((Space::configurationOf(dense.getState(i)) - expected[i]).norm(), 0.0);
}

// Eviction must be loud. An evicted edge still validates -- the fallback re-derives it,
// which is what every consumer used to get -- but the result is then a different
// trajectory from the one the planner found, so the counter has to move.
BOOST_AUTO_TEST_CASE(AnEvictedEdgeIsReportedAndStillValidates)
{
    const UR5 robot;
    const Barrier barrier(robot, emptyField(), 0.0);
    const Filter filter(barrier, filterParameters());
    auto space = makeSpace(filter);
    space->setLedgerCapacity(4);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
    si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
    si->setup();

    const UR5::Configuration a = configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0);
    ob::ScopedState<> from(space), to(space), reached(space);
    Space::setState(from.get(), a);

    for (int i = 1; i <= 3; ++i)
    {
        Space::setState(to.get(), a + UR5::Configuration::Constant(0.2 * i));
        space->interpolate(from.get(), to.get(), 1.0, reached.get());
        BOOST_CHECK(si->checkMotion(from.get(), reached.get()));
    }

    BOOST_CHECK_GT(space->statistics().evicted, 0u);
    BOOST_CHECK_LE(space->ledgerWaypoints(), space->ledgerCapacity());

    // The oldest edge is gone, so asking again re-derives it -- and executedPath() says so.
    Space::setState(to.get(), a + UR5::Configuration::Constant(0.2));
    space->interpolate(from.get(), to.get(), 1.0, reached.get());
    space->clearLedger();
    og::PathGeometric sparse(si);
    sparse.append(from.get());
    sparse.append(reached.get());
    std::size_t misses = 0;
    ompl::cbf::executedPath(sparse, 0.0, &misses);
    BOOST_CHECK_EQUAL(misses, 1u);
}

// The recorded endpoint has to be bit-identical to the state the planner will store, or
// the lookup misses forever. enforceBounds() can break that, so when it bites nothing is
// staged and the edge falls back to being rolled -- correct, just not free.
BOOST_AUTO_TEST_CASE(ABoundsClampedEndpointIsNotStaged)
{
    const UR5 robot;
    const Barrier barrier(robot, emptyField(), 0.0);
    const Filter filter(barrier, filterParameters());
    auto space = makeSpace(filter);

    ob::RealVectorBounds tight(dim);
    for (int j = 0; j < dim; ++j)
    {
        tight.setLow(j, UR5::lowerBounds()[j]);
        tight.setHigh(j, UR5::upperBounds()[j]);
    }
    tight.setHigh(0, 0.2);
    space->setBounds(tight);

    const UR5::Configuration a = UR5::Configuration::Zero();
    const UR5::Configuration b = configuration(1.5, 0.0, 0.0, 0.0, 0.0, 0.0);
    ob::ScopedState<> from(space), to(space), out(space);
    Space::setState(from.get(), a);
    Space::setState(to.get(), b);
    space->interpolate(from.get(), to.get(), 1.0, out.get());

    const UR5::Configuration stored = Space::configurationOf(out.get());
    BOOST_REQUIRE_LE(stored[0], 0.2 + 1e-12);
    BOOST_REQUIRE_GT(b[0] - stored[0], 1e-6);  // the clamp really did bite
    BOOST_CHECK(!space->staged(a, stored));
}

// interpolate() must report "nowhere" rather than "sideways" when a rollout makes no
// headway, because RRTConnect's connect loop (RRTConnect.cpp:294, `while (gsc ==
// ADVANCED)`) has no termination-condition guard and assumes every ADVANCED closes the
// gap by maxDistance_. Returning a deflected state that is no closer to the target spins
// that loop forever -- a hang, not a timeout.
BOOST_AUTO_TEST_CASE(NoProgressIsReportedAsNoMotion)
{
    const UR5 robot;
    const Barrier barrier(robot, corneringField(), 0.0);
    const Filter filter(barrier, filterParameters());
    auto space = makeSpace(filter);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
    si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
    si->setup();

    const UR5::Configuration a = UR5::Configuration::Zero();
    const UR5::Configuration b = configuration(1.5, 0.0, 0.0, 0.0, 0.0, 0.0);
    ob::ScopedState<> from(space), to(space), out(space);
    Space::setState(from.get(), a);
    Space::setState(to.get(), b);
    space->interpolate(from.get(), to.get(), 1.0, out.get());

    BOOST_CHECK_LE((Space::configurationOf(out.get()) - a).norm(), 1e-12);
    BOOST_CHECK_GT(space->statistics().abandoned, 0u);

    // And the planner must not be able to turn that into an edge. geometric::RRT has no
    // equal-states guard of its own -- unlike RRTConnect.cpp:132 -- so it asks whether the
    // state can reach itself. A zero-horizon rollout trivially "arrives", which would earn
    // a duplicate node joined by a zero-length edge, so the validator refuses outright.
    BOOST_CHECK(!si->checkMotion(from.get(), out.get()));
    BOOST_CHECK_EQUAL(space->ledgerEdges(), 0u);
}

// The validator answers "does the rollout go where you asked", not "is it safe" --
// safety is already guaranteed by construction, and no StateValidityChecker is
// consulted. On failure it hands back the state the rollout did reach, which is more
// useful to a planner than a point on a straight line the robot never follows.
BOOST_AUTO_TEST_CASE(MotionValidatorReportsReachabilityAndTheStateActuallyReached)
{
    const UR5 robot;
    const UR5::Configuration a = UR5::Configuration::Zero();
    const UR5::Configuration b = configuration(1.5, 0.0, 0.0, 0.0, 0.0, 0.0);

    const Barrier clear(robot, emptyField(), 0.0);
    const Filter clearFilter(clear, filterParameters());
    const Barrier cornered(robot, corneringField(), 0.0);
    const Filter corneredFilter(cornered, filterParameters());

    for (const bool reachable : {true, false})
    {
        auto space = makeSpace(reachable ? clearFilter : corneredFilter);
        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
        si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
        si->setup();

        ob::ScopedState<> from(space), to(space), last(space);
        Space::setState(from.get(), a);
        Space::setState(to.get(), b);
        std::pair<ob::State *, double> lastValid(last.get(), -1.0);

        BOOST_CHECK_EQUAL(si->checkMotion(from.get(), to.get()), reachable);
        BOOST_CHECK_EQUAL(si->checkMotion(from.get(), to.get(), lastValid), reachable);
        if (!reachable)
        {
            // Nothing safe was available, so the rollout never left the start.
            BOOST_CHECK_LE((Space::configurationOf(last.get()) - a).norm(), 1e-12);
            BOOST_CHECK_EQUAL(lastValid.second, 0.0);
        }
    }
}

// The payoff: stock bidirectional RRTConnect, no collision checking anywhere, and the
// motion that would actually be executed -- recovered by densifying the waypoints
// through the same rollout -- clear at every step.
//
// The scene is the demo's, not `obstacleField()`: start and goal are the same arm shape
// rotated about the base, with the obstacle sitting where the arm would be at the
// midpoint of the direct sweep. `obstacleField()` is deliberately close to the base,
// which is what makes it useful for the rollout tests above (the filter engages
// immediately) and useless here -- with the arm extended it blocks about 2.5 rad of base
// rotation, leaving a goal region only a few cm wide. That is a test about goal
// tolerances in a near-infeasible problem, not about planning.
BOOST_AUTO_TEST_CASE(StockRRTConnectPlansSafelyThroughTheRolloutSpace)
{
    const UR5 robot;
    const sdf::GridSDF field(
        [](const Eigen::Vector3d &p)
        {
            return std::min((p - Eigen::Vector3d(-0.742, 0.121, 1.083)).norm() - 0.18,
                            (p - Eigen::Vector3d(-0.742, 0.121, 1.383)).norm() - 0.14);
        },
        UR5::reachableBounds(), voxel);
    const Barrier truth(robot, field, Barrier::defaultMargin);
    const Barrier guard = Barrier::guarding(robot, field, Barrier::defaultMargin);
    const Filter filter(guard, filterParameters());
    auto space = makeSpace(filter);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
    si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
    si->setup();

    const UR5::Configuration startConfiguration = configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0);
    const UR5::Configuration goalConfiguration = configuration(2.4, -1.2, 1.8, -0.6, 1.57, 0.0);
    BOOST_REQUIRE(guard.isSafe(startConfiguration));
    BOOST_REQUIRE(guard.isSafe(goalConfiguration));
    // The point of the scene: the direct joint-space motion is not available.
    BOOST_REQUIRE(straightLineBlocked(truth, startConfiguration, goalConfiguration));

    ob::ScopedState<> start(space), goal(space);
    Space::setState(start.get(), startConfiguration);
    Space::setState(goal.get(), goalConfiguration);

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal, 0.35);

    auto planner = std::make_shared<og::RRTConnect>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();
    BOOST_REQUIRE(planner->solve(ob::timedPlannerTerminationCondition(10.0)) ==
                  ob::PlannerStatus::EXACT_SOLUTION);

    auto path = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());

    // The motion that would actually be executed, replayed from the records the planner
    // built the tree on. Sampled at 0.02 rad, which the old rollout could not honour --
    // it quantised every fraction to a whole step, so a fine request came back coarse.
    std::size_t misses = 1;
    const og::PathGeometric dense = ompl::cbf::executedPath(*path, 0.02, &misses);
    BOOST_CHECK_EQUAL(misses, 0u);
    BOOST_CHECK_EQUAL(space->statistics().evicted, 0u);
    BOOST_REQUIRE_GT(dense.getStateCount(), 2u);
    for (std::size_t i = 0; i < dense.getStateCount(); ++i)
        BOOST_CHECK(truth.isSafe(Space::configurationOf(dense.getState(i))));

    BOOST_CHECK_GT(space->statistics().steps, 0u);

    // Safety at sampled states is not enough: a path can be safe everywhere it is
    // looked at and still be unexecutable, if the reconstruction of one edge does not
    // finish where the next edge starts. Nothing else in this suite would notice --
    // every state of a re-derived trajectory is filter-certified too, so the barrier
    // assertions above pass either way. So measure the seams directly.
    //
    // The bound is one full-speed step plus `reachTolerance`, which is how far the
    // arrive-or-reject fallback may finish from the waypoint the planner stored. Replayed
    // edges have no seam at all; this is sized for the fallback ones.
    const double stride = UR5::velocityLimits().norm() * stepSize + space->reachTolerance();
    double worstGap = 0.0;
    for (std::size_t i = 0; i + 1 < dense.getStateCount(); ++i)
        worstGap = std::max(worstGap, (Space::configurationOf(dense.getState(i + 1)) -
                                       Space::configurationOf(dense.getState(i)))
                                          .norm());
    BOOST_CHECK_LE(worstGap, stride);
}

// What the certificate is for, and what self-collision costs it.
//
// With nothing in the way this edge used to be a *single* straight step: one filter call
// established that the filter had nothing to say about the whole extension. That is over.
// The certificate is a minimum over every barrier, and the self-collision pairs are always
// in it — the tightest pair in `UR5::selfPairs()` is never clearer than 58.2 mm at any
// configuration, so however empty the workspace is, one call can only ever certify what
// 58.2 mm of clearance buys.
//
// What survives is the shape of the win rather than its size. The edge is still exactly
// the straight line, still lands on the target to the last bit, and still costs a small
// fraction of the fixed-step rollout — every step of it is a certified hop. It is a
// constant factor now, not a collapse to one call.
BOOST_AUTO_TEST_CASE(AnUnobstructedEdgeIsAStraightLineOfCoarseSteps)
{
    const UR5 robot;
    const Barrier barrier(robot, emptyField(), 0.0);
    const Filter filter(barrier, filterParameters());
    auto space = makeSpace(filter);

    const UR5::Configuration a = configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0);
    const UR5::Configuration b = configuration(1.4, -0.7, 1.2, -0.2, 1.20, 0.5);
    const unsigned int fixed = space->horizonSteps(a, b);
    BOOST_REQUIRE_GT(fixed, 40u);  // 40 steps is what it costs without a certificate

    const Space::Rollout rollout = space->roll(a, b, 1.0);
    BOOST_CHECK(rollout.reachedTarget);
    BOOST_CHECK_EQUAL((rollout.end - b).norm(), 0.0);

    // Every step ran past `stepSize` on a certificate, and there are far fewer of them
    // than the fixed step would have taken.
    BOOST_CHECK_EQUAL(rollout.coarse, rollout.steps);
    BOOST_CHECK_EQUAL(rollout.waypoints.size(), rollout.steps + 1u);
    BOOST_CHECK_LT(rollout.steps, fixed / 2u);

    // And it is genuinely the straight line, not merely a motion that ends in the right
    // place: total travel equals the distance, so no interior waypoint is off it.
    BOOST_CHECK_EQUAL((rollout.waypoints.front() - a).norm(), 0.0);
    BOOST_CHECK_CLOSE(rollout.travel, (b - a).norm(), 1e-9);
}

// The cap is the A/B: at 1 the rollout steps at `stepSize` whatever the certificate says,
// which is what it did before there was one.
BOOST_AUTO_TEST_CASE(TheStepCapRestoresTheFixedStep)
{
    const UR5 robot;
    const Barrier barrier(robot, emptyField(), 0.0);
    const Filter filter(barrier, filterParameters());
    auto space = makeSpace(filter);
    space->setMaxStepScale(1.0);

    const UR5::Configuration a = configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0);
    const UR5::Configuration b = configuration(1.4, -0.7, 1.2, -0.2, 1.20, 0.5);

    const Space::Rollout rollout = space->roll(a, b, 1.0);
    BOOST_CHECK_EQUAL(rollout.steps, space->horizonSteps(a, b));
    BOOST_CHECK_EQUAL(rollout.coarse, 0u);
    BOOST_CHECK(rollout.reachedTarget);

    // Same motion either way -- the certificate changes what it costs to find out, not
    // where the arm goes.
    space->setMaxStepScale(std::numeric_limits<double>::infinity());
    BOOST_CHECK_LE((space->roll(a, b, 1.0).end - rollout.end).norm(), 1e-12);
    BOOST_CHECK_THROW(space->setMaxStepScale(0.5), ompl::Exception);
}

// The claim that makes a long step admissible at all: over a certified span the filter is
// a no-op, so the straight line joining its endpoints is safe at *every* point, not only
// where it was evaluated. Nothing else in this suite would catch a certificate that is
// too long -- the audit samples the same waypoints the rollout produced, and those are
// safe by construction -- so sample strictly inside the coarse steps and check there.
//
// Checked against the barrier the filter guards, which is the one it promised to hold:
// a certified step is not merely collision free, it is a step the QP would have passed
// through untouched.
BOOST_AUTO_TEST_CASE(CertifiedStepsAreSafeAlongTheirWholeSpan)
{
    const UR5 robot;
    // The demo's scene rather than obstacleField(): a base sweep that starts in the open,
    // works its way around a pair of spheres and comes out the other side, so the rollout
    // passes through both regimes in one motion. obstacleField() sits so close to the base
    // that the arm never has room to certify anything, which would leave nothing to check.
    const sdf::GridSDF field(
        [](const Eigen::Vector3d &p)
        {
            return std::min((p - Eigen::Vector3d(-0.742, 0.121, 1.083)).norm() - 0.18,
                            (p - Eigen::Vector3d(-0.742, 0.121, 1.383)).norm() - 0.14);
        },
        UR5::reachableBounds(), voxel);
    const Barrier guard = Barrier::guarding(robot, field, 0.0);
    const Filter filter(guard, filterParameters());
    auto space = makeSpace(filter);

    const UR5::Configuration a = configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0);
    const UR5::Configuration b = configuration(2.4, -1.2, 1.8, -0.6, 1.57, 0.0);
    BOOST_REQUIRE(straightLineBlocked(Barrier(robot, field, 0.0), a, b));

    const Space::Rollout rollout = space->roll(a, b, 1.0);

    // The scene has to actually exercise both regimes, or the test proves nothing.
    BOOST_REQUIRE_GT(rollout.filtered, 0u);
    BOOST_REQUIRE_GT(rollout.coarse, 0u);
    BOOST_REQUIRE_LT(rollout.coarse, rollout.steps);

    const double stride = UR5::velocityLimits().norm() * stepSize;
    int spans = 0;
    for (std::size_t i = 0; i + 1 < rollout.waypoints.size(); ++i)
    {
        const UR5::Configuration &from = rollout.waypoints[i];
        const UR5::Configuration &to = rollout.waypoints[i + 1];
        if ((to - from).norm() <= stride)
            continue;  // an ordinary QP step; the margin covers what happens inside it

        ++spans;
        constexpr int samples = 50;
        for (int k = 0; k <= samples; ++k)
            BOOST_REQUIRE(guard.isSafe(from + (to - from) * (static_cast<double>(k) / samples)));
    }
    BOOST_REQUIRE_GT(spans, 0);
}
