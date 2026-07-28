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
            const double clearance = barrier.worstValue(to);
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
    // off the straight line it was aimed down.
    const unsigned int total = space->horizonSteps(a, b);
    double deviation = 0.0;
    for (unsigned int i = 0; i <= total; ++i)
    {
        const double t = static_cast<double>(i) / total;
        const UR5::Configuration rolled = space->roll(a, b, t).end;
        BOOST_CHECK(truth.isSafe(rolled));
        deviation = std::max(deviation, (rolled - (a + t * (b - a))).norm());
    }
    BOOST_CHECK_GT(deviation, 0.05);
}

// PathGeometric::check() re-derives every motion and compares, so a rollout that is not
// a pure function of its inputs would break every audit built on it.
BOOST_AUTO_TEST_CASE(RolloutIsDeterministic)
{
    const UR5 robot;
    const Barrier truth(robot, obstacleField(), 0.0);
    const Barrier guard = Barrier::guarding(robot, obstacleField(), 0.0);
    const Filter filter(guard, filterParameters());
    auto space = makeSpace(filter);

    const UR5::Configuration a = UR5::Configuration::Zero();
    const UR5::Configuration b = pastObstacle(truth, a);
    const UR5::Configuration first = space->roll(a, b, 1.0).end;
    for (int repeat = 0; repeat < 4; ++repeat)
    {
        // Roll a different motion in between, so the repeat re-derives the trajectory
        // instead of reading it back out of roll()'s memo.
        space->roll(b, a, 1.0);
        BOOST_CHECK_EQUAL((space->roll(a, b, 1.0).end - first).norm(), 0.0);
    }
}

// roll() resumes the previous rollout when asked to carry it further, which is only
// legitimate if it is indistinguishable from starting over. Checked against a cold space
// at every fraction, on a motion the filter genuinely deflects -- the counters have to
// agree as well as the endpoint, because they are what a benchmark reads.
BOOST_AUTO_TEST_CASE(ResumingAMemoizedRolloutMatchesRestartingIt)
{
    const UR5 robot;
    const Barrier truth(robot, obstacleField(), 0.0);
    const Barrier guard = Barrier::guarding(robot, obstacleField(), 0.0);
    const Filter filter(guard, filterParameters());

    const UR5::Configuration a = UR5::Configuration::Zero();
    const UR5::Configuration b = pastObstacle(truth, a);

    auto warm = makeSpace(filter);
    const unsigned int total = warm->horizonSteps(a, b);
    BOOST_REQUIRE_GT(total, 4u);
    BOOST_REQUIRE_GT(warm->roll(a, b, 1.0).filtered, 0u);
    warm = makeSpace(filter);

    for (unsigned int i = 0; i <= total; ++i)
    {
        const double t = static_cast<double>(i) / total;
        const Space::Rollout resumed = warm->roll(a, b, t);  // walks forward, resuming
        const Space::Rollout cold = makeSpace(filter)->roll(a, b, t);
        BOOST_CHECK_EQUAL((resumed.end - cold.end).norm(), 0.0);
        BOOST_CHECK_EQUAL(resumed.steps, cold.steps);
        BOOST_CHECK_EQUAL(resumed.filtered, cold.filtered);
        BOOST_CHECK_EQUAL(resumed.blocked, cold.blocked);
        BOOST_CHECK_EQUAL(resumed.reachedTarget, cold.reachedTarget);
    }

    // And the whole sweep cost one rollout's worth of filter calls, not total^2 / 2.
    // (Plus at most one for a refusal, which does not advance the rollout.)
    BOOST_CHECK_LE(warm->statistics().steps, total + 1);
    BOOST_CHECK_GT(warm->statistics().resumed, 0u);
}

// A rollout that stopped on a Blocked step stops in the same place next time: the filter
// would see the identical state and nominal control at that step index. So the memo takes
// the answer as final rather than paying for the same refusal again -- and the diagnostic
// counter still moves exactly once.
BOOST_AUTO_TEST_CASE(ABlockedRolloutIsNotRetried)
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
    const std::size_t calls = space->statistics().steps;

    const Space::Rollout second = space->roll(a, b, 1.0);
    BOOST_CHECK_EQUAL(second.steps, 0u);
    BOOST_CHECK_EQUAL(second.blocked, 1u);
    BOOST_CHECK_EQUAL(space->statistics().steps, calls);
    BOOST_CHECK_EQUAL(space->statistics().blocked, 1u);
}

// Densifying a motion into n states costs one rollout, not n of them.
// SpaceInformation::getMotionStates -- which is how PathGeometric::interpolate() and the
// planners' addIntermediateStates mode build their states -- asks for fractions 1/n … n/n
// in order, and every one of those used to restart from the first waypoint. That is
// O(n^2) filter calls to walk a trajectory once, and it landed squarely on the demo's
// audit path.
BOOST_AUTO_TEST_CASE(DensifyingAMotionCostsOneRolloutNotOnePerState)
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
    const unsigned int total = space->horizonSteps(a, b);

    ob::ScopedState<> from(space), to(space);
    Space::setState(from.get(), a);
    Space::setState(to.get(), b);

    og::PathGeometric path(si);
    path.append(from.get());
    path.append(to.get());

    space->resetStatistics();
    path.interpolate();

    BOOST_CHECK_GT(path.getStateCount(), 10u);
    BOOST_CHECK_LE(space->statistics().steps, total);
    BOOST_CHECK_GT(space->statistics().stepsSaved, space->statistics().steps);
}

// The other half of the same saving: a tree planner rolls to produce a state and then
// asks the validator about the state it produced. When the filter never engaged, the
// first rollout is already the answer -- see certifiedByLastRollout().
BOOST_AUTO_TEST_CASE(AnUnfilteredExtensionIsCertifiedWithoutRollingAgain)
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
        const bool valid = si->checkMotion(from.get(), reached.get());

        if (unobstructed)
        {
            BOOST_CHECK(valid);
            BOOST_CHECK_EQUAL(space->statistics().steps, rolled);
            BOOST_CHECK_EQUAL(space->statistics().certified, 1u);
        }
        else
        {
            // The filter engaged, so the two rollouts aim at different targets and the
            // motion is checked for real.
            BOOST_CHECK_GT(space->statistics().steps, rolled);
            BOOST_CHECK_EQUAL(space->statistics().certified, 0u);
        }
    }
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

    const UR5::Configuration a = UR5::Configuration::Zero();
    const UR5::Configuration b = configuration(1.5, 0.0, 0.0, 0.0, 0.0, 0.0);
    ob::ScopedState<> from(space), to(space), out(space);
    Space::setState(from.get(), a);
    Space::setState(to.get(), b);
    space->interpolate(from.get(), to.get(), 1.0, out.get());

    BOOST_CHECK_LE((Space::configurationOf(out.get()) - a).norm(), 1e-12);
    BOOST_CHECK_GT(space->statistics().abandoned, 0u);
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
    path->interpolate();
    BOOST_REQUIRE_GT(path->getStateCount(), 2u);
    for (std::size_t i = 0; i < path->getStateCount(); ++i)
        BOOST_CHECK(truth.isSafe(Space::configurationOf(path->getState(i))));

    BOOST_CHECK_GT(space->statistics().steps, 0u);
}
