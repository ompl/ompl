#define BOOST_TEST_MODULE FilteredStatePropagatorTest
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/cbf/CBFControlFilter.h>
#include <ompl/cbf/FilteredStatePropagator.h>
#include <ompl/cbf/JointSteeringControlSampler.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
using Barrier = ompl::cbf::ClearanceBarrier;
using Filter = ompl::cbf::CBFControlFilter;
using Propagator = ompl::cbf::FilteredStatePropagator;
using ompl::cbf::ControlFilter;
using UR5 = ompl::robots::UR5;
namespace sdf = ompl::sdf;

namespace
{
    constexpr double stepSize = 0.05;
    constexpr double voxel = 0.03;

    sdf::DistanceFn sphereField(const Eigen::Vector3d &center, double radius)
    {
        return [center, radius](const Eigen::Vector3d &p) { return (p - center).norm() - radius; };
    }

    /// The field must cover everything the arm can reach, not just the states the
    /// test steers through: the filter is free to move *all six* joints to preserve
    /// clearance, so it reshapes the arm in ways the nominal control did not ask
    /// for. Undersize this and the filter reports Blocked (no trustworthy barrier)
    /// and the rollout stalls, which looks like a solver failure but is not.
    const sdf::GridSDF &obstacleField()
    {
        static const sdf::GridSDF field(sphereField(Eigen::Vector3d(0.3, 0.3, 1.1), 0.2),
                                        UR5::reachableBounds(), voxel);
        return field;
    }

    /// Everything a planner-shaped test needs: a 6-D real vector state space, a
    /// matching control space, the barrier-backed validity checker, and a
    /// propagator wrapping whichever filter is passed in.
    struct Setup
    {
        UR5 robot;
        Barrier barrier;
        oc::SpaceInformationPtr si;
        std::shared_ptr<Propagator> propagator;

        /// \p collisionChecking selects between the barrier-backed validity checker
        /// and no checking at all — the latter being how a CBF filter is meant to be
        /// deployed, with the barrier as the only thing keeping the planner safe.
        explicit Setup(const ControlFilter &filter, double margin = 0.0, bool collisionChecking = true)
          : robot(), barrier(robot, obstacleField(), margin)
        {
            auto space = std::make_shared<ob::RealVectorStateSpace>(Propagator::dimension);
            ob::RealVectorBounds bounds(Propagator::dimension);
            for (int j = 0; j < Propagator::dimension; ++j)
            {
                bounds.setLow(j, UR5::lowerBounds()[j]);
                bounds.setHigh(j, UR5::upperBounds()[j]);
            }
            space->setBounds(bounds);

            auto controls = std::make_shared<oc::RealVectorControlSpace>(space, Propagator::dimension);
            ob::RealVectorBounds controlBounds(Propagator::dimension);
            controlBounds.setLow(-10.0);
            controlBounds.setHigh(10.0);
            controls->setBounds(controlBounds);

            si = std::make_shared<oc::SpaceInformation>(space, controls);
            si->setPropagationStepSize(stepSize);
            si->setMinMaxControlDuration(1, 20);

            if (collisionChecking)
            {
                const Barrier *barrierPtr = &barrier;
                si->setStateValidityChecker([barrierPtr](const ob::State *state)
                                            {
                                                const auto *values =
                                                    state->as<ob::RealVectorStateSpace::StateType>()->values;
                                                UR5::Configuration q;
                                                for (int j = 0; j < Propagator::dimension; ++j)
                                                    q[j] = values[j];
                                                return barrierPtr->isSafe(q);
                                            });
            }
            else
            {
                si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
            }

            propagator = std::make_shared<Propagator>(si, filter);
            si->setStatePropagator(propagator);
            si->setDirectedControlSamplerAllocator(
                [](const oc::SpaceInformation *space)
                { return std::make_shared<ompl::cbf::JointSteeringControlSampler>(space); });
            si->setup();
        }

        void setState(ob::State *state, const UR5::Configuration &q) const
        {
            for (int j = 0; j < Propagator::dimension; ++j)
                state->as<ob::RealVectorStateSpace::StateType>()->values[j] = q[j];
        }

        UR5::Configuration getState(const ob::State *state) const
        {
            UR5::Configuration q;
            for (int j = 0; j < Propagator::dimension; ++j)
                q[j] = state->as<ob::RealVectorStateSpace::StateType>()->values[j];
            return q;
        }

        void setControl(oc::Control *control, const UR5::Configuration &u) const
        {
            for (int j = 0; j < Propagator::dimension; ++j)
                control->as<oc::RealVectorControlSpace::ControlType>()->values[j] = u[j];
        }
    };

    Filter::Parameters filterParameters()
    {
        Filter::Parameters p;
        p.gamma = 0.4;
        p.maxSpeed = UR5::Configuration::Constant(10.0);
        p.respectJointLimits = true;
        return p;
    }

    /// A nominal control that sweeps joint 0 in whichever direction *reduces*
    /// clearance at \p q — i.e. straight towards trouble. The sign is derived from
    /// the barrier row rather than hardcoded, because which way is "towards" depends
    /// on where the obstacle sits.
    UR5::Configuration sweepControl(const Barrier &barrier, const UR5::Configuration &q, double speed)
    {
        const Barrier::Evaluation evaluation = barrier.evaluate(q);
        const double towardsObstacle = evaluation.rows(static_cast<Eigen::Index>(evaluation.worst), 0) > 0.0
                                           ? -1.0
                                           : 1.0;
        UR5::Configuration u = UR5::Configuration::Zero();
        u[0] = towardsObstacle * speed;
        return u;
    }
}  // namespace

BOOST_AUTO_TEST_CASE(PropagationAppliesTheFilteredControl)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const Filter filter(barrier, filterParameters());
    Setup setup(filter);

    const UR5::Configuration q = UR5::Configuration::Zero();
    const UR5::Configuration nominal = sweepControl(barrier, q, 4.0);

    // What the filter alone would produce.
    UR5::Configuration expectedControl;
    filter.filter(q, nominal, stepSize, expectedControl);

    ob::State *from = setup.si->allocState();
    ob::State *to = setup.si->allocState();
    oc::Control *control = setup.si->allocControl();
    setup.setState(from, q);
    setup.setControl(control, nominal);

    setup.si->getStatePropagator()->propagate(from, control, stepSize, to);
    BOOST_CHECK_LE((setup.getState(to) - (q + expectedControl * stepSize)).norm(), 1e-12);

    setup.si->freeState(from);
    setup.si->freeState(to);
    setup.si->freeControl(control);
}

// propagate() is documented to allow result == state; reading the configuration
// before writing it back is what makes that safe.
BOOST_AUTO_TEST_CASE(PropagationHandlesAliasedStates)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const Filter filter(barrier, filterParameters());
    Setup setup(filter);

    const UR5::Configuration q = UR5::Configuration::Zero();
    const UR5::Configuration nominal = sweepControl(barrier, q, 4.0);

    ob::State *separate = setup.si->allocState();
    ob::State *aliased = setup.si->allocState();
    oc::Control *control = setup.si->allocControl();
    setup.setState(separate, q);
    setup.setState(aliased, q);
    setup.setControl(control, nominal);

    ob::State *reference = setup.si->allocState();
    setup.si->getStatePropagator()->propagate(separate, control, stepSize, reference);
    setup.si->getStatePropagator()->propagate(aliased, control, stepSize, aliased);
    BOOST_CHECK_LE((setup.getState(aliased) - setup.getState(reference)).norm(), 0.0);

    setup.si->freeState(separate);
    setup.si->freeState(aliased);
    setup.si->freeState(reference);
    setup.si->freeControl(control);
}

// The whole reason for filtering inside propagation: an unfiltered rollout that
// drives into the obstacle gets truncated by propagateWhileValid, while the
// filtered one runs to full length and stays safe the whole way.
BOOST_AUTO_TEST_CASE(FilteringTurnsATruncatedEdgeIntoAFullOne)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const UR5::Configuration q = UR5::Configuration::Zero();
    const UR5::Configuration nominal = sweepControl(barrier, q, 4.0);
    constexpr int steps = 20;

    unsigned int plainSteps = 0;
    {
        const ompl::cbf::PassthroughFilter passthrough;
        Setup setup(passthrough);
        ob::State *from = setup.si->allocState();
        ob::State *to = setup.si->allocState();
        oc::Control *control = setup.si->allocControl();
        setup.setState(from, q);
        setup.setControl(control, nominal);
        plainSteps = setup.si->propagateWhileValid(from, control, steps, to);
        setup.si->freeState(from);
        setup.si->freeState(to);
        setup.si->freeControl(control);
    }

    unsigned int filteredSteps = 0;
    {
        const Filter filter(barrier, filterParameters());
        Setup setup(filter);
        ob::State *from = setup.si->allocState();
        oc::Control *control = setup.si->allocControl();
        setup.setState(from, q);
        setup.setControl(control, nominal);

        std::vector<ob::State *> path;
        filteredSteps = setup.si->propagateWhileValid(from, control, steps, path, true);

        // Every state along the filtered rollout is genuinely clear.
        for (unsigned int i = 0; i < filteredSteps; ++i)
            BOOST_CHECK(barrier.isSafe(setup.getState(path[i])));
        for (auto *state : path)
            setup.si->freeState(state);

        BOOST_CHECK_GT(setup.propagator->statistics().filtered, 0u);
        setup.si->freeState(from);
        setup.si->freeControl(control);
    }

    // Unfiltered, the obstacle stops the edge short; filtered, it does not.
    BOOST_CHECK_LT(plainSteps, static_cast<unsigned int>(steps));
    BOOST_CHECK_EQUAL(filteredSteps, static_cast<unsigned int>(steps));
}

// PathControl::check() re-propagates every edge and compares the result against
// the stored state within float epsilon. It therefore passes only if the filter is
// deterministic -- which makes it a free audit of any CBF path a planner returns.
BOOST_AUTO_TEST_CASE(PathControlCheckAcceptsAFilteredEdge)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const Filter filter(barrier, filterParameters());
    Setup setup(filter);

    const UR5::Configuration q = UR5::Configuration::Zero();
    const UR5::Configuration nominal = sweepControl(barrier, q, 4.0);
    constexpr int steps = 20;

    ob::State *from = setup.si->allocState();
    ob::State *to = setup.si->allocState();
    oc::Control *control = setup.si->allocControl();
    setup.setState(from, q);
    setup.setControl(control, nominal);

    const unsigned int taken = setup.si->propagateWhileValid(from, control, steps, to);
    BOOST_REQUIRE_EQUAL(taken, static_cast<unsigned int>(steps));

    // One edge: the nominal control held for `steps` steps.
    oc::PathControl path(setup.si);
    path.append(from);
    path.append(to, control, taken * stepSize);
    BOOST_CHECK(path.check());

    // And the filter really did engage on that edge, so this is not a trivial pass.
    BOOST_CHECK_GT(setup.propagator->statistics().filtered, 0u);

    setup.si->freeState(from);
    setup.si->freeState(to);
    setup.si->freeControl(control);
}

BOOST_AUTO_TEST_CASE(BlockedStepsHoldStillAndAreCounted)
{
    const UR5 robot;
    const UR5::Configuration q = UR5::Configuration::Zero();
    // An obstacle swallowing the immovable base sphere makes the QP infeasible.
    const Eigen::Vector3d base = robot.sphereCenters(q).col(0);
    const sdf::GridSDF field(sphereField(base, 0.5), UR5::reachableBounds(), voxel);
    const Barrier barrier(robot, field, 0.0);
    const Filter filter(barrier, filterParameters());

    // Setup's own barrier (and so its validity checker) uses obstacleField(), where
    // this configuration is fine; the *filter* sees the cornering field. So the
    // state stays valid while the filter still has nothing safe to offer, which is
    // exactly the stalling case worth pinning down.
    Setup setup(filter);
    ob::State *from = setup.si->allocState();
    ob::State *to = setup.si->allocState();
    oc::Control *control = setup.si->allocControl();
    setup.setState(from, q);
    setup.setControl(control, sweepControl(setup.barrier, q, 1.0));

    const auto status = setup.propagator->propagateReporting(from, control, stepSize, to);
    BOOST_CHECK(status == ControlFilter::Status::Blocked);
    BOOST_CHECK_LE((setup.getState(to) - q).norm(), 1e-12);
    BOOST_CHECK_EQUAL(setup.propagator->statistics().blocked, 1u);

    setup.si->freeState(from);
    setup.si->freeState(to);
    setup.si->freeControl(control);
}

BOOST_AUTO_TEST_CASE(StatisticsClassifyEveryStep)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const Filter filter(barrier, filterParameters());
    Setup setup(filter);

    ob::State *from = setup.si->allocState();
    ob::State *to = setup.si->allocState();
    oc::Control *control = setup.si->allocControl();
    setup.setState(from, UR5::Configuration::Zero());

    // A tiny control is safe; a large one towards the obstacle is not.
    const UR5::Configuration q = UR5::Configuration::Zero();
    setup.setControl(control, sweepControl(setup.barrier, q, 0.01));
    setup.propagator->propagate(from, control, stepSize, to);
    setup.setControl(control, sweepControl(setup.barrier, q, 4.0));
    setup.propagator->propagate(from, control, stepSize, to);

    const Propagator::Statistics stats = setup.propagator->statistics();
    BOOST_CHECK_EQUAL(stats.propagations, 2u);
    BOOST_CHECK_EQUAL(stats.unchanged + stats.filtered + stats.blocked, stats.propagations);
    BOOST_CHECK_EQUAL(stats.unchanged, 1u);
    BOOST_CHECK_EQUAL(stats.filtered, 1u);

    setup.propagator->resetStatistics();
    BOOST_CHECK_EQUAL(setup.propagator->statistics().propagations, 0u);

    setup.si->freeState(from);
    setup.si->freeState(to);
    setup.si->freeControl(control);
}

BOOST_AUTO_TEST_CASE(BackwardPropagationIsRefused)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const Filter filter(barrier, filterParameters());
    Setup setup(filter);
    BOOST_CHECK(!setup.propagator->canPropagateBackward());
    BOOST_CHECK(!setup.si->canPropagateBackward());
}

BOOST_AUTO_TEST_CASE(WrongSpaceDimensionIsRejected)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const Filter filter(barrier, filterParameters());

    auto space = std::make_shared<ob::RealVectorStateSpace>(3);
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);
    auto controls = std::make_shared<oc::RealVectorControlSpace>(space, 3);
    controls->setBounds(bounds);
    auto si = std::make_shared<oc::SpaceInformation>(space, controls);

    BOOST_CHECK_THROW(Propagator(si, filter), ompl::Exception);
}

// The integration itself: stock ompl::control::RRT, not subclassed or patched,
// planning through the filtered propagator. If this ever needs a modified planner,
// the whole design premise has broken.
BOOST_AUTO_TEST_CASE(StockRRTPlansThroughTheFilteredPropagator)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const Filter filter(barrier, filterParameters());
    Setup setup(filter);

    const UR5::Configuration start = UR5::Configuration::Zero();
    UR5::Configuration goal = UR5::Configuration::Zero();
    goal[0] = 1.5;  // a base sweep past the obstacle
    BOOST_REQUIRE(barrier.isSafe(start));
    BOOST_REQUIRE(barrier.isSafe(goal));

    ob::ScopedState<ob::RealVectorStateSpace> startState(setup.si->getStateSpace());
    ob::ScopedState<ob::RealVectorStateSpace> goalState(setup.si->getStateSpace());
    for (int j = 0; j < Propagator::dimension; ++j)
    {
        startState->values[j] = start[j];
        goalState->values[j] = goal[j];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(setup.si);
    pdef->setStartAndGoalStates(startState, goalState, 0.3);

    auto planner = std::make_shared<oc::RRT>(setup.si);
    planner->setProblemDefinition(pdef);
    planner->setup();
    const ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(10.0));

    BOOST_REQUIRE(status == ob::PlannerStatus::EXACT_SOLUTION);
    const auto path = std::static_pointer_cast<oc::PathControl>(pdef->getSolutionPath());

    // Deterministic filter => the returned path survives re-propagation.
    BOOST_CHECK(path->check());

    // And every state on it is genuinely clear.
    for (std::size_t i = 0; i < path->getStateCount(); ++i)
        BOOST_CHECK(barrier.isSafe(setup.getState(path->getState(i))));
}

// The claim the whole design rests on: with the CBF filter in the propagator, the
// collision checker can be deleted outright and the planner still never leaves the
// free space. There is no StateValidityChecker here at all -- AllValid means the
// barrier is the only thing keeping RRT honest.
//
// The filter enforces a buffered barrier and the assertions use an unbuffered one,
// which is the whole point of ClearanceBarrier::guarding(): enforcing h >= 0 against
// a trilinearly interpolated field only delivers h >= -O(voxel), and with no
// collision checker in the loop nothing else would absorb that. Drop the buffer here
// and this test fails on real violations, not on a tolerance.
BOOST_AUTO_TEST_CASE(StockRRTPlansSafelyWithNoCollisionCheckerAtAll)
{
    const UR5 robot;
    const Barrier guard = Barrier::guarding(robot, obstacleField(), 0.0);  // filter enforces
    const Barrier truth(robot, obstacleField(), 0.0);                     // we hold it to
    BOOST_REQUIRE_GT(guard.margin(), truth.margin());
    const Filter filter(guard, filterParameters());
    Setup setup(filter, 0.0, /*collisionChecking=*/false);

    BOOST_REQUIRE(setup.si->getStateValidityChecker() != nullptr);

    const UR5::Configuration start = UR5::Configuration::Zero();
    UR5::Configuration goal = UR5::Configuration::Zero();
    goal[0] = 1.5;  // a base sweep past the obstacle
    // Nothing validates the start once the checker is gone, so do it here.
    BOOST_REQUIRE(guard.isSafe(start));
    BOOST_REQUIRE(guard.isSafe(goal));

    ob::ScopedState<ob::RealVectorStateSpace> startState(setup.si->getStateSpace());
    ob::ScopedState<ob::RealVectorStateSpace> goalState(setup.si->getStateSpace());
    for (int j = 0; j < Propagator::dimension; ++j)
    {
        startState->values[j] = start[j];
        goalState->values[j] = goal[j];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(setup.si);
    pdef->setStartAndGoalStates(startState, goalState, 0.3);

    auto planner = std::make_shared<oc::RRT>(setup.si);
    planner->setProblemDefinition(pdef);
    planner->setup();
    BOOST_REQUIRE(planner->solve(ob::timedPlannerTerminationCondition(10.0)) ==
                  ob::PlannerStatus::EXACT_SOLUTION);

    auto path = std::static_pointer_cast<oc::PathControl>(pdef->getSolutionPath());
    BOOST_CHECK(path->check());

    // Audit every propagation step, not just the edge endpoints: with no checker in
    // the loop, an edge could in principle dive through the obstacle and come back
    // out with both endpoints clear. interpolate() expands the path to one state per
    // step using plain propagate(), so it re-runs the filter and nothing else.
    path->interpolate();
    BOOST_REQUIRE_GT(path->getStateCount(), 2u);
    double worst = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path->getStateCount(); ++i)
        worst = std::min(worst, truth.worstValue(setup.getState(path->getState(i))));
    BOOST_CHECK_GE(worst, 0.0);

    // The obstacle really was in the way -- otherwise this proves nothing.
    BOOST_CHECK_GT(setup.propagator->statistics().filtered, 0u);
}
