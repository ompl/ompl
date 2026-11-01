// Regression tests for AORRTC.

#define BOOST_TEST_MODULE "AORRTC"
#include <boost/test/unit_test.hpp>

#include <cmath>

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/AORRTC.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    /// R^2 with the square root of the Euclidean distance:
    /// a path of several steps costs more than the distance between its endpoints.
    class SqrtDistanceSpace : public ob::RealVectorStateSpace
    {
    public:
        SqrtDistanceSpace() : ob::RealVectorStateSpace(2)
        {
        }

        double distance(const ob::State *state1, const ob::State *state2) const override
        {
            return std::sqrt(ob::RealVectorStateSpace::distance(state1, state2));
        }
    };

    /// Plans in the unit square with every state valid, from (0.1, 0.5) to (0.9, 0.5).
    struct Problem
    {
        Problem(const std::shared_ptr<ob::RealVectorStateSpace> &space, double range)
          : si(std::make_shared<ob::SpaceInformation>(space))
          , pdef(std::make_shared<ob::ProblemDefinition>(si))
          , start(space)
          , goal(space)
          , planner(std::make_shared<og::AORRTC>(si))
        {
            space->setBounds(0., 1.);
            si->setStateValidityChecker([](const ob::State *) { return true; });
            si->setup();
            start[0] = 0.1;
            start[1] = 0.5;
            goal[0] = 0.9;
            goal[1] = 0.5;
            pdef->setStartAndGoalStates(start, goal);
            planner->setRange(range);
            planner->setProblemDefinition(pdef);
            planner->setup();
        }

        ob::PlannerStatus solve(double duration)
        {
            return planner->solve(ob::timedPlannerTerminationCondition(duration));
        }

        ob::SpaceInformationPtr si;
        ob::ProblemDefinitionPtr pdef;
        ob::ScopedState<> start, goal;
        std::shared_ptr<og::AORRTC> planner;
    };

    const std::string EXACT = ob::PlannerStatus(ob::PlannerStatus::EXACT_SOLUTION).asString();

    // The start and goal are directly connectable.
    // AOXRRTConnect primes its first iteration to connect the goal tree straight to the start.
    void checkDirectConnection(double range)
    {
        Problem problem(std::make_shared<ob::RealVectorStateSpace>(2), range);
        BOOST_REQUIRE_EQUAL(problem.solve(0.1).asString(), EXACT);
        BOOST_REQUIRE(problem.pdef->hasExactSolution());

        const auto &path = *problem.pdef->getSolutionPath()->as<og::PathGeometric>();
        BOOST_REQUIRE_GE(path.getStateCount(), 2u);
        BOOST_CHECK_SMALL(problem.si->distance(path.getState(0), problem.start.get()), 1e-9);
        BOOST_CHECK_SMALL(problem.si->distance(path.getState(path.getStateCount() - 1), problem.goal.get()), 1e-9);
        BOOST_CHECK_CLOSE(path.length(), problem.si->distance(problem.start.get(), problem.goal.get()), 1e-6);
    }
}  // namespace

BOOST_AUTO_TEST_CASE(DirectConnectionInOneStep)
{
    checkDirectConnection(1.0);
}

BOOST_AUTO_TEST_CASE(DirectConnectionInSeveralSteps)
{
    checkDirectConnection(0.1);
}

// Once a solution exists, AOXRRTConnect only extends from nodes whose cost plus remaining distance
// fits the cost budget. When no node fit, the neighbour search fell back to the tree root, the
// connect step re-created the same node from it, and the loop (which has no termination check)
// ran forever. With the square-root distance every multi-step connection costs more than the
// budget allows, which triggers this on every iteration after the first.
BOOST_AUTO_TEST_CASE(ConnectStopsWhenTheBudgetRejectsEveryStep)
{
    Problem problem(std::make_shared<SqrtDistanceSpace>(), 0.1);
    BOOST_CHECK_EQUAL(problem.solve(0.2).asString(), EXACT);
}
