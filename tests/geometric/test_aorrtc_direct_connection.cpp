// Regression test for AORRTC when the start and goal are directly connectable.

#define BOOST_TEST_MODULE "AORRTC direct connection regression"
#include <boost/test/unit_test.hpp>

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
    void checkDirectConnection(double range)
    {
        auto space = std::make_shared<ob::RealVectorStateSpace>(2);
        space->setBounds(0.0, 1.0);
        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker([](const ob::State *) { return true; });
        si->setup();

        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        ob::ScopedState<> start(space), goal(space);
        start[0] = 0.1;
        start[1] = 0.5;
        goal[0] = 0.9;
        goal[1] = 0.5;
        pdef->setStartAndGoalStates(start, goal);

        auto planner = std::make_shared<og::AORRTC>(si);
        planner->setRange(range);
        planner->setProblemDefinition(pdef);
        planner->setup();

        const ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(0.1));
        BOOST_REQUIRE_EQUAL(status.asString(), ob::PlannerStatus(ob::PlannerStatus::EXACT_SOLUTION).asString());
        BOOST_REQUIRE(pdef->hasExactSolution());

        const auto &path = *pdef->getSolutionPath()->as<og::PathGeometric>();
        BOOST_REQUIRE_GE(path.getStateCount(), 2u);
        BOOST_CHECK_SMALL(si->distance(path.getState(0), start.get()), 1e-9);
        BOOST_CHECK_SMALL(si->distance(path.getState(path.getStateCount() - 1), goal.get()), 1e-9);
        BOOST_CHECK_CLOSE(path.length(), si->distance(start.get(), goal.get()), 1e-6);
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
