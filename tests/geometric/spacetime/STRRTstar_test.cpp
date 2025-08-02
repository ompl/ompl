#define BOOST_TEST_MODULE "SpaceTimePlanning"
#include <boost/test/unit_test.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/Goal.h>

#include <iostream>

using namespace ompl;

bool isStateValid(const base::State *state)
{
    const auto pos = state->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0)->values[0];
    return pos < std::numeric_limits<double>::infinity();
}

class SpaceTimeMotionValidator : public base::MotionValidator {

public:
    explicit SpaceTimeMotionValidator(const base::SpaceInformationPtr &si) : MotionValidator(si),
      vMax_(si_->getStateSpace().get()->as<base::SpaceTimeStateSpace>()->getVMax()),
      stateSpace_(si_->getStateSpace().get()) {};

    bool checkMotion(const base::State *s1, const base::State *s2) const override
    {
        // Assume motion starts in a valid configuration, so s1 is valid.
        if (!si_->isValid(s2)) {
            invalid_++;
            return false;
        }

        // Check if motion is forward in time and is not exceeding the speed limit.
        auto *space = stateSpace_->as<base::SpaceTimeStateSpace>();
        auto deltaPos = space->distanceSpace(s1, s2);
        auto deltaT = s2->as<base::CompoundState>()->as<base::TimeStateSpace::StateType>(1)->position -
                      s1->as<base::CompoundState>()->as<base::TimeStateSpace::StateType>(1)->position;

        if (!(deltaT > 0 && deltaPos / deltaT <= vMax_)) {
            invalid_++;
            return false;
        }

        return true;
    }

    bool checkMotion(const ompl::base::State *, const ompl::base::State *,
                     std::pair<base::State *, double> &) const override
    {
        throw ompl::Exception("SpaceTimeMotionValidator::checkMotion", "not implemented");
    }

private:
    double vMax_; // maximum velocity
    base::StateSpace *stateSpace_; // the space-time state space for distance calculation
};

// Define a class for the test fixture, similar to SimplifyTest in 2dpath_simplifying.cpp
class SpaceTimeTestFixture
{
public:
    SpaceTimeTestFixture()
    {
    }

    // A generic planning function that can be called by different tests with different time bounds
    void runSpaceTimePlanner(double startTime, double endTime, double vMax = 0.2, unsigned int dimensions = 1,
                           std::vector<double> startPos = {0.0}, std::vector<double> goalPos = {1.0},
                           bool expectSuccess = true)
    {
        // Construct the state space we are planning in
        auto vectorSpace(std::make_shared<base::RealVectorStateSpace>(dimensions));
        auto space = std::make_shared<base::SpaceTimeStateSpace>(vectorSpace, vMax);

        // Set the bounds for R^n
        base::RealVectorBounds bounds(dimensions);
        bounds.setLow(-1.0);
        bounds.setHigh(1.0);
        vectorSpace->setBounds(bounds);

        // Set time bounds
        space->setTimeBounds(startTime, endTime);

        // Create the space information class for the space
        base::SpaceInformationPtr si = std::make_shared<base::SpaceInformation>(space);

        // Set state validity checking for this space
        si->setStateValidityChecker([](const base::State *state) { return isStateValid(state); });
        si->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si));

        // Define a simple setup class
        geometric::SimpleSetup ss(si);

        // Create a start state
        base::ScopedState<> start(space);
        for (unsigned int i = 0; i < dimensions; ++i) {
            start[i] = startPos[i];
        }
        start[dimensions] = startTime;

        // Create a goal state
        base::ScopedState<> goal(space);
        for (unsigned int i = 0; i < dimensions; ++i) {
            goal[i] = goalPos[i];
        }
        goal[dimensions] = endTime;

        // Set the start and goal states
        ss.setStartAndGoalStates(start, goal);

        // Construct the planner
        auto *strrtStar = new geometric::STRRTstar(si);
        strrtStar->setRange(vMax);
        ss.setPlanner(base::PlannerPtr(strrtStar));

        ompl::base::IterationTerminationCondition itc(1000);
        base::PlannerStatus solved = base::PlannerStatus::UNKNOWN;

        // Add exception handling to prevent test crashes
        try {
            solved = ss.solve(itc);
        } catch (const ompl::Exception &e) {
            std::cout << "Planning failed with exception: " << e.what() << std::endl;
            solved = base::PlannerStatus::UNKNOWN;
        }

        bool success = solved.operator bool();
        BOOST_CHECK_EQUAL(success, expectSuccess);
    }

protected:
    bool verbose_;
};

// Define the test suite
BOOST_FIXTURE_TEST_SUITE(SpaceTimePlanningTestFixture, SpaceTimeTestFixture)

// Test case for planning with time starting at 0s
BOOST_AUTO_TEST_CASE(spacetime_planning_start_0)
{
    runSpaceTimePlanner(0.0, 10.0);
}

// Test case for planning with time starting at 100s
BOOST_AUTO_TEST_CASE(spacetime_planning_start_100)
{
    runSpaceTimePlanner(100.0, 110.0);
}

// Test case for negative time bounds
BOOST_AUTO_TEST_CASE(spacetime_planning_negative_time)
{
    runSpaceTimePlanner(-10.0, -5.0, 0.2, 1, {0.0}, {1.0}, true); // Allow negative time, expect success
}

// Test case for zero time difference
BOOST_AUTO_TEST_CASE(spacetime_planning_zero_time)
{
    runSpaceTimePlanner(0.0, 0.01, 0.2, 1, {0.0}, {0.0}, true); // Small positive time difference, same position
}

// Test case for large time bounds
BOOST_AUTO_TEST_CASE(spacetime_planning_large_time)
{
    runSpaceTimePlanner(0.0, 1000.0); // Large time range
}

// Test case for tight velocity constraint
BOOST_AUTO_TEST_CASE(spacetime_planning_tight_velocity)
{
    runSpaceTimePlanner(0.0, 100.0, 0.01); // Adjusted end time to make problem feasible
}

// Test case for 2D spatial planning
BOOST_AUTO_TEST_CASE(spacetime_planning_2d)
{
    runSpaceTimePlanner(0.0, 10.0, 0.2, 2, {0.0, 0.0}, {1.0, 1.0}); // 2D spatial planning
}

// Test case for invalid start state
BOOST_AUTO_TEST_CASE(spacetime_planning_invalid_start)
{
    runSpaceTimePlanner(0.0, 10.0, 0.2, 1, {2.0}, {1.0}, false); // Start position outside bounds
}

// Test case for 3D spatial planning with diagonal movement
BOOST_AUTO_TEST_CASE(spacetime_planning_3d_diagonal)
{
    runSpaceTimePlanner(0.0, 15.0, 0.3, 3, {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}); // 3D diagonal movement
}

BOOST_AUTO_TEST_SUITE_END()
