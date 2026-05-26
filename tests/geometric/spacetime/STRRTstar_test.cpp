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

bool isStateValidHyperball(const ompl::base::State *state)
{
    const auto *values =
        state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    double s = 0.0;
    for (unsigned int i = 0; i < 6; ++i)
        s += values[i] * values[i];
    return (s > 0.1) && (s < 24);
}

class SpaceTimeMotionValidator : public base::MotionValidator
{
public:
    explicit SpaceTimeMotionValidator(const base::SpaceInformationPtr &si)
      : MotionValidator(si)
      , vMax_(si_->getStateSpace().get()->as<base::SpaceTimeStateSpace>()->getVMax())
      , stateSpace_(si_->getStateSpace().get()) {};

    bool checkMotion(const base::State *s1, const base::State *s2) const override
    {
        // Assume motion starts in a valid configuration, so s1 is valid.
        if (!si_->isValid(s2))
        {
            invalid_++;
            return false;
        }

        // Check if motion is forward in time and is not exceeding the speed limit.
        auto *space = stateSpace_->as<base::SpaceTimeStateSpace>();
        auto deltaPos = space->distanceSpace(s1, s2);
        auto deltaT = s2->as<base::CompoundState>()->as<base::TimeStateSpace::StateType>(1)->position -
                      s1->as<base::CompoundState>()->as<base::TimeStateSpace::StateType>(1)->position;

        if (!(deltaT > 0 && deltaPos / deltaT <= vMax_))
        {
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
    double vMax_;                   // maximum velocity
    base::StateSpace *stateSpace_;  // the space-time state space for distance calculation
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
                             bool expectSuccess = true, double OptimumApproxFactor = 1.0, bool addObstacle = false)
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
        if(addObstacle)
        {
            si->setStateValidityChecker([](const base::State *state) { return isStateValidHyperball(state); });
        }
        else
        {
            si->setStateValidityChecker([](const base::State *state) { return isStateValid(state); });
        }
        si->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si));

        // Define a simple setup class
        geometric::SimpleSetup ss(si);

        // Create a start state
        base::ScopedState<> start(space);
        for (unsigned int i = 0; i < dimensions; ++i)
        {
            start[i] = startPos[i];
        }
        start[dimensions] = startTime;

        // Create a goal state
        base::ScopedState<> goal(space);
        for (unsigned int i = 0; i < dimensions; ++i)
        {
            goal[i] = goalPos[i];
        }
        goal[dimensions] = endTime;

        // Set the start and goal states
        ss.setStartAndGoalStates(start, goal);

        // Construct the planner
        auto *strrtStar = new geometric::STRRTstar(si);
        strrtStar->setRange(vMax);
        strrtStar->setOptimumApproxFactor(OptimumApproxFactor);
        ss.setPlanner(base::PlannerPtr(strrtStar));

        ompl::base::IterationTerminationCondition itc(1000);
        base::PlannerStatus solved = base::PlannerStatus::UNKNOWN;

        // Add exception handling to prevent test crashes
        try
        {
            solved = ss.solve(itc);
        }
        catch (const ompl::Exception &e)
        {
            std::cout << "Planning failed with exception: " << e.what() << std::endl;
            solved = base::PlannerStatus::UNKNOWN;
        }

        bool success = solved.operator bool();
        BOOST_CHECK_EQUAL(success, expectSuccess);
    }

protected:
    bool verbose_;
};

// Define test cases with fixed tree to test functions and to determenistically reproduce segfault
class STRRTstarTestAccess : public geometric::STRRTstar
{
public:

    static std::shared_ptr<base::SpaceInformation> makeSpaceTimeStateSpace()
    {
        double vMax = 1.0;
        std::shared_ptr<ompl::base::RealVectorStateSpace> vectorSpace(std::make_shared<base::RealVectorStateSpace>(1));
                base::RealVectorBounds bounds(1);
        bounds.setLow(-1.0);
        bounds.setHigh(1.0);
        vectorSpace->setBounds(bounds);
        std::shared_ptr<base::SpaceTimeStateSpace> timeSpace = std::make_shared<base::SpaceTimeStateSpace>(vectorSpace, vMax);


        timeSpace->setTimeBounds(0.0, 10.0);

        std::shared_ptr<base::SpaceInformation> space = std::make_shared<base::SpaceInformation>(timeSpace);
        space->setStateValidityChecker([](const base::State *state) { return isStateValid(state); });
        space->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(space));
        space->setup();
        return space;
    }

    explicit STRRTstarTestAccess() : STRRTstar(makeSpaceTimeStateSpace())
    {
    }

    // create such tree that will trigger recursion in the constructSolution() and segfault
    void constructPrunedRecursiveSolution()
    {
        std::cout<<"Simulated recusion in constructSolution() test"<<std::endl;
        setRange(1.0);
        setOptimumApproxFactor(0.6);// so 4.0 > 5.0*0.6 > 2.0 
        setup();

        minimumTime_ = 0.0;
        upperTimeBound_ = 5.0; // must be > then invalidGoal time
        isTimeBounded_ = true;

        startMotion_ = makeMotion(0.0, 0.0);
        startMotion_->root = startMotion_->state;
        tStart_->add(startMotion_);

        auto *validGoal = makeMotion(1.0, 2.0);
        validGoal->root = validGoal->state;
        goalMotions_.push_back(validGoal);
        tGoal_->add(validGoal);

        // The connected descendant is rewired to validGoal during pruning,
        // while its original invalid goal root is deleted.
        auto *invalidGoal = makeMotion(1.0, 4.0);
        invalidGoal->root = invalidGoal->state;
        invalidGoal->numConnections = 1;
        goalMotions_.push_back(invalidGoal);
        tGoal_->add(invalidGoal);

        auto *connectedInvalidDescendant = makeMotion(0.9, 1.5);
        connectedInvalidDescendant->parent = invalidGoal;
        connectedInvalidDescendant->root = invalidGoal->state;
        connectedInvalidDescendant->connectionPoint = startMotion_;
        connectedInvalidDescendant->numConnections = 1;
        invalidGoal->children.push_back(connectedInvalidDescendant);
        tGoal_->add(connectedInvalidDescendant);


        // Nodes:
        // S   = startMotion_                 (x=0.0, t=0.0)
        // CID = connectedInvalidDescendant   (x=0.9, t=1.5)
        // VG  = validGoal                    (x=1.0, t=2.0)
        // IG  = invalidGoal                  (x=1.0, t=4.0)
        // CID.connectionPoint -> S     shown as straight line S * CID
        // CID.parent          -> IG    shown as parent edge IG * CID
        // CID.root            -> IG.state

        //     time t
        //       ^
        //   4.0 |                                      IG   [invalid goal root]
        //       |                                      *
        //       |                                     *
        //       |                                    * 
        //       |                                   *
        //       |                                  *
        //   2.0 |                                 *    VG   [valid goal]
        //       |                                 *    *
        //       |                                 *
        //   1.5 |                                CID
        //       |                              ***
        //       |                         ******
        //       |                    *****
        //       |           *********
        //       |  *********        
        //   0.0 *------------------------------------------> coordinate x
        //     S (0.0)                              0.9  1.0
        //    start

                
        constructSolution(startMotion_, invalidGoal, base::ReportIntermediateSolutionFn(),
                          base::plannerNonTerminatingCondition());
                         
    }

private:
    Motion *makeMotion(double position, double time)
    {
        auto *motion = new Motion(si_);
        auto *compound = motion->state->as<base::CompoundState>();
        compound->as<base::RealVectorStateSpace::StateType>(0)->values[0] = position;
        compound->as<base::TimeStateSpace::StateType>(1)->position = time;
        return motion;
    }
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
    runSpaceTimePlanner(-10.0, -5.0, 0.2, 1, {0.0}, {1.0}, true);  // Allow negative time, expect success
}

// Test case for zero time difference
BOOST_AUTO_TEST_CASE(spacetime_planning_zero_time)
{
    runSpaceTimePlanner(0.0, 0.01, 0.2, 1, {0.0}, {0.0}, true);  // Small positive time difference, same position
}

// Test case for large time bounds
BOOST_AUTO_TEST_CASE(spacetime_planning_large_time)
{
    runSpaceTimePlanner(0.0, 1000.0);  // Large time range
}

// Test case for tight velocity constraint
BOOST_AUTO_TEST_CASE(spacetime_planning_tight_velocity)
{
    runSpaceTimePlanner(0.0, 100.0, 0.01);  // Adjusted end time to make problem feasible
}

// Test case for 2D spatial planning
BOOST_AUTO_TEST_CASE(spacetime_planning_2d)
{
    runSpaceTimePlanner(0.0, 10.0, 0.2, 2, {0.0, 0.0}, {1.0, 1.0});  // 2D spatial planning
}

// Test case for invalid start state
BOOST_AUTO_TEST_CASE(spacetime_planning_invalid_start)
{
    runSpaceTimePlanner(0.0, 10.0, 0.2, 1, {2.0}, {1.0}, false);  // Start position outside bounds
}

// Test case for 3D spatial planning with diagonal movement
BOOST_AUTO_TEST_CASE(spacetime_planning_3d_diagonal)
{
    runSpaceTimePlanner(0.0, 15.0, 0.3, 3, {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});  // 3D diagonal movement
}

// Test case with fixed tree for broken recursive and use-after-free in constructSolution
BOOST_AUTO_TEST_CASE(spacetime_pruned_goal_solution_uses_rewired_goal_motion)
{
    STRRTstarTestAccess planner;

    BOOST_CHECK_NO_THROW(planner.constructPrunedRecursiveSolution());
}

// Stress test for pruning and final time optimising with different seeds
BOOST_AUTO_TEST_CASE(spacetime_planning_stress)
{
    const unsigned int runs = 100;

    for (unsigned int i = 0; i < runs; ++i)
    {
        BOOST_TEST_CONTEXT("stress run " << i)
        {
            runSpaceTimePlanner(0.0, 20.0, 0.5, 6, {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
                                {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, true, 0.99,
                                true);  // 6D diagonal movement with goal time optimisation
        }
    }
}


BOOST_AUTO_TEST_SUITE_END()
