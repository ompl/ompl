#include <vector>
#include <array>
#include <chrono>

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/vamp/Utils.h>
#include <ompl/vamp/VampStateValidityChecker.h>
#include <ompl/vamp/VampMotionValidator.h>
#include <ompl/vamp/VampStateSpace.h>

#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>

#define BOOST_TEST_MODULE VAMPPlanningTest
#include <boost/test/unit_test.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;

BOOST_AUTO_TEST_CASE(VAMPPlanning)
{
    // Build a simple sphere obstacle environment
    vamp::collision::Environment<float> env_float;
    
    // Add some sphere obstacles
    std::vector<std::array<float, 3>> obstacles = {
        {0.55, 0, 0.25},
        {0.35, 0.35, 0.25},
        {0, 0.55, 0.25},
        {-0.55, 0, 0.25},
        {-0.35, -0.35, 0.25},
        {0, -0.55, 0.25},
        {0.35, -0.35, 0.25},
        {0.35, 0.35, 0.8},
        {0, 0.55, 0.8},
        {-0.35, 0.35, 0.8},
        {-0.55, 0, 0.8},
        {-0.35, -0.35, 0.8},
        {0, -0.55, 0.8},
        {0.35, -0.35, 0.8},
    };
    
    constexpr float radius = 0.2f;
    for (const auto& obs : obstacles)
    {
        env_float.spheres.emplace_back(
            vamp::collision::factory::sphere::array(obs, radius));
    }
    env_float.sort();
    
    // Convert to vectorized environment for SIMD collision checking
    Environment env(env_float);

    auto space = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();

    // Create simple setup
    og::SimpleSetup ss(space);

    auto si = ss.getSpaceInformation();

    si->setStateValidityChecker(
        std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(si, env));
    si->setMotionValidator(
        std::make_shared<ompl::vamp::VampMotionValidator<Robot>>(si, env));

    // Define start and goal configurations (Panda 7 DOF)
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    
    std::array<double, 7> start_config = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    std::array<double, 7> goal_config = {2.35, 1., 0., -0.8, 0., 2.5, 0.785};
    
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        start[i] = start_config[i];
        goal[i] = goal_config[i];
    }

    // Create RRTConnect planner (optional, a default will be automatically chosen otherwise)  
    auto planner = std::make_shared<og::RRTConnect>(si);
    ss.setPlanner(planner);

    // Setup problem definition
    ss.setStartAndGoalStates(start, goal);

    // Solve with 5 second timeout
    auto start_time = std::chrono::steady_clock::now();
    ob::PlannerStatus status = ss.solve(ob::timedPlannerTerminationCondition(5.0));
    auto end_time = std::chrono::steady_clock::now();
    
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    auto duration_ms = duration_ns / 1e6;
    BOOST_TEST(status == ob::PlannerStatus::EXACT_SOLUTION);
}
