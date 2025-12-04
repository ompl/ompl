// demos/vamp/rrtc_planning.cpp
#include <iostream>
#include <vector>
#include <array>
#include <chrono>

// OMPL headers
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// VAMP integration headers
#include <ompl/vamp/Utils.h>
#include <ompl/vamp/VampStateValidityChecker.h>
#include <ompl/vamp/VampMotionValidator.h>
#include <ompl/vamp/collision/factory.hh>

// Robot definition (local to demos)
#include "robots/panda.hh"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = ompl::vamp::robots::Panda;
using Environment = ompl::vamp::collision::Environment<ompl::vamp::FloatVector<ompl::vamp::FloatVectorWidth>>;

int main()
{
    // Build a simple sphere obstacle environment
    ompl::vamp::collision::Environment<float> env_float;
    
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
            ompl::vamp::collision::factory::sphere::array(obs, radius));
    }
    env_float.sort();
    
    // Convert to vectorized environment for SIMD collision checking
    Environment env(env_float);

    // Create state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(Robot::dimension);
    // Get bounds from VAMP Robot information, scale 0/1 config to min/max
    static constexpr std::array<float, Robot::dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
    static constexpr std::array<float, Robot::dimension> ones = {1., 1., 1., 1., 1., 1., 1.};

    auto zero_v = Robot::Configuration(zeros);
    auto one_v = Robot::Configuration(ones);

    Robot::scale_configuration(zero_v);
    Robot::scale_configuration(one_v);
    std::cout << "Zero config: " << zero_v << std::endl;
    std::cout << "One config: " << one_v << std::endl;
    ob::RealVectorBounds bounds(Robot::dimension);
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        bounds.setLow(i, zero_v[{0, i}]);
        bounds.setHigh(i, one_v[{0, i}]);
    }

    space->setBounds(bounds);
    // Create space information with VAMP validators
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(
        std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(si, env));
    si->setMotionValidator(
        std::make_shared<ompl::vamp::VampMotionValidator<Robot>>(si, env));
    si->setup();

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

    // Setup problem definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    // Create RRTConnect planner
    auto planner = std::make_shared<og::RRTConnect>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Solve with 5 second timeout
    auto start_time = std::chrono::steady_clock::now();
    ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(5.0));
    auto end_time = std::chrono::steady_clock::now();
    
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "Planning time: " << duration_ms << " ms" << std::endl;

    if (status == ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "Found solution!" << std::endl;
        
        auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
        std::cout << "Path has " << path->getStateCount() << " states" << std::endl;
        path->print(std::cout);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }

    return 0;
}