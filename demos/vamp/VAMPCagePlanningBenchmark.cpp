// demos/vamp/VAMPCagePlanningBenchmark.cpp
#include <iostream>
#include <vector>
#include <array>
#include <chrono>

// OMPL headers
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

// Planners
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

// Benchmark
#include <ompl/tools/benchmark/Benchmark.h>

// VAMP integration headers
#include <ompl/vamp/Utils.h>
#include <ompl/vamp/VampStateValidityChecker.h>
#include <ompl/vamp/VampMotionValidator.h>
#include <ompl/vamp/VampStateSpace.h>

#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

using Robot = vamp::robots::Panda;
using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;

int main()
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

    // create state space
    auto space = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();

    std::cout << "Robot bounds:" << std::endl;
    for (std::size_t i = 0; i < Robot::dimension; ++i){
        std::cout << i << ": " << space->getBounds().low[i] << " to " << space->getBounds().high[i] << std::endl;
    }

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

    // Create RRTConnect planner
    auto planner = std::make_shared<og::RRTConnect>(si);
    ss.setPlanner(planner);

    // Setup problem definition
    ss.setStartAndGoalStates(start, goal);

    // setup benchmark
    double memoryLimit = 4096;
    double runtimeLimit = 5.0;
    double runCount = 100;
    ot::Benchmark b(ss, "VAMP_Cage_Planning");
    ot::Benchmark::Request request(runtimeLimit, memoryLimit, runCount);

    b.addPlanner(std::make_shared<og::RRTConnect>(si));
    b.addPlanner(std::make_shared<og::RRT>(si));
    b.addPlanner(std::make_shared<og::KPIECE1>(si));    
    b.addPlanner(std::make_shared<og::LBKPIECE1>(si));

    b.benchmark(request);
    b.saveResultsToFile("vamp_cage_planning_benchmark_cpp.log");

    // Use python script to transfer .log to .db
    // ompl/scripts/ompl_benchmark_statistics.py vamp_cage_planning_benchmark_cpp.log -d vamp_cage_planning_benchmark_cpp.db

    return 0;
}