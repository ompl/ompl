// demos/vamp/VAMPSubgroupPlanning.cpp
//
// Demonstrates subgroup planning: plan over a user-selected subset of a
// VAMP robot's DOFs while the remaining joints are pinned to a frozen
// reference pose. The subspace itself is a general
// ompl::base::SubspaceStateSpace (not VAMP-specific) so the same idea
// applies to any OMPL setup whose ambient space is real-vector and whose
// collision backend can operate on the lifted ambient configuration.
// The two VAMP-specific pieces are the subgroup-aware state validity
// checker and motion validator, which read the active indices and frozen
// pose from the subspace and forward to VAMP's SIMD validate_motion.

#include <array>
#include <chrono>
#include <iostream>
#include <vector>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SubspaceStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/vamp/VampStateSpace.h>  // getRobotBounds<Robot>()
#include <ompl/vamp/VampSubgroupMotionValidator.h>
#include <ompl/vamp/VampSubgroupStateValidityChecker.h>

#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;

// Same sphere cage as the full-body Panda demo.
static const std::vector<std::array<float, 3>> obstacles = {
    {0.55, 0, 0.25},  {0.35, 0.35, 0.25},  {0, 0.55, 0.25},   {-0.55, 0, 0.25},   {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25}, {0.35, -0.35, 0.25}, {0.35, 0.35, 0.8}, {0, 0.55, 0.8},     {-0.35, 0.35, 0.8},
    {-0.55, 0, 0.8},  {-0.35, -0.35, 0.8}, {0, -0.55, 0.8},   {0.35, -0.35, 0.8},
};

int main()
{
    vamp::collision::Environment<float> env_float;
    constexpr float radius = 0.2f;
    for (const auto &obs : obstacles)
        env_float.spheres.emplace_back(vamp::collision::factory::sphere::array(obs, radius));
    env_float.sort();
    Environment env(env_float);

    // Reference full-body pose: joints not in the active subset stay here for every
    // collision check the planner performs.
    std::vector<double> frozen{0., -0.785, 0., -2.356, 0., 1.571, 0.785};

    // Plan over the distal four joints {3, 4, 5, 6}; joints 0–2 stay at their frozen values.
    std::vector<std::size_t> active_indices{3, 4, 5, 6};

    auto subspace =
        std::make_shared<ob::SubspaceStateSpace>(ompl::vamp::getRobotBounds<Robot>(), active_indices, frozen);

    std::cout << "Planning over " << subspace->getDimension() << " of " << subspace->getAmbientDimension() << " DOFs"
              << std::endl;
    for (std::size_t i = 0; i < active_indices.size(); ++i)
    {
        std::cout << "  joint " << active_indices[i] << ": [" << subspace->getBounds().low[i] << ", "
                  << subspace->getBounds().high[i] << "]" << std::endl;
    }

    og::SimpleSetup ss(subspace);
    auto si = ss.getSpaceInformation();
    si->setStateValidityChecker(std::make_shared<ompl::vamp::VampSubgroupStateValidityChecker<Robot>>(si, env));
    si->setMotionValidator(std::make_shared<ompl::vamp::VampSubgroupMotionValidator<Robot>>(si, env));

    // Reduced-DOF start and goal — coordinates are in the active subspace, indexed by
    // the order of ``active_indices``. The frozen joints are filled in by the validators.
    ob::ScopedState<> start(subspace);
    ob::ScopedState<> goal(subspace);
    start[0] = -2.356;  // joint 3
    start[1] = 0.0;     // joint 4
    start[2] = 1.571;   // joint 5
    start[3] = 0.785;   // joint 6
    goal[0] = -0.8;
    goal[1] = 0.0;
    goal[2] = 2.5;
    goal[3] = 0.785;
    ss.setStartAndGoalStates(start, goal);

    ss.setPlanner(std::make_shared<og::RRTConnect>(si));

    auto t0 = std::chrono::steady_clock::now();
    auto status = ss.solve(5.0);
    auto t1 = std::chrono::steady_clock::now();
    std::cout << "Planning time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << " ms"
              << std::endl;

    if (status == ob::PlannerStatus::EXACT_SOLUTION)
    {
        ss.simplifySolution();
        auto path = ss.getSolutionPath();
        std::cout << "Solution: " << path.getStateCount() << " waypoints" << std::endl;
        path.print(std::cout);
        return 0;
    }
    std::cout << "No solution found." << std::endl;
    return 1;
}
