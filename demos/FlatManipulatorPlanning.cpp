/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Clayton W. Ramsey */

// Plans a Panda through the sphere cage as a differentially flat system.
// The planner's states are joint angles stacked with joint velocities, edges between them are the
// polynomials that spend the least effort getting from one to the other, and the solution comes back as
// a trajectory a controller can run rather than a list of waypoints.
//
// The whole flat setup is the handful of lines under "flat setup" below.
// Everything else is the scenario and the reporting.

#include <array>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/FlatEffortObjective.h>
#include <ompl/base/spaces/FlatStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/FlatTrajectory.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/vamp/VampStateSpace.h>
#include <ompl/vamp/VampStateValidityChecker.h>

#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;

namespace
{
    constexpr unsigned int ORDER = 2;
    constexpr double SOLVE_SECONDS = 10.;

    /// The Panda's velocity limit at each joint in radians per second.
    constexpr std::array<double, Robot::dimension> SPEED_LIMITS{2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};

    /// Construct a sphere cage environment.
    Environment sphereCage()
    {
        vamp::collision::Environment<float> environment;

        const std::vector<std::array<float, 3>> centers = {
            {0.55, 0, 0.25},  {0.35, 0.35, 0.25},  {0, 0.55, 0.25},   {-0.55, 0, 0.25},   {-0.35, -0.35, 0.25},
            {0, -0.55, 0.25}, {0.35, -0.35, 0.25}, {0.35, 0.35, 0.8}, {0, 0.55, 0.8},     {-0.35, 0.35, 0.8},
            {-0.55, 0, 0.8},  {-0.35, -0.35, 0.8}, {0, -0.55, 0.8},   {0.35, -0.35, 0.8},
        };

        for (const auto &center : centers)
            environment.spheres.emplace_back(vamp::collision::factory::sphere::array(center, 0.2f));
        environment.sort();

        return Environment(environment);
    }

    /// Put the arm at \e joints and hold it still there.
    void atRest(const ob::FlatStateSpace *space, ob::State *state, const std::array<double, Robot::dimension> &joints)
    {
        Eigen::MatrixXd flat = Eigen::MatrixXd::Zero(ORDER, Robot::dimension);
        for (unsigned int i = 0; i < Robot::dimension; ++i)
            flat(0, i) = joints[i];
        space->fromFlatState(flat, state);
    }
}  // namespace

int main()
{
    const Environment environment = sphereCage();

    // The flat setup.
    // The joints move independently under separate velocity limits, so the velocity level is a box with
    // a side per joint.
    auto output = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();
    auto space = std::make_shared<ob::FlatStateSpace>(output, ORDER);

    ob::RealVectorBounds speeds(Robot::dimension);
    for (unsigned int i = 0; i < Robot::dimension; ++i)
    {
        speeds.setLow(i, -SPEED_LIMITS[i]);
        speeds.setHigh(i, SPEED_LIMITS[i]);
    }
    space->getDerivativeSpace(1u)->setBounds(speeds);

    og::SimpleSetup setup(space);

    // The collision checker only knows about joint angles, so it gets handed the flat output and nothing
    // else.
    // The bounds go in the same check because the steering polynomial overshoots, and a velocity inside
    // its cap at both ends of an edge runs over it in the middle.
    auto outputInformation = std::make_shared<ob::SpaceInformation>(space->getOutputSpace());
    auto collision = std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(outputInformation, environment);
    setup.setStateValidityChecker(
        [space, collision](const ob::State *state)
        {
            return space->satisfiesBounds(state) &&
                   collision->isValid(state->as<ob::FlatStateSpace::StateType>()->output());
        });

    setup.setOptimizationObjective(std::make_shared<ob::FlatEffortObjective>(setup.getSpaceInformation()));

    // The arm starts tucked and finishes reaching across itself, both at rest.
    ob::ScopedState<> start(space), goal(space);
    atRest(space.get(), start.get(), {0., -0.785, 0., -2.356, 0., 1.571, 0.785});
    atRest(space.get(), goal.get(), {2.35, 1., 0., -0.8, 0., 2.5, 0.785});
    setup.setStartAndGoalStates(start, goal);

    // RRTConnect supports asymmetric edges.
    setup.setPlanner(std::make_shared<og::RRTConnect>(setup.getSpaceInformation()));

    const auto began = std::chrono::steady_clock::now();
    const ob::PlannerStatus status = setup.solve(SOLVE_SECONDS);
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();

    if (status != ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "No solution in " << SOLVE_SECONDS << " seconds\n";
        return 1;
    }

    og::PathGeometric &path = setup.getSolutionPath();
    std::cout << "Solved in " << elapsed << " seconds over " << path.getStateCount() << " states\n";
    std::cout << "The path passes its own validity check: " << std::boolalpha << path.check() << "\n";

    // A controller runs the trajectory.
    const ob::FlatTrajectory trajectory(path);
    std::cout << "Trajectory of " << trajectory.size() << " segments lasting " << trajectory.duration() << " seconds\n";

    const auto objective = setup.getOptimizationObjective();
    ob::Cost cost = objective->identityCost();
    for (std::size_t i = 1; i < path.getStateCount(); ++i)
        cost = objective->combineCosts(cost, objective->motionCost(path.getState(i - 1u), path.getState(i)));
    std::cout << "Steering cost " << cost.value() << "\n";

    // Sampling the whole run confirms every joint held to its limit everywhere, not only at waypoints.
    Eigen::VectorXd rate(Robot::dimension);
    Eigen::VectorXd fastest = Eigen::VectorXd::Zero(Robot::dimension);
    constexpr unsigned int SAMPLES = 20000;
    for (unsigned int i = 0; i <= SAMPLES; ++i)
    {
        trajectory.evaluate(trajectory.duration() * i / SAMPLES, 1u, rate);
        fastest = fastest.cwiseMax(rate.cwiseAbs());
    }

    const Eigen::VectorXd limits = Eigen::Map<const Eigen::VectorXd>(SPEED_LIMITS.data(), Robot::dimension);
    std::cout << "Fastest each joint moves over the run " << fastest.transpose() << "\n";
    std::cout << "Against caps of " << limits.transpose() << "\n";
    return (fastest.array() <= limits.array()).all() ? 0 : 1;
}
