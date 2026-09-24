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

// Plans a quadrotor through a room of pillars and out a window as a differentially flat system.
// A quadrotor's flat output is its position and its yaw, so the flat output space here is R^3 and SO(2)
// side by side, and yaw wraps around.
// The rotor forces can then be derived from the flat output state and its derivatives.

#include <cmath>
#include <iostream>
#include <memory>

#include <boost/math/constants/constants.hpp>

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/FlatEffortObjective.h>
#include <ompl/base/spaces/FlatStateSpace.h>
#include <ompl/base/spaces/FlatTrajectory.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    constexpr unsigned int ORDER = 2;
    constexpr double SOLVE_SECONDS = 10.;
    constexpr double GRAVITY = 9.81;

    /// size of the quadrotor, in meters.
    constexpr double BODY_RADIUS = 0.3;

    /// The quadrotor's limits on speed along each axis and on yaw rate.
    constexpr double SPEED_LIMIT = 3.;      // m/s
    constexpr double YAW_RATE_LIMIT = 1.5;  // rad/s

    /// The room runs 10 meters on a side and 4 meters up.
    constexpr double HALF_WIDTH = 5.;
    constexpr double HEIGHT = 4.;

    /// A row of pillars stands on either side of a wall splitting the room at x of zero, and a window in
    /// the wall is the only way through.
    constexpr double PILLAR_RADIUS = 0.4;
    constexpr double WALL_HALF_THICKNESS = 0.1;
    constexpr double WINDOW_LOW_Y = 1., WINDOW_HIGH_Y = 2.5;
    constexpr double WINDOW_LOW_Z = 1.5, WINDOW_HIGH_Z = 2.8;

    /// Whether a quadrotor centered at \e x, \e y, \e z clears every obstacle in the room.
    bool collisionFree(double x, double y, double z)
    {
        for (double px : {-2.5, 2.5})
            for (double py : {-3., -1., 1., 3.})
                if (std::hypot(x - px, y - py) < PILLAR_RADIUS + BODY_RADIUS)
                    return false;

        if (std::abs(x) < WALL_HALF_THICKNESS + BODY_RADIUS)
        {
            const bool throughWindow = y > WINDOW_LOW_Y + BODY_RADIUS && y < WINDOW_HIGH_Y - BODY_RADIUS &&
                                       z > WINDOW_LOW_Z + BODY_RADIUS && z < WINDOW_HIGH_Z - BODY_RADIUS;
            if (!throughWindow)
                return false;
        }

        return z > BODY_RADIUS && z < HEIGHT - BODY_RADIUS;
    }

    /// The position of the flat output in \e state.
    const double *positionOf(const ob::State *state)
    {
        return state->as<ob::FlatStateSpace::StateType>()
            ->output()
            ->as<ob::CompoundState>()
            ->as<ob::RealVectorStateSpace::StateType>(0)
            ->values;
    }

    /// Hover at \e x, \e y, \e z facing \e yaw.
    void hover(const ob::FlatStateSpace *space, ob::State *state, double x, double y, double z, double yaw)
    {
        Eigen::MatrixXd flat = Eigen::MatrixXd::Zero(ORDER, 4);
        flat.row(0) << x, y, z, yaw;
        space->fromFlatState(flat, state);
    }
}  // namespace

int main()
{
    const double pi = boost::math::constants::pi<double>();

    // The flat setup.
    auto position = std::make_shared<ob::RealVectorStateSpace>(3);
    ob::RealVectorBounds room(3);
    room.setLow(0, -HALF_WIDTH);
    room.setHigh(0, HALF_WIDTH);
    room.setLow(1, -HALF_WIDTH);
    room.setHigh(1, HALF_WIDTH);
    room.setLow(2, 0.);
    room.setHigh(2, HEIGHT);
    position->setBounds(room);

    auto output = std::make_shared<ob::CompoundStateSpace>();
    output->addSubspace(position, 1.);
    output->addSubspace(std::make_shared<ob::SO2StateSpace>(), 0.5);

    auto space = std::make_shared<ob::FlatStateSpace>(output, ORDER);

    ob::RealVectorBounds rates(4);
    for (unsigned int i = 0; i < 3; ++i)
    {
        rates.setLow(i, -SPEED_LIMIT);
        rates.setHigh(i, SPEED_LIMIT);
    }
    rates.setLow(3, -YAW_RATE_LIMIT);
    rates.setHigh(3, YAW_RATE_LIMIT);
    space->getDerivativeSpace(1u)->setBounds(rates);

    og::SimpleSetup setup(space);

    // validates bounds and collision
    setup.setStateValidityChecker(
        [space](const ob::State *state)
        {
            const double *p = positionOf(state);
            return space->satisfiesBounds(state) && collisionFree(p[0], p[1], p[2]);
        });

    setup.setOptimizationObjective(std::make_shared<ob::FlatEffortObjective>(setup.getSpaceInformation()));

    // set start and goal states.
    ob::ScopedState<> start(space), goal(space);
    hover(space.get(), start.get(), -4., -4., 1., 3.);
    hover(space.get(), goal.get(), 4., 4., 2.5, -3.);
    setup.setStartAndGoalStates(start, goal);

    // RRTConnect supports asymmetric edges.
    setup.setPlanner(std::make_shared<og::RRTConnect>(setup.getSpaceInformation()));

    if (setup.solve(SOLVE_SECONDS) != ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "No solution in " << SOLVE_SECONDS << " seconds\n";
        return 1;
    }

    // Apply path simplification.
    og::PathGeometric &path = setup.getSolutionPath();
    const std::size_t planned = path.getStateCount();
    og::PathSimplifier(setup.getSpaceInformation()).reduceVertices(path);
    std::cout << "Solved over " << planned << " states, reduced to " << path.getStateCount() << "\n";
    std::cout << "The path passes its own validity check: " << std::boolalpha << path.check() << "\n";

    const ob::FlatTrajectory trajectory(path);
    std::cout << "Trajectory of " << trajectory.size() << " segments lasting " << trajectory.duration() << " seconds\n";

    // A quadrotor's thrust points along its body axis, so the rotors have to supply the acceleration the
    // trajectory asks for plus whatever holds the airframe up against gravity.
    // Its direction gives the tilt and its size over gravity gives the thrust to weight ratio.
    Eigen::VectorXd acceleration(4);
    Eigen::VectorXd rate(4);
    double peakThrust = 0.;
    double peakTilt = 0.;
    double peakSpeed = 0.;
    constexpr unsigned int SAMPLES = 20000;
    for (unsigned int i = 0; i <= SAMPLES; ++i)
    {
        const double t = trajectory.duration() * i / SAMPLES;
        trajectory.evaluate(t, 2u, acceleration);
        trajectory.evaluate(t, 1u, rate);

        const Eigen::Vector3d thrust(acceleration[0], acceleration[1], acceleration[2] + GRAVITY);
        peakThrust = std::max(peakThrust, thrust.norm() / GRAVITY);
        peakTilt = std::max(peakTilt, std::acos(thrust.z() / thrust.norm()));
        peakSpeed = std::max(peakSpeed, rate.head<3>().cwiseAbs().maxCoeff());
    }

    std::cout << "Fastest along any axis " << peakSpeed << " meters per second against a cap of " << SPEED_LIMIT
              << "\n";
    std::cout << "Peak thrust " << peakThrust << " times the weight, peak tilt " << peakTilt * 180. / pi
              << " degrees\n";

    // Level 0 of the trajectory unwraps the yaw, so it finishes a whole number of turns from the goal
    // heading rather than jumping at the branch cut.
    std::cout << "Yaw runs from " << trajectory.evaluate(0.)[3] << " to "
              << trajectory.evaluate(trajectory.duration())[3] << " radians unwrapped\n";

    return path.check() ? 0 : 1;
}
