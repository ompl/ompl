/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, University of Santa Cruz Hybrid Systems Laboratory
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
 *   * Neither the name of the University of Santa Cruz nor the names of 
 *     its contributors may be used to endorse or promote products derived
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

/* Author: Beverly Xu */

#include "ompl/control/planners/rrt/HyRRT.h"
#include "ompl/base/StateSpace.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/spaces/HybridStateSpace.h"
#include "ompl/control/ODESolver.h"

struct PinballSetup {
    double paddleLength = 1;
    double paddleWidth = 0.25;
    std::vector<std::vector<double>> paddleCoords = {{1, -1}, {2.5, -4}, {1.5, -5}, {3, -7}, {1, -8}};
};

/** \brief Function computes the Pythagorean distance between two states. */
double distanceFunc(ompl::base::State *state1, ompl::base::State *state2)
{
    double dist = 0;
    dist = hypot(state1->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] - state2->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0],
                    state1->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] - state2->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1]);
    return fabs(dist);
}

/** \brief Individual jump set for a paddle. */
bool inPaddle(ompl::control::HyRRT::Motion *motion, std::vector<double> paddleCoord) 
{
    PinballSetup pinballSetup;

    auto *motion_state = motion->state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double paddleX = paddleCoord[0];
    double paddleY = paddleCoord[1];
    double x1 = motion_state->values[0];
    double x2 = motion_state->values[1];
    double v1 = motion_state->values[2];
    double v2 = motion_state->values[3];

    bool inPaddle = false;

    if ((x1 >= paddleX && x1 <= paddleX + pinballSetup.paddleLength) && (x2 >= paddleY && x2 <= paddleY + pinballSetup.paddleWidth / 2) && v2 < 0)
        inPaddle = true;    // In jump set if both velocity and position vectors are directed toward top of paddle
    else if ((x1 >= paddleX && x1 <= paddleX + pinballSetup.paddleLength) && (x2 >= paddleY + pinballSetup.paddleWidth / 2 && x2 <= paddleY + pinballSetup.paddleWidth) && v2 > 0)
        inPaddle = true;    // In jump set if both velocity and position vectors are directed toward bottom of paddle
    else if ((x1 >= paddleX && x1 <= paddleX + pinballSetup.paddleLength / 4) && (x2 >= paddleY && x2 <= paddleY + pinballSetup.paddleWidth) && v1 > 0)
        inPaddle = true;    // In jump set if both velocity and position vectors are directed toward left side of paddle
    else if ((x1 >= paddleX + (3/4) * pinballSetup.paddleLength && x1 <= paddleX + pinballSetup.paddleLength) && (x2 >= paddleY && x2 <= paddleY + pinballSetup.paddleWidth) && v1 < 0)
        inPaddle = true;    // In jump set if both velocity and position vectors are directed toward left side of paddle

    return inPaddle;
}

/** \brief Jump set is true whenever the ball is colliding with the walls or paddles. */
bool jumpSet(ompl::control::HyRRT::Motion *motion)
{
    PinballSetup pinballSetup;

    auto *motion_state = motion->state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x1 = motion_state->values[0];
    double v1 = motion_state->values[2];

    for (std::vector<double> paddleCoord : pinballSetup.paddleCoords)    // If ball is in any of the paddles
    {
        if (inPaddle(motion, paddleCoord))
            return true;
    }

    if ((x1 <= 0 && v1 < 0) || (x1 >= 5 && v1 > 0)) // If ball is in either side wall
        return true;

    return false;
}

/** \brief Flow set is true whenever the ball is in free space. */
bool flowSet(ompl::control::HyRRT::Motion *motion)
{
    return !jumpSet(motion);
}

/** \brief Unsafe set is below the walls and to the left or right of the goal section's x range. */
bool unsafeSet(ompl::control::HyRRT::Motion *motion)
{
    auto *motion_state = motion->state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x1 = motion_state->values[0];
    double x2 = motion_state->values[1];

    if (((x1 >= 0 && x1 < 1) || (x1 > 4 && x1 <= 5)) && x2 <= -10)
        return true;
    return false;
}

/** \brief Represents the flow map, or the first-order derivative of the pinball state when in flow regime. 
 * The only force applied here is of gravity in the negative y direction. */
void flowODE(const ompl::control::ODESolver::StateType &x_cur, const ompl::control::Control *u,
             ompl::control::ODESolver::StateType &x_new)
{
    (void)u;    // No control is applied when a state is in the flow set

    // Retrieve the current orientation of the pinball.
    const double v_1 = x_cur[2];
    const double v_2 = x_cur[3];
    const double a_1 = x_cur[4];
    const double a_2 = x_cur[5];

    // Ensure qdot is the same size as q.  Zero out all values.
    x_new.resize(x_cur.size(), 0);

    x_new[0] = v_1;
    x_new[1] = v_2;
    x_new[2] = a_1;
    x_new[3] = a_2;
    x_new[4] = 0;    // No change to acceleration
    x_new[5] = 0;    // No change to acceleration
}

/** \brief Simulates the dynamics of the ball when in jump regime, with input from the surface of the paddles and walls. As the collisions
 * are inelastic, there is a coefficient of restitution of 0.8 for the wall and 0.6 for the paddles.
 * Control input it only applied when pinball is colliding with top or sides of the paddle, in which case the input is in the same direction
 * as the horizontal component of the ball's post-collision velocity. 
*/
ompl::base::State *discreteSimulator(ompl::base::State *x_cur, const ompl::control::Control *u, ompl::base::State *new_state)
{
    // Retrieve control values.
    const double *control = u->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double u_x = control[0];

    auto *state_values = x_cur->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    double x1 = state_values[0];
    double x2 = state_values[1];
    double v1 = state_values[2];
    double v2 = state_values[3];
    double a1 = state_values[4];
    double a2 = state_values[5];

    if ((x1 <= 0 && v1 < 0) || (x1 >= 5 && v1 > 0)) // If state is colliding with the wall
    {
        new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = -v1 * 0.8;
        new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[3] = v2;
    }
    else if (v2 < 0)   // If state is colliding with the top of the paddle
    {
        new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = v1 + u_x;
        new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[3] = -v2 * 0.6;
    }
    else if (v2 > 0)   // If state is colliding with the bottom of the paddle
    {
        new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = v1;
        new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[3] = -v2 * 0.6;
    }
    else    // If state is colliding with the side of the paddle
    {
        new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = -(v1 * 0.6 + std::copysign(u_x, v1)); // Input is in the same direction as rebound
        new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[3] = v2;
    }
    
    // The position and acceleration doesn't change
    new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = x1;
    new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = x2;
    new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[4] = a1;
    new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[5] = a2;
    return new_state;
}

// Define goal region [1, 4] x (-inf, -10] x ℝ⁴
class EuclideanGoalRegion : public ompl::base::Goal
{
public:
    EuclideanGoalRegion(const ompl::base::SpaceInformationPtr &si) : ompl::base::Goal(si) {}

    virtual bool isSatisfied(const ompl::base::State *st, double *distance) const
    {
        // perform any operations and return a truth value
        std::vector<double> goalRegion = {1, 4, -10};    // x-min, x-max, y
        auto *values = st->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;

        if (values[0] > goalRegion[1])
            *distance = hypot(values[1] + 10, values[0] - goalRegion[1]);
        else if (values[0] < goalRegion[0])
            *distance = hypot(values[1] + 10, values[0] - goalRegion[0]);
        else
            *distance = values[1] + 10;
        
        return values[0] >= goalRegion[0] && values[0] <= goalRegion[1] && values[1] <= goalRegion[2];
    }

    virtual bool isSatisfied(const ompl::base::State *st) const
    {
        double distance = -1.0;
        return isSatisfied(st, &distance);
    }
};

int main()
{
    // Set the bounds of space
    ompl::base::RealVectorStateSpace *statespace = new ompl::base::RealVectorStateSpace(0);
    statespace->addDimension(0, 5);
    statespace->addDimension(-10, 0);
    statespace->addDimension(-10, 10);
    statespace->addDimension(-10, 10);
    statespace->addDimension(-10, 10);
    statespace->addDimension(-10, 10);
    ompl::base::StateSpacePtr stateSpacePtr(statespace);

    ompl::base::HybridStateSpace *hybridSpace = new ompl::base::HybridStateSpace(stateSpacePtr);
    ompl::base::StateSpacePtr hybridSpacePtr(hybridSpace);

    // Define control space
    ompl::control::RealVectorControlSpace *flowControlSpace = new ompl::control::RealVectorControlSpace(hybridSpacePtr, 1);
    ompl::control::RealVectorControlSpace *jumpControlSpace = new ompl::control::RealVectorControlSpace(hybridSpacePtr, 1);

    ompl::base::RealVectorBounds flowBounds(1);
    flowBounds.setLow(0, 0);
    flowBounds.setHigh(0, 0);
    flowControlSpace->setBounds(flowBounds);

    ompl::base::RealVectorBounds jumpBounds(1);
    jumpBounds.setLow(0, -4);
    jumpBounds.setHigh(0, 4);
    jumpControlSpace->setBounds(jumpBounds);

    ompl::control::RealVectorControlUniformSampler flowControlSampler(flowControlSpace);
    flowControlSpace->setControlSamplerAllocator([flowControlSpace](const ompl::control::ControlSpace *space) -> ompl::control::ControlSamplerPtr {
        return std::make_shared<ompl::control::RealVectorControlUniformSampler>(space);
    });

    ompl::control::RealVectorControlUniformSampler jumpControlSampler(jumpControlSpace);     // Doesn't do anything because the bounds for jump input are just [0, 0], but here for demonstration
    jumpControlSpace->setControlSamplerAllocator([jumpControlSpace](const ompl::control::ControlSpace *space) -> ompl::control::ControlSamplerPtr {
        return std::make_shared<ompl::control::RealVectorControlUniformSampler>(space);
    });

    ompl::control::ControlSpacePtr flowControlSpacePtr(flowControlSpace);
    ompl::control::ControlSpacePtr jumpControlSpacePtr(jumpControlSpace);

    ompl::control::CompoundControlSpace *controlSpace = new ompl::control::CompoundControlSpace(hybridSpacePtr);
    controlSpace->addSubspace(flowControlSpacePtr);
    controlSpace->addSubspace(jumpControlSpacePtr);

    ompl::control::ControlSpacePtr controlSpacePtr(controlSpace);

    // Construct a space information instance for this state space
    ompl::control::SpaceInformationPtr si(new ompl::control::SpaceInformation(hybridSpacePtr, controlSpacePtr));
    ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (si, &flowODE));
    
    si->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    si->setPropagationStepSize(0.01);
    si->setup();
    
    // Create a problem instance
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

    std::vector<ompl::base::ScopedState<>> startStates;
    std::vector<double> startXs = {0.5, 1, 2, 3.5, 4, 4.5};

    for (std::size_t i = 0; i < startXs.size(); i++) 
    {
        startStates.push_back(ompl::base::ScopedState<>(hybridSpacePtr));
        startStates.back()->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = startXs[i];
        startStates.back()->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = 0;
        startStates.back()->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = 0;
        startStates.back()->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[3] = 0;
        startStates.back()->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[4] = 0;
        startStates.back()->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[5] = -9.81;
        pdef->addStartState(startStates.back());
    }

    // Set goal state to be on floor with a zero velocity, and tolerance of 0.1
    std::shared_ptr<EuclideanGoalRegion> goal = std::make_shared<EuclideanGoalRegion>(si);

    // Set the goal region
    pdef->setGoal(goal);

    ompl::control::HyRRT cHyRRT(si);

    // Set the problem instance for our planner to solve
    cHyRRT.setProblemDefinition(pdef);
    cHyRRT.setup();

    // Set parameters
    cHyRRT.setDiscreteSimulator(discreteSimulator);
    cHyRRT.setDistanceFunction(distanceFunc);
    cHyRRT.setFlowSet(flowSet);
    cHyRRT.setJumpSet(jumpSet);
    cHyRRT.setTm(0.5);
    cHyRRT.setFlowStepDuration(0.01);
    cHyRRT.setUnsafeSet(unsafeSet);

    // attempt to solve the planning problem within 15 seconds
    ompl::time::point t0 = ompl::time::now();
    ompl::base::PlannerStatus solved = cHyRRT.solve(ompl::base::timedPlannerTerminationCondition(15));
    double planTime = ompl::time::seconds(ompl::time::now() - t0);

    if (solved)  // If either approximate or exact solution has beenf ound
        OMPL_INFORM("Solution found in %f seconds", planTime);
    OMPL_INFORM("Solution status: %s", solved.asString().c_str());

    // print path
    pdef->getSolutionPath()->as<ompl::control::PathControl>()->printAsMatrix(std::cout);
}