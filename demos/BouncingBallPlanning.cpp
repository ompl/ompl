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

/** \brief Function computes the Pythagorean distance between two states. */
double distanceFunc(ompl::base::State *state1, ompl::base::State *state2)
{
    double dist = 0;
    dist = sqrt(pow(state1->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] - state2->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0], 2) + pow(state1->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] - state2->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1], 2));
    return fabs(dist);
}

/** \brief Jump set is true whenever the ball is on or below the surface and has a downwards velocity. */
bool jumpSet(ompl::control::HyRRT::Motion *motion)
{
    auto *motion_state = motion->state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double velocity = motion_state->values[1];
    double pos_cur = motion_state->values[0];

    if (pos_cur <= 0 && velocity <= 0)
        return true;
    else
        return false;
}

/** \brief Flow set is true whenever the ball is above the surface or has an upwards velocity. */
bool flowSet(ompl::control::HyRRT::Motion *motion)
{
    return !jumpSet(motion);
}

/** \brief Unsafe set is true whenever the ball is above 10 units from the ground, to reduce time spent planning. */
bool unsafeSet(ompl::control::HyRRT::Motion *motion)
{
    double u = motion->control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0];
    if (u > 5 || u < 0)
        return true;
    return false;
}

/** \brief Represents the flow map, or the first-order derivative of the pinball state when in flow regime. 
 * The only force applied here is of gravity in the negative y direction. */
void flowODE(const ompl::control::ODESolver::StateType& x_cur, const ompl::control::Control* u, ompl::control::ODESolver::StateType& x_new)
{
    (void)u;    // No control is applied when a state is in the flow set

    // Ensure qdot is the same size as q.  Zero out all values.
    x_new.resize(x_cur.size(), 0);
 
    x_new[0] = x_cur[1];            // x-dot
    x_new[1] = x_cur[2];            // v-dot
    x_new[2] = 0;    // a-dot
}

/** \brief Simulates the dynamics of the ball when in jump regime, with input from the surface. */
ompl::base::State *discreteSimulator(ompl::base::State *x_cur, const ompl::control::Control *u, ompl::base::State *new_state)
{
    double velocity = -0.8 * x_cur->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];

    new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = x_cur->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
    new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = velocity - u->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0];
    new_state->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = x_cur->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2];
    return new_state;
}

// Define goal region as a ball of radius 0.1 centered at {height, velocity} = {0, 0}
class EuclideanGoalRegion : public ompl::base::Goal
{
public:
    EuclideanGoalRegion(const ompl::base::SpaceInformationPtr &si) : ompl::base::Goal(si)
    {
    }

    virtual bool isSatisfied(const ompl::base::State *st, double *distance) const
    {
        // perform any operations and return a truth value
        std::vector<double> goal = {0, 0};
        double distSqr = 0;
        for (int i = 0; i < 2; i++)
        {
            distSqr +=
                pow(st->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[i] -
                        goal[i],
                    2);
        }

        *distance = sqrt(distSqr);

        if (*distance < 0.1)
            return true;
        else
            return false;
    }

    virtual bool isSatisfied(const ompl::base::State *st) const
    {
        double distance = 0.0;
        return isSatisfied(st, &distance);
    }
};

int main()
{
    // Set the bounds of space
    ompl::base::RealVectorStateSpace *statespace = new ompl::base::RealVectorStateSpace(0);
    statespace->addDimension(-10, 10);
    statespace->addDimension(-10, 10);
    statespace->addDimension(-10, 5);
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
    jumpBounds.setLow(0, 0);
    jumpBounds.setHigh(0, 5);
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

    // Set start state to be 1 unit above the floor with zero velocity and gravitaional acceleration of 9.81
    ompl::base::ScopedState<> start(hybridSpacePtr);
    start->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = 1;
    start->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = 0;
    start->as<ompl::base::HybridStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = -9.81;

    // Set goal state to be on floor with a zero velocity, and tolerance of 0.1
    std::shared_ptr<EuclideanGoalRegion> goal = std::make_shared<EuclideanGoalRegion>(si);

    // Create a problem instance
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

    // Set the start and goal states
    pdef->addStartState(start);
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

    // attempt to solve the planning problem within 2 seconds
    ompl::time::point t0 = ompl::time::now();
    ompl::base::PlannerStatus solved = cHyRRT.solve(ompl::base::timedPlannerTerminationCondition(2));
    double planTime = ompl::time::seconds(ompl::time::now() - t0);

    if (solved)  // If either approximate or exact solution has beenf ound
        OMPL_INFORM("Solution found in %f seconds", planTime);
    OMPL_INFORM("Solution status: %s", solved.asString().c_str());

    // print path
    pdef->getSolutionPath()->as<ompl::control::PathControl>()->printAsMatrix(std::cout);
}