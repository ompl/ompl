/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Ryan Luna */

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Kinematic car model object definition.  This class does NOT use ODESolver to propagate the system.
class KinematicCarModel : public oc::StatePropagator
{
    public:
        KinematicCarModel(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
        {
            space_     = si->getStateSpace();
            carLength_ = 0.2;
            timeStep_  = 0.01;
        }

        void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override
        {
            EulerIntegration(state, control, duration, result);
        }

    protected:
        // Explicit Euler Method for numerical integration.
        void EulerIntegration(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
        {
            double t = timeStep_;
            std::valarray<double> dstate;
            space_->copyState(result, start);
            while (t < duration + std::numeric_limits<double>::epsilon())
            {
                ode(result, control, dstate);
                update(result, timeStep_ * dstate);
                t += timeStep_;
            }
            if (t + std::numeric_limits<double>::epsilon() > duration)
            {
                ode(result, control, dstate);
                update(result, (t - duration) * dstate);
            }
        }

        /// implement the function describing the robot motion: qdot = f(q, u)
        void ode(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
        {
            const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
            const double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

            dstate.resize(3);
            dstate[0] = u[0] * cos(theta);
            dstate[1] = u[0] * sin(theta);
            dstate[2] = u[0] * tan(u[1]) / carLength_;
        }

        /// implement y(n+1) = y(n) + d
        void update(ob::State *state, const std::valarray<double> &dstate) const
        {
            ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
            s.setX(s.getX() + dstate[0]);
            s.setY(s.getY() + dstate[1]);
            s.setYaw(s.getYaw() + dstate[2]);
            space_->enforceBounds(state);
        }

        ob::StateSpacePtr        space_;
        double                   carLength_;
        double                   timeStep_;
};

// Definition of the ODE for the kinematic car.
// This method is analogous to the above KinematicCarModel::ode function.
void KinematicCarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    double carLength = 0.2;

    // Zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = u[0] * cos(theta);
    qdot[1] = u[0] * sin(theta);
    qdot[2] = u[0] * tan(u[1]) / carLength;
}

// This is a callback method invoked after numerical integration.
void KinematicCarPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    /// cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    /// extract the first component of the state and cast it to what we expect
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    /// check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}

/// @cond IGNORE
class DemoControlSpace : public oc::RealVectorControlSpace
{
public:

    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
    {
    }
};
/// @endcond

void planWithSimpleSetup()
{
    /// construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    /// set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State *state) { return isStateValid(si, state); });

    // Setting the propagation routine for this space:
    // KinematicCarModel does NOT use ODESolver
    //ss.setStatePropagator(std::make_shared<KinematicCarModel>(ss.getSpaceInformation()));

    // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // when integration has finished to normalize the orientation values.
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &KinematicCarODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));

    /// create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-0.5);
    start->setY(0.0);
    start->setYaw(0.0);

    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(0.0);
    goal->setY(0.5);
    goal->setYaw(0.0);

    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);

    /// we want to have a reasonable value for the propagation step size
    ss.setup();

    /// attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        /// print the path to screen

        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    planWithSimpleSetup();

    return 0;
}
