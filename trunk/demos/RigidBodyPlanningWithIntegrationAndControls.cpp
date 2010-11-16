/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/GoalState.h>
#include <ompl/base/manifolds/SE2StateManifold.h>
#include <ompl/control/manifolds/RealVectorControlManifold.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>

namespace ob = ompl::base;
namespace oc = ompl::control;


/// Model defining the motion of the robot
class KinematicCarModel
{
public:
    
    KinematicCarModel(const ob::StateManifold *manifold) : manifold_(manifold), carLength_(0.2)
    {
    }
    
    /// implement the function describing the robot motion: qdot = f(q, u)
    void operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
    {	
	const double *u = control->as<oc::RealVectorControlManifold::ControlType>()->values;
	const double theta = state->as<ob::SE2StateManifold::StateType>()->getYaw();
	
	dstate.resize(3);
	dstate[0] = u[0] * cos(theta);
	dstate[1] = u[0] * sin(theta);
	dstate[2] = u[0] * tan(u[1]) / carLength_;
    }
    
    /// implement y(n+1) = y(n) + d
    void update(ob::State *state, const std::valarray<double> &dstate) const
    {
	ob::SE2StateManifold::StateType &s = *state->as<ob::SE2StateManifold::StateType>();
	s.setX(s.getX() + dstate[0]);
	s.setY(s.getY() + dstate[1]);
	s.setYaw(s.getYaw() + dstate[2]);
	manifold_->enforceBounds(state);
    }

private:

    const ob::StateManifold *manifold_;
    const double             carLength_;
    
};


/// Simple integrator: Euclidean method
template<typename F>
class EulerIntegrator
{
public:
    
    EulerIntegrator(const ob::StateManifold *manifold, double timeStep) : manifold_(manifold), timeStep_(timeStep), ode_(manifold)
    {
    }
    
    void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
    {
	double t = timeStep_;
	std::valarray<double> dstate;
	manifold_->copyState(result, start);
	while (t < duration + std::numeric_limits<double>::epsilon())
	{
	    ode_(result, control, dstate);
	    ode_.update(result, timeStep_ * dstate);
	    t += timeStep_;
	}
	if (t + std::numeric_limits<double>::epsilon() > duration)
	{
	    ode_(result, control, dstate);
	    ode_.update(result, (t - duration) * dstate);
	}
    }
   
    double getTimeStep(void) const
    {
	return timeStep_;
    }
    
    void setTimeStep(double timeStep)
    {
	timeStep_ = timeStep;
    }
    
private:

    const ob::StateManifold *manifold_;
    double                   timeStep_;
    F                        ode_;    
};

    
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateManifold>
    /// cast the abstract state type to the type we expect
    const ob::SE2StateManifold::StateType *se2state = state->as<ob::SE2StateManifold::StateType>();

    /// extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateManifold::StateType *pos = se2state->as<ob::RealVectorStateManifold::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    const ob::SO2StateManifold::StateType *rot = se2state->as<ob::SO2StateManifold::StateType>(1);
    
    /// check validity of state defined by pos & rot
    
    
    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (void*)rot != (void*)pos;
}

/// @cond IGNORE
class DemoControlManifold : public oc::RealVectorControlManifold
{
public:
    
    DemoControlManifold(const ob::StateManifoldPtr &stateManifold) : oc::RealVectorControlManifold(stateManifold, 2),
								     integrator_(stateManifold.get(), 0.0)
    {
    }
    
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const
    {
	integrator_.propagate(state, control, duration, result);
    }
    
    void setIntegrationTimeStep(double timeStep)
    {
	integrator_.setTimeStep(timeStep);
    }
    
    double getIntegrationTimeStep(void) const
    {
	return integrator_.getTimeStep();
    }
        
    EulerIntegrator<KinematicCarModel> integrator_;
    
};
/// @endcond

void planWithSimpleSetup(void)
{
    /// construct the manifold we are planning in
    ob::StateManifoldPtr manifold(new ob::SE2StateManifold());

    /// set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    
    manifold->as<ob::SE2StateManifold>()->setBounds(bounds);

    // create a control manifold
    oc::ControlManifoldPtr cmanifold(new DemoControlManifold(manifold));
    
    // set the bounds for the control manifold
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);
    
    cmanifold->as<DemoControlManifold>()->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cmanifold);

    /// set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1));
    
    /// create a start state
    ob::ScopedState<ob::SE2StateManifold> start(manifold);
    start->setX(-0.5);
    start->setY(0.0);
    start->setYaw(0.0);
    
    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::SE2StateManifold> goal(manifold);
    goal->setX(0.0);
    goal->setY(0.5);
    goal->setYaw(0.0);

    /// set the start and goal states; this call allows SimpleSetup to infer the planning manifold, if needed
    ss.setStartAndGoalStates(start, goal, 0.05);
    
    /// we want to have a reasonable value for the propagation step size
    ss.setup();
    cmanifold->as<DemoControlManifold>()->setIntegrationTimeStep(ss.getSpaceInformation()->getPropagationStepSize());
    
    /// attempt to solve the problem within one second of planning time
    bool solved = ss.solve(10.0);

    if (solved)
    {
	std::cout << "Found solution:" << std::endl;
	/// print the path to screen

	ss.getSolutionPath().asGeometric().print(std::cout);
    }
    else
	std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
    std::cout << "ompl version: " << OMPL_VERSION << std::endl;
   
    planWithSimpleSetup();
    
    return 0;
}
