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

#include <ompl/extensions/ode/ODESimpleSetup.h>
#include <ompl/base/GoalRegion.h>
#include <ompl/config.h>
#include <iostream>
#include <ode/ode.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

class RigidBodyEnvironment : public oc::ODEEnvironment
{
public:

    RigidBodyEnvironment(void) : oc::ODEEnvironment()
    {
	createWorld();
    }
    
    virtual ~RigidBodyEnvironment(void)
    {
	destroyWorld();
    }
    
    /**************************************************
     * Implementation of functions needed by planning *
     **************************************************/

    virtual unsigned int getControlDimension(void) const
    {
	return 3;
    }
    
    virtual void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const
    {
	static double maxForce = 0.2;
	lower.resize(3);
	lower[0] = -maxForce;
	lower[1] = -maxForce;
	lower[2] = -maxForce;
	
	upper.resize(3);
	upper[0] = maxForce;
	upper[1] = maxForce;
	upper[2] = maxForce;
    }
    
    virtual void applyControl(const double *control) const
    {
	dBodyAddForce(boxBody, control[0], control[1], control[2]);
    }
    
    virtual bool isValidCollision(dGeomID geom1, dGeomID geom2, const dContact& contact) const
    {
	return false;
    }
    
    virtual void setupContact(dContact &contact) const
    {
	contact.surface.mode = dContactSoftCFM | dContactApprox1;
	contact.surface.mu = 0.9;
	contact.surface.soft_cfm = 0.2;
    }

    /**************************************************/


    // OMPL does not require this function here; we implement it here
    // for convenience. This function is only ODE code to create a
    // simulation environment. At the end of the function, there is a
    // call to setPlanningParameters(), which configures members of
    // the base class needed by planners.
    void createWorld(void);

    // Clear all ODE objects
    void destroyWorld(void);
    
    // Set parameters needed by the base class (such as the bodies
    // that make up to state of the system we are planning for)
    void setPlanningParameters(void);

    // the simulation world
    dWorldID bodyWorld;

    // the space for all objects
    dSpaceID space;

    // the car mass
    dMass    m;  
  
    // the body geom
    dGeomID  boxGeom;	
    
    // the body
    dBodyID  boxBody;	
	
};


// Define the goal we want to reach
class RigidBodyGoal : public ob::GoalRegion
{
public:
    
    RigidBodyGoal(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si)
    {
	threshold_ = 0.5;
    }
    
    virtual double distanceGoal(const ob::State *st) const
    {
	const double *pos = st->as<oc::ODEStateManifold::StateType>()->getBodyPosition(0);
	double dx = fabs(pos[0] - 30);
	double dy = fabs(pos[1] - 55);
	double dz = fabs(pos[2] - 35);
	return sqrt(dx * dx + dy * dy + dz * dz);
    }
    
};  

// Define how we project a state 
class RigidBodyStateProjectionEvaluator : public ob::ProjectionEvaluator
{
public: 

    RigidBodyStateProjectionEvaluator(const ob::StateManifold *manifold) : ob::ProjectionEvaluator(manifold)
    {
	cellDimensions_.resize(3);
	cellDimensions_[0] = 1;
	cellDimensions_[1] = 1;
	cellDimensions_[2] = 1;
    }
    
    virtual unsigned int getDimension(void) const
    {
	return 3;
    }
    
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const
    {
	const double *pos = state->as<oc::ODEStateManifold::StateType>()->getBodyPosition(0);
	projection[0] = pos[0];
	projection[1] = pos[1];
	projection[2] = pos[2];
    }
    
};

// Define our own manifold, to include a distance function we want and register a default projection
class RigidBodyStateManifold : public oc::ODEStateManifold
{
public:

    RigidBodyStateManifold(const RigidBodyEnvironment &env) : oc::ODEStateManifold(env)
    {
    }
    
    virtual double distance(const ob::State *s1, const ob::State *s2) const
    {
	const double *p1 = s1->as<oc::ODEStateManifold::StateType>()->getBodyPosition(0);
	const double *p2 = s1->as<oc::ODEStateManifold::StateType>()->getBodyPosition(0);
	double dx = fabs(p1[0] - p2[0]);
	double dy = fabs(p1[1] - p2[1]);
	double dz = fabs(p1[2] - p2[2]);
	return sqrt(dx * dx + dy * dy + dz * dz);
    }

    virtual void registerProjections(void)
    {
    	registerDefaultProjection(ob::ProjectionEvaluatorPtr(new RigidBodyStateProjectionEvaluator(this)));
    }
    
};

int main(int, char **)
{
    // initialize ODE
    dInitODE2(0);
    
    // create the ODE environment
    RigidBodyEnvironment env;
    
    // create the state space manifold and the control manifold for planning
    RigidBodyStateManifold *stateManifold = new RigidBodyStateManifold(env);
    ob::StateManifoldPtr stateManifoldPtr = ob::StateManifoldPtr(stateManifold);
    
    // this will take care of setting a proper collision checker and the starting state for the planner as the initial ODE state
    oc::ODESimpleSetup ss(stateManifoldPtr);

    // set the goal we would like to reach
    ss.setGoal(ob::GoalPtr(new RigidBodyGoal(ss.getSpaceInformation())));
    
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-200);
    bounds.setHigh(200);
    stateManifold->setVolumeBounds(bounds);

    bounds.setLow(-20);
    bounds.setHigh(20);
    stateManifold->setLinearVelocityBounds(bounds);
    stateManifold->setAngularVelocityBounds(bounds);

    ss.setup();
    ss.print();
    
    if (ss.solve(10))
    {
	ob::PathPtr p = ss.getSolutionPath().asGeometric();
	const std::vector<ob::State*> &states = static_cast<og::PathGeometric*>(p.get())->states;
        for (unsigned int i = 0 ; i < states.size() ; ++i)
	{
	    const double *pos = states[i]->as<ob::CompoundState>()->as<ob::RealVectorStateManifold::StateType>(0)->values;
	    std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
	}
    }
    
    dCloseODE();
    
    return 0;
}









/***********************************************
 * Member function implementations             *
 ***********************************************/

void RigidBodyEnvironment::createWorld(void)
{
    // BEGIN SETTING UP AN ODE ENVIRONMENT
    // ***********************************
    
    bodyWorld = dWorldCreate();
    space = dHashSpaceCreate(0);
    
    dWorldSetGravity(bodyWorld, 0, 0, -0.981);
    
    double lx = 0.2;
    double ly = 0.2;
    double lz = 0.1;
    
    dMassSetBox(&m, 1, lx, ly, lz);

    boxGeom = dCreateBox(space, lx, ly, lz);
    boxBody = dBodyCreate(bodyWorld);
    dBodySetMass(boxBody, &m);
    dGeomSetBody(boxGeom, boxBody);
    
    // *********************************
    // END SETTING UP AN ODE ENVIRONMENT
    
    setPlanningParameters();
}

void RigidBodyEnvironment::destroyWorld(void)
{
    dSpaceDestroy(space);
    dWorldDestroy(bodyWorld);
}

void RigidBodyEnvironment::setPlanningParameters(void)
{    
    /// Fill in parameters for OMPL:
    world_ = bodyWorld;
    collisionSpaces_.push_back(space);
    stateBodies_.push_back(boxBody);
    stepSize_ = 0.05;
    maxContacts_ = 3;
    minControlSteps_ = 10;
    maxControlSteps_ = 500;
}
