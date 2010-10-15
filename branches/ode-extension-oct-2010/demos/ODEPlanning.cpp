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

#include <ompl/extensions/ode/ODEControlManifold.h>
#include <ompl/base/GoalRegion.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/version.h>
#include <iostream>

#include <ode/ode.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class ODECarEnvironment : public oc::ODEEnvironment
{
public:

    ODECarEnvironment(void) : oc::ODEEnvironment()
    {
    }

    /**************************************************
     * Implementation of functions needed by planning *
     **************************************************/

    virtual unsigned int getControlDimension(void) const
    {
	return 2;
    }
    
    virtual void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const
    {
	static double maxSteer = 3;
	static double maxSpeed = 5;
	lower.resize(2);
	lower[0] = -maxSpeed;
	lower[1] = -maxSteer;
	
	upper.resize(2);
	upper[0] = maxSpeed;
	upper[1] = maxSteer;
    }
    
    virtual void applyControl(const double *control) const
    {
	static double maxturn = 0.53; // 30 degrees
	static double maxforce = 0.1;
	dReal speed = control[0]; dReal steer = control[1];
	
	dJointSetHinge2Param(joint[0], dParamVel2, speed);
	dJointSetHinge2Param(joint[0], dParamFMax2, maxforce);
	dJointSetHinge2Param(joint[3], dParamVel2, speed);
	dJointSetHinge2Param(joint[3], dParamFMax2, maxforce);
	
	// steering
	dReal v = steer - dJointGetHinge2Angle1(joint[0]);
	if (v > 0.1) v = 0.1;
	if (v < -0.1) v = -0.1;

	dJointSetHinge2Param(joint[0], dParamVel,v);
	dJointSetHinge2Param(joint[0], dParamFMax,0.2);
	dJointSetHinge2Param(joint[0], dParamLoStop,-maxturn);
	dJointSetHinge2Param(joint[0], dParamHiStop,maxturn);
	dJointSetHinge2Param(joint[0], dParamFudgeFactor,0.1);
	dJointSetHinge2Param(joint[3], dParamVel,v);
	dJointSetHinge2Param(joint[3], dParamFMax,0.2);
	dJointSetHinge2Param(joint[3], dParamLoStop,-maxturn);
	dJointSetHinge2Param(joint[3], dParamHiStop,maxturn);
	dJointSetHinge2Param(joint[3], dParamFudgeFactor,0.1);
    }
    
    virtual bool isValidCollision(dGeomID geom1, dGeomID geom2, dContact& contact) const
    {
	return (geom1 == ground || geom2 == ground);
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
    
    // Set parameters needed by the base class (such as the bodies
    // that make up to state of the system we are planning for)
    void setPlanningParameters(void);

    // the simulation world
    dWorldID carWorld;

    // the space for all objects
    dSpaceID space;

    // the ground plane
    dGeomID  ground;

    // the car mass
    dMass    m;    

    // the car joints
    dJointID joint[4];

    // the car geoms (4 wheels + body)
    dGeomID  gw[4], boxGeom;	
    
    // the car bodies (4 wheels + body)
    dBodyID  bw[4], boxBody;	
	
};


// Define the goal we want to reach
class CarGoal : public ob::GoalRegion
{
public:
    CarGoal(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si)
    {
	threshold_ = 1.0;
    }
    
    virtual double distanceGoal(const ob::State *st) const
    {
	const double *pos = st->as<oc::ODEStateManifold::StateType>()->getBodyPosition(0);
	std::cerr << pos[0] << " " << pos[1] << std::endl;
	
	double dx = fabs(pos[0] - 30);
	double dy = fabs(pos[1] - 55);
	return sqrt(dx * dx + dy * dy);
    }
    
};  

void plan(void)
{ 
}

int main(int, char **)
{
    // initialize ODE
    dInitODE2(0);
    
    // create the ODE environment
    ODECarEnvironment env;
    env.createWorld();
    
    // create the state space manifold and the control manifold for planning
    oc::ODEStateManifold *stateManifold = new oc::ODEStateManifold(env);
    oc::ODEControlManifold *odeCM = new oc::ODEControlManifold(ob::StateManifoldPtr(stateManifold));
    oc::ControlManifoldPtr cm(odeCM);
    oc::SimpleSetup ss(cm);


    // set the current state of the ODE world as the start state for planning    
    ob::ScopedState<> start(ss.getStateManifold());
    stateManifold->readState(start.get());
    ss.addStartState(start);

    // set the goal we would like to reach
    ss.setGoal(ob::GoalPtr(new CarGoal(ss.getSpaceInformation())));

    // set further needed parameters
    ss.getSpaceInformation()->setPropagationStepSize(env.stepSize);
    ss.getSpaceInformation()->setMinMaxControlDuration(130, 180);
    

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1000);
    bounds.setHigh(1000);
    stateManifold->setVolumeBounds(bounds);

    bounds.setLow(-20);
    bounds.setHigh(20);
    stateManifold->setLinearVelocityBounds(bounds);
    stateManifold->setAngularVelocityBounds(bounds);

    
    ss.setup();
    ss.print();
    
    oc::Control *c = odeCM->allocControl();
    oc::ControlSamplerPtr cs = odeCM->allocControlSampler();
    cs->sample(c);

    /*    
    ob::ScopedState<oc::ODEStateManifold> dest = start;
    odeCM->propagate(start.get(), c, 10, dest.get());
    dest.print();
    */

    /*
    std::vector<ob::State*> dest;
    ss.getSpaceInformation()->propagate(start.get(), c, 100, dest, true);
    std::cout << "B=[";
    for (unsigned int i = 0 ; i < dest.size() ; ++i)
    {
	const double *pos = dest[i]->as<ob::CompoundState>()->as<ob::RealVectorStateManifold::StateType>(0)->values;
	std::cout << pos[0] << " " << pos[1] << std::endl;
    }
    std::cout << "];" << std::endl;
    std::cout << "plot(B(:,1),B(:,2))" << std::endl;
    */

    ss.solve();
    
    ob::PlannerData data;
    ss.getPlanner()->getPlannerData(data);
    for (unsigned int i = 0 ; i < data.states.size() ; ++i)
    {
	const double *pos = data.states[i]->as<ob::CompoundState>()->as<ob::RealVectorStateManifold::StateType>(0)->values;
	std::cout << pos[0] << " " << pos[1] << std::endl;
    }
    
    
    dCloseODE();
    
    return 0;
}












/***********************************************
 * Member function implementations             *
 ***********************************************/

void ODECarEnvironment::createWorld(void)
{
    // BEGIN SETTING UP AN ODE ENVIRONMENT
    // ***********************************
    
    carWorld = dWorldCreate();
    space = dHashSpaceCreate(0);
    
    dWorldSetGravity(carWorld, 0, 0, -0.5);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    
    double wradius  = 0.4;
    
    double
	carx = 2,
	cary = 0.4,
	carz = 3.5,
	cary_ = 0.7,
	cz = wradius;
    
    dMassSetSphere (&m, 1, 1);
    dMassAdjust(&m, 0.1); /// the mass of our wheels
    
    // roue 1 (wheel1)
    bw[0] = dBodyCreate(carWorld);
    dBodySetMass(bw[0], &m);
    gw[0] = dCreateSphere(space, wradius);
    dGeomSetBody(gw[0], bw[0]);
    dBodySetPosition(bw[0], 0.8, 1.5, cz);
    
    // roue 2 (wheel2)
    bw[1] = dBodyCreate(carWorld);
    dBodySetMass(bw[1], &m);
    gw[1] = dCreateSphere(space, wradius);
    dGeomSetBody(gw[1], bw[1]);
    dBodySetPosition(bw[1], 0.8, -1.45, cz);
    
    // roue 3 (wheel3)
    bw[2] = dBodyCreate(carWorld);
    dBodySetMass(bw[2], &m);
    gw[2] = dCreateSphere(space, wradius);
    dGeomSetBody(gw[2], bw[2]);
    dBodySetPosition(bw[2], -0.8, -1.45, cz);
    
    // roue 4 (wheel4)
    bw[3] = dBodyCreate(carWorld);
    dBodySetMass(bw[3], &m);
    gw[3] = dCreateSphere(space, wradius);
    dGeomSetBody(gw[3], bw[3]);
    dBodySetPosition(bw[3], -0.8, 1.5, cz);
    
    /// CAR box
    dMassSetBox(&m, 1, 1, 1, 1);
    dMassAdjust(&m, 10);
    boxBody = dBodyCreate(carWorld);
    dBodySetMass(boxBody, &m);
    boxGeom = dCreateBox(space, carx, carz, cary);
    dGeomSetBody(boxGeom, boxBody);
    dGeomSetPosition(boxGeom, 0, 0, cary_);
    
    /// front and back wheel hinges
    for ( int i = 0; i <= 3; i++)
    {
	joint[i] = dJointCreateHinge2(carWorld, 0);
	dJointAttach(joint[i], boxBody, bw[i]);
	const dReal *a = dBodyGetPosition(bw[i]);
	dJointSetHinge2Anchor(joint[i], a[0], a[1], a[2]);
	dJointSetHinge2Axis1(joint[i], 0, 0, 1);
	dJointSetHinge2Axis2(joint[i], 1, 0, 0);
    }
    
    /// set joint suspension
    for ( int i = 0; i <= 3; i++ )
    {
	dJointSetHinge2Param(joint[i], dParamSuspensionERP, 1.8);
	dJointSetHinge2Param(joint[i], dParamSuspensionCFM, 1.8);
    }
    
    /// lock back wheels along the steering axis
    for ( int i = 1 ; i <= 3; i++ )
    {
	dJointSetHinge2Param(joint[i], dParamLoStop, 0);
	dJointSetHinge2Param(joint[i], dParamHiStop, 0);
    }
    
    // *********************************
    // END SETTING UP AN ODE ENVIRONMENT
    
    setPlanningParameters();
}

void ODECarEnvironment::setPlanningParameters(void)
{    
    /// Fill in parameters for OMPL:
    world = carWorld;
    collisionSpaces.push_back(space);
    stateBodies.push_back(boxBody);
    stateBodies.push_back(bw[0]);
    stateBodies.push_back(bw[1]);
    stateBodies.push_back(bw[2]);
    stateBodies.push_back(bw[3]);

    stepSize = 0.05;
    maxContacts = 3;
}

