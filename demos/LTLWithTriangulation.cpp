/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

/* Author: Matt Maly */

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <vector>

#include <ompl/extensions/triangle/PropositionalTriangularDecomposition.h>
#include <ompl/control/planners/ltl/PropositionalDecomposition.h>
#include <ompl/control/planners/ltl/Automaton.h>
#include <ompl/control/planners/ltl/ProductGraph.h>
#include <ompl/control/planners/ltl/LTLPlanner.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

typedef oc::PropositionalTriangularDecomposition::Polygon Polygon;
typedef oc::PropositionalTriangularDecomposition::Vertex Vertex;

// a decomposition is only needed for SyclopRRT and SyclopEST
// use TriangularDecomp
class MyDecomposition : public oc::PropositionalTriangularDecomposition
{
public:
    MyDecomposition(const ob::RealVectorBounds& bounds)
        : oc::PropositionalTriangularDecomposition(bounds)

    {
    }

    virtual ~MyDecomposition() {
    }

    virtual void project(const ob::State* s, std::vector<double>& coord) const
    {
        coord.resize(2);
        coord[0] = s->as<ob::SE2StateSpace::StateType>()->getX();
        coord[1] = s->as<ob::SE2StateSpace::StateType>()->getY();
    }

    virtual void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const
    {
       sampler->sampleUniform(s);
       ob::SE2StateSpace::StateType* ws = s->as<ob::SE2StateSpace::StateType>();
       ws->setXY(coord[0], coord[1]);
    }

private:
    ompl::RNG rng_;
};

void addObstaclesAndPropositions(oc::PropositionalTriangularDecomposition* decomp)
{
    Polygon obstacle(4);
    obstacle.pts[0] = Vertex(0.,.9);
    obstacle.pts[1] = Vertex(1.1,.9);
    obstacle.pts[2] = Vertex(1.1,1.1);
    obstacle.pts[3] = Vertex(0.,1.1);
    decomp->addHole(obstacle);

    Polygon p0(4);
    p0.pts[0] = Vertex(.9,.3);
    p0.pts[1] = Vertex(1.1,.3);
    p0.pts[2] = Vertex(1.1,.5);
    p0.pts[3] = Vertex(.9,.5);
    decomp->addProposition(p0);

    Polygon p1(4);
    p1.pts[0] = Vertex(1.5,1.6);
    p1.pts[1] = Vertex(1.6,1.6);
    p1.pts[2] = Vertex(1.6,1.7);
    p1.pts[3] = Vertex(1.5,1.7);
    decomp->addProposition(p1);

    Polygon p2(4);
    p2.pts[0] = Vertex(.2,1.7);
    p2.pts[1] = Vertex(.3,1.7);
    p2.pts[2] = Vertex(.3,1.8);
    p2.pts[3] = Vertex(.2,1.8);
    decomp->addProposition(p2);
}

/* Returns whether a point (x,y) is within a given polygon.
   We are assuming that the polygon is a axis-aligned rectangle, with vertices stored
   in counter-clockwise order, beginning with the bottom-left vertex. */
bool polyContains(const Polygon& poly, double x, double y)
{
    return x >= poly.pts[0].x && x <= poly.pts[2].x
        && y >= poly.pts[0].y && y <= poly.pts[2].y;
}

/* Our state validity checker queries the decomposition for its obstacles,
   and checks for collisions against them.
   This is to prevent us from having to redefine the obstacles in multiple places. */
bool isStateValid(
    const oc::SpaceInformation *si,
    const oc::PropositionalTriangularDecomposition* decomp,
    const ob::State *state)
{
    if (!si->satisfiesBounds(state))
        return false;
    const ob::SE2StateSpace::StateType* se2 = state->as<ob::SE2StateSpace::StateType>();

	double x = se2->getX();
	double y = se2->getY();
    const std::vector<Polygon>& obstacles = decomp->getHoles();
    typedef std::vector<Polygon>::const_iterator ObstacleIter;
    for (ObstacleIter o = obstacles.begin(); o != obstacles.end(); ++o)
    {
        if (polyContains(*o, x, y))
            return false;
    }
	return true;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const ob::SE2StateSpace::StateType* se2 = start->as<ob::SE2StateSpace::StateType>();
    const oc::RealVectorControlSpace::ControlType* rctrl = control->as<oc::RealVectorControlSpace::ControlType>();

    double xout = se2->getX() + rctrl->values[0]*duration*cos(se2->getYaw());
    double yout = se2->getY() + rctrl->values[0]*duration*sin(se2->getYaw());
    double yawout = se2->getYaw() + rctrl->values[1];

    ob::SE2StateSpace::StateType* se2out = result->as<ob::SE2StateSpace::StateType>();
    se2out->setXY(xout, yout);
    se2out->setYaw(yawout);

    ob::SO2StateSpace::StateType* so2out = se2out->as<ob::SO2StateSpace::StateType>(1);
    ob::SO2StateSpace SO2;
    SO2.enforceBounds (so2out);
}

void planWithSimpleSetup(void)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE2StateSpace());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(2);

    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    //create triangulation that ignores obstacle and respects propositions
    MyDecomposition* ptd = new MyDecomposition(bounds);
    addObstaclesAndPropositions(ptd);
    ptd->setup();
    oc::PropositionalDecompositionPtr pd(ptd);

    //print the triangulation to stdout
    ptd->print(std::cout);

    // create a control space
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-.5);
    cbounds.setHigh(.5);

    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    oc::SimpleSetup ss(cspace);
    ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), ptd, _1));
    ss.setStatePropagator(boost::bind(&propagate, _1, _2, _3, _4));
    ss.getSpaceInformation()->setPropagationStepSize(0.025);

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.2);
    start->setY(0.2);
    start->setYaw(0.0);

    // create a goal state - this is a dummy goal state that won't get used
    ob::ScopedState<ob::SE2StateSpace> goal(start);

    ss.setStartAndGoalStates(start, goal, 0.);

    //LTL co-safety sequencing formula: visit p2,p0 in that order
    std::vector<unsigned int> sequence(2);
    sequence[0] = 2;
    sequence[1] = 0;
    oc::AutomatonPtr cosafety = oc::Automaton::SequenceAutomaton(3, sequence);

    //LTL safety avoidance formula: never visit p1
    std::vector<unsigned int> toAvoid(1);
    toAvoid[0] = 1;
    oc::AutomatonPtr safety = oc::Automaton::AvoidanceAutomaton(3, toAvoid);
    
    //construct product graph (propDecomp x A_{cosafety} x A_{safety})
    oc::ProductGraphPtr product(new oc::ProductGraph(pd, cosafety, safety));

    //LTL planner (input: state information, product automaton)
    oc::LTLPlanner* ltlPlanner = new oc::LTLPlanner(ss.getSpaceInformation(), product);

    //hand the planner to SimpleSetup
    ss.setPlanner(ob::PlannerPtr(ltlPlanner));

    // attempt to solve the problem within thirty seconds of planning time
    ob::PlannerStatus solved = ss.solve(30);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    planWithSimpleSetup();
    return 0;
}
