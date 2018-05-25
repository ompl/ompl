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
#include <ompl/control/planners/ltl/LTLProblemDefinition.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

using Polygon = oc::PropositionalTriangularDecomposition::Polygon;
using Vertex = oc::PropositionalTriangularDecomposition::Vertex;

// a decomposition is only needed for SyclopRRT and SyclopEST
// use TriangularDecomp
class MyDecomposition : public oc::PropositionalTriangularDecomposition
{
public:
    MyDecomposition(const ob::RealVectorBounds& bounds)
        : oc::PropositionalTriangularDecomposition(bounds) { }
    ~MyDecomposition() override = default;

    void project(const ob::State* s, std::vector<double>& coord) const override
    {
        coord.resize(2);
        coord[0] = s->as<ob::SE2StateSpace::StateType>()->getX();
        coord[1] = s->as<ob::SE2StateSpace::StateType>()->getY();
    }

    void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
    {
       sampler->sampleUniform(s);
       auto* ws = s->as<ob::SE2StateSpace::StateType>();
       ws->setXY(coord[0], coord[1]);
    }
};

void addObstaclesAndPropositions(std::shared_ptr<oc::PropositionalTriangularDecomposition> &decomp)
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
    const std::shared_ptr<oc::PropositionalTriangularDecomposition> &decomp,
    const ob::State *state)
{
    if (!si->satisfiesBounds(state))
        return false;
    const auto* se2 = state->as<ob::SE2StateSpace::StateType>();

    double x = se2->getX();
    double y = se2->getY();
    const std::vector<Polygon>& obstacles = decomp->getHoles();
    for (const auto & obstacle : obstacles)
    {
        if (polyContains(obstacle, x, y))
            return false;
    }
    return true;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto* se2 = start->as<ob::SE2StateSpace::StateType>();
    const auto* rctrl = control->as<oc::RealVectorControlSpace::ControlType>();

    double xout = se2->getX() + rctrl->values[0]*duration*cos(se2->getYaw());
    double yout = se2->getY() + rctrl->values[0]*duration*sin(se2->getYaw());
    double yawout = se2->getYaw() + rctrl->values[1];

    auto* se2out = result->as<ob::SE2StateSpace::StateType>();
    se2out->setXY(xout, yout);
    se2out->setYaw(yawout);

    auto* so2out = se2out->as<ob::SO2StateSpace::StateType>(1);
    ob::SO2StateSpace SO2;
    SO2.enforceBounds (so2out);
}

void plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(2);

    space->setBounds(bounds);

    // create triangulation that ignores obstacle and respects propositions
    std::shared_ptr<oc::PropositionalTriangularDecomposition> ptd = std::make_shared<MyDecomposition>(bounds);
    // helper method that adds an obstacle, as well as three propositions p0,p1,p2
    addObstaclesAndPropositions(ptd);
    ptd->setup();

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-.5);
    cbounds.setHigh(.5);

    cspace->setBounds(cbounds);

    oc::SpaceInformationPtr si(std::make_shared<oc::SpaceInformation>(space, cspace));
    si->setStateValidityChecker(
        [&si, ptd](const ob::State *state)
        {
            return isStateValid(si.get(), ptd, state);
        });
    si->setStatePropagator(propagate);
    si->setPropagationStepSize(0.025);

    //LTL co-safety sequencing formula: visit p2,p0 in that order
#if OMPL_HAVE_SPOT
    // This shows off the capability to construct an automaton from LTL-cosafe formula using Spot
    auto cosafety = std::make_shared<oc::Automaton>(3, "! p0 U ((p2 & !p0) & XF p0)");
#else
    auto cosafety = oc::Automaton::SequenceAutomaton(3, {2, 0});
#endif
    //LTL safety avoidance formula: never visit p1
#if OMPL_HAVE_SPOT
    // This shows off the capability to construct an automaton from LTL-safe formula using Spot
    auto safety = std::make_shared<oc::Automaton>(3, "G ! p1", false);
#else
    auto safety = oc::Automaton::AvoidanceAutomaton(3, {1});
#endif

    // construct product graph (propDecomp x A_{cosafety} x A_{safety})
    auto product(std::make_shared<oc::ProductGraph>(ptd, cosafety, safety));

    // LTLSpaceInformation creates a hybrid space of robot state space x product graph.
    // It takes the validity checker from SpaceInformation and expands it to one that also
    // rejects any hybrid state containing rejecting automaton states.
    // It takes the state propagator from SpaceInformation and expands it to one that
    // follows continuous propagation with setting the next decomposition region
    // and automaton states accordingly.
    //
    // The robot state space, given by SpaceInformation, is referred to as the "lower space".
    auto ltlsi(std::make_shared<oc::LTLSpaceInformation>(si, product));

    // LTLProblemDefinition creates a goal in hybrid space, corresponding to any
    // state in which both automata are accepting
    auto pdef(std::make_shared<oc::LTLProblemDefinition>(ltlsi));

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.2);
    start->setY(0.2);
    start->setYaw(0.0);

    // addLowerStartState accepts a state in lower space, expands it to its
    // corresponding hybrid state (decomposition region containing the state, and
    // starting states in both automata), and adds that as an official start state.
    pdef->addLowerStartState(start.get());

    //LTL planner (input: LTL space information, product automaton)
    oc::LTLPlanner ltlPlanner(ltlsi, product);
    ltlPlanner.setProblemDefinition(pdef);

    // attempt to solve the problem within thirty seconds of planning time
    // considering the above cosafety/safety automata, a solution path is any
    // path that visits p2 followed by p0 while avoiding obstacles and avoiding p1.
    ob::PlannerStatus solved = ltlPlanner.ob::Planner::solve(30.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // The path returned by LTLProblemDefinition is through hybrid space.
        // getLowerSolutionPath() projects it down into the original robot state space
        // that we handed to LTLSpaceInformation.
        static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    plan();
    return 0;
}
