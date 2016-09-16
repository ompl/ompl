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

/* Authors: Ioan Sucan, Caleb Voss */

#include "ompl/extensions/morse/MorseSimpleSetup.h"
#include "ompl/extensions/morse/MorseControlSpace.h"
#include "ompl/extensions/morse/MorseProjection.h"
#include "ompl/extensions/morse/MorseStatePropagator.h"
#include "ompl/extensions/morse/MorseStateValidityChecker.h"
#include "ompl/extensions/morse/MorseTerminationCondition.h"
#include "ompl/util/Console.h"

ompl::control::MorseSimpleSetup::MorseSimpleSetup(const base::MorseEnvironmentPtr &env)
  : SimpleSetup(std::make_shared<MorseControlSpace>(std::make_shared<base::MorseStateSpace>(env))), env_(env)
{
    si_->setPropagationStepSize(env_->stepSize_);
    si_->setMinMaxControlDuration(env_->minControlSteps_, env_->maxControlSteps_);
    si_->setStatePropagator(std::make_shared<MorseStatePropagator>(si_));
}

ompl::base::ScopedState<ompl::base::MorseStateSpace> ompl::control::MorseSimpleSetup::getCurrentState() const
{
    base::ScopedState<base::MorseStateSpace> current(getStateSpace());
    getStateSpace()->as<base::MorseStateSpace>()->readState(current.get());
    return current;
}

void ompl::control::MorseSimpleSetup::setCurrentState(const base::State *state)
{
    getStateSpace()->as<base::MorseStateSpace>()->writeState(state);
}

void ompl::control::MorseSimpleSetup::setCurrentState(const base::ScopedState<> &state)
{
    getStateSpace()->as<base::MorseStateSpace>()->writeState(state.get());
}

void ompl::control::MorseSimpleSetup::setup()
{
    if (!si_->getStateValidityChecker())
    {
        OMPL_INFORM("Using default state validity checker for MORSE");
        si_->setStateValidityChecker(std::make_shared<base::MorseStateValidityChecker>(si_));
    }
    base::StateSpacePtr space = si_->getStateSpace();
    if (!space->hasDefaultProjection())
    {
        OMPL_INFORM("Registering MorseProjection as default projection evaluator for MORSE");
        space->registerDefaultProjection(std::make_shared<base::MorseProjection>(space));
    }
    if (pdef_->getStartStateCount() == 0)
    {
        OMPL_INFORM("Using the initial state of MORSE as the starting state for the planner");
        pdef_->addStartState(getCurrentState());
    }
    SimpleSetup::setup();
}

ompl::base::PlannerStatus ompl::control::MorseSimpleSetup::solve()
{
    setup();
    // terminate if user exits MORSE
    return SimpleSetup::solve(base::MorseTerminationCondition(env_));
}

void ompl::control::MorseSimpleSetup::playSolutionPath() const
{
    if (haveSolutionPath())
        playPath(pdef_->getSolutionPath());
}

void ompl::control::MorseSimpleSetup::playPath(const base::PathPtr &path) const
{
    PathControl *pc = dynamic_cast<PathControl *>(path.get());
    if (pc)
    {
        unsigned int i;
        base::State *result = si_->allocState();
        for (i = 0; i < pc->getControlCount(); i++)
        {
            si_->getStatePropagator()->propagate(pc->getState(i), pc->getControl(i), pc->getControlDuration(i), result);
        }
        getStateSpace()->as<base::MorseStateSpace>()->writeState(pc->getState(i));
    }
    else
    {
        geometric::PathGeometric *pg = dynamic_cast<geometric::PathGeometric *>(path.get());
        if (!pg)
            throw Exception("Unknown type of path");
        if (pg->getStateCount() > 0)
        {
            double d = si_->getPropagationStepSize();
            getStateSpace()->as<base::MorseStateSpace>()->writeState(pg->getState(0));
            for (unsigned int i = 1; i < pg->getStateCount(); ++i)
            {
                getEnvironment()->worldStep(d);
                getStateSpace()->as<base::MorseStateSpace>()->writeState(pg->getState(i));
            }
        }
    }
}

ompl::base::PathPtr ompl::control::MorseSimpleSetup::simulateControl(const double *control, unsigned int steps) const
{
    Control *c = si_->allocControl();
    memcpy(c->as<MorseControlSpace::ControlType>()->values, control,
           sizeof(double) * getControlSpace()->getDimension());
    base::PathPtr path = simulateControl(c, steps);
    si_->freeControl(c);
    return path;
}

ompl::base::PathPtr ompl::control::MorseSimpleSetup::simulateControl(const Control *control, unsigned int steps) const
{
    auto p(std::make_shared<PathControl>(si_));

    base::State *s0 = si_->allocState();
    getStateSpace()->as<base::MorseStateSpace>()->readState(s0);
    p->getStates().push_back(s0);

    base::State *s1 = si_->allocState();
    si_->propagate(s0, control, steps, s1);
    p->getStates().push_back(s1);

    p->getControls().push_back(si_->cloneControl(control));
    p->getControlDurations().push_back(steps);
    return p;
}

ompl::base::PathPtr ompl::control::MorseSimpleSetup::simulate(unsigned int steps) const
{
    Control *c = si_->allocControl();
    si_->nullControl(c);
    base::PathPtr path = simulateControl(c, steps);
    si_->freeControl(c);
    return path;
}
