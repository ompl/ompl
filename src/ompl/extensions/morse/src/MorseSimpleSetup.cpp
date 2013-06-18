/* MorseSimpleSetup.cpp */

#include "ompl/extensions/morse/MorseSimpleSetup.h"
#include "ompl/util/Exception.h"
#include <boost/thread.hpp>

ompl::control::MorseSimpleSetup::MorseSimpleSetup(const ControlSpacePtr &space) : SimpleSetup(space)
{
    if (!dynamic_cast<MorseControlSpace*>(space.get()))
        throw Exception("MORSE Control Space needed for MORSE Simple Setup");
    useEnvParams();
}

ompl::control::MorseSimpleSetup::MorseSimpleSetup(const base::StateSpacePtr &space) :
    SimpleSetup(ControlSpacePtr(new MorseControlSpace(space)))
{
    useEnvParams();
}

ompl::control::MorseSimpleSetup::MorseSimpleSetup(const base::MorseEnvironmentPtr &env) :
    SimpleSetup(ControlSpacePtr(new MorseControlSpace(base::StateSpacePtr(new base::MorseStateSpace(env)))))
{
    useEnvParams();
}

void ompl::control::MorseSimpleSetup::useEnvParams(void)
{
    si_->setPropagationStepSize(getStateSpace()->as<base::MorseStateSpace>()->getEnvironment()->stepSize_);
    si_->setMinMaxControlDuration(getStateSpace()->as<base::MorseStateSpace>()->getEnvironment()->minControlSteps_,
                                  getStateSpace()->as<base::MorseStateSpace>()->getEnvironment()->maxControlSteps_);
    si_->setStatePropagator(StatePropagatorPtr(new MorseStatePropagator(si_)));
}

ompl::base::ScopedState<ompl::base::MorseStateSpace> ompl::control::MorseSimpleSetup::getCurrentState(void) const
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

void ompl::control::MorseSimpleSetup::setup(void)
{
    if (!si_->getStateValidityChecker())
    {
        OMPL_INFORM("Using default state validity checker for MORSE");
        si_->setStateValidityChecker(base::StateValidityCheckerPtr(new base::MorseStateValidityChecker(si_)));
    }
    if (pdef_->getStartStateCount() == 0)
    {
        OMPL_INFORM("Using the initial state of MORSE as the starting state for the planner");
        pdef_->addStartState(getCurrentState());
    }
    SimpleSetup::setup();
}

void ompl::control::MorseSimpleSetup::playSolutionPath(double timeFactor) const
{
    if (haveSolutionPath())
        playPath(pdef_->getSolutionPath(), timeFactor);
}

void ompl::control::MorseSimpleSetup::playPath(const base::PathPtr &path, double timeFactor) const
{
    bool ctl = false;
    if (dynamic_cast<PathControl*>(path.get()))
        ctl = true;
    else
        if (!dynamic_cast<geometric::PathGeometric*>(path.get()))
            throw Exception("Unknown type of path");

    const geometric::PathGeometric &pg = ctl ?
        static_cast<PathControl*>(path.get())->asGeometric() : *static_cast<geometric::PathGeometric*>(path.get());

    if (pg.getStateCount() > 0)
    {
        OMPL_DEBUG("Playing through %u states (%0.3f seconds)", (unsigned int)pg.getStateCount(),
                   timeFactor * si_->getPropagationStepSize() * (double)(pg.getStateCount() - 1));
        time::duration d = time::seconds(timeFactor * si_->getPropagationStepSize());
        getStateSpace()->as<base::MorseStateSpace>()->writeState(pg.getState(0));
        for (unsigned int i = 1 ; i < pg.getStateCount() ; ++i)
        {
            boost::this_thread::sleep(d);
            getStateSpace()->as<base::MorseStateSpace>()->writeState(pg.getState(i));
        }
    }
}

ompl::base::PathPtr ompl::control::MorseSimpleSetup::simulateControl(const double* control, unsigned int steps) const
{
    Control *c = si_->allocControl();
    memcpy(c->as<MorseControlSpace::ControlType>()->values, control, sizeof(double) * getControlSpace()->getDimension());
    base::PathPtr path = simulateControl(c, steps);
    si_->freeControl(c);
    return path;
}

ompl::base::PathPtr ompl::control::MorseSimpleSetup::simulateControl(const Control* control, unsigned int steps) const
{
    PathControl *p = new PathControl(si_);

    base::State *s0 = si_->allocState();
    getStateSpace()->as<base::MorseStateSpace>()->readState(s0);
    p->getStates().push_back(s0);

    base::State *s1 = si_->allocState();
    si_->propagate(s0, control, steps, s1);
    p->getStates().push_back(s1);

    p->getControls().push_back(si_->cloneControl(control));
    p->getControlDurations().push_back(steps);
    return base::PathPtr(p);
}

ompl::base::PathPtr ompl::control::MorseSimpleSetup::simulate(unsigned int steps) const
{
    Control *c = si_->allocControl();
    si_->nullControl(c);
    base::PathPtr path = simulateControl(c, steps);
    si_->freeControl(c);
    return path;
}
