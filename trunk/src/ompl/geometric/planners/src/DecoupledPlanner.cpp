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

#include "ompl/geometric/planners/DecoupledPlanner.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/GoalStates.h"
#include <boost/lexical_cast.hpp>
#include <limits>

ompl::geometric::DecoupledPlanner::DecoupledPlanner(const base::SpaceInformationPtr &si) : base::Planner(si, "DecoupledPlanner"), currentStartState_(si)
{
    type_ = base::PLAN_TO_GOAL_SAMPLEABLE_REGION;
    if (!dynamic_cast<base::CompoundStateManifold*>(si->getStateManifold().get()))
        throw Exception("A compound state manifold is needed for decoupled planning");
    startStateVersion_ = 0;
}

/// @cond IGNORE
namespace ompl
{
    struct ManifoldIndex
    {
        ManifoldIndex(void) : singleIndex(-1)
        {
        }
        
        int                       singleIndex;
        std::vector<unsigned int> multiIndex;
    };

    // This class maintains a "full state" (a state from a compound manifold)
    // and rewires it as needed to include the value of a "partial state"
    // This allows looking at partial states as full states without any memory copying
    class PartialToFullStateConverter
    {
    public:
        
        PartialToFullStateConverter(const base::StateManifoldPtr &fullStateManifold) : fullStateManifold_(fullStateManifold)
        {
            totalComponents_ = static_cast<base::CompoundStateManifold*>(fullStateManifold_.get())->getSubManifoldCount();
            backup_ = new base::State*[totalComponents_];
            fullState_ = static_cast<base::CompoundState*>(fullStateManifold_->allocState());
            memcpy(backup_, fullState_->components, sizeof(base::State*) * totalComponents_);
        }
        
        ~PartialToFullStateConverter(void)
        {
            memcpy(fullState_->components, backup_, sizeof(base::State*) * totalComponents_);
            delete[] backup_;
            fullStateManifold_->freeState(fullState_);
        }
        
        void useFullState(const ManifoldIndex &mi, const base::State *state)
        {
            mi_ = mi;
            memcpy(fullState_->components, backup_, sizeof(base::State*) * totalComponents_);
            fullStateManifold_->copyState(fullState_, state);
        }
        
        void applyPartialState(const base::State *state)
        {
            if (mi_.singleIndex >= 0)
                fullState_->components[mi_.singleIndex] = const_cast<base::State*>(state);
            else
                for (unsigned int i = 0 ; i < mi_.multiIndex.size() ; ++i)
                    fullState_->components[mi_.multiIndex[i]] = static_cast<const base::CompoundState*>(state)->components[i];
        }
        
        const base::State* getFullState(void) const
        {
            return fullState_;
        }
        
    private:
        
        base::StateManifoldPtr fullStateManifold_;
        unsigned int           totalComponents_;
        ManifoldIndex          mi_;
        base::CompoundState   *fullState_;
        base::State          **backup_;
    };
    
    
    // Perform state validation for a partial state using a full
    // state to fill in the blanks and the state validator for the full state.
    // This class automatically updates the full state based on a version
    class DecoupledStateValidityChecker : public base::StateValidityChecker
    {
    public:
        DecoupledStateValidityChecker(const base::SpaceInformationPtr &partialSI, const base::SpaceInformationPtr &fullSI, const ManifoldIndex &mi,
                                      const base::ScopedState<> *fullStartStatePtr, const unsigned int *fullStartStateVersion) :
            base::StateValidityChecker(partialSI), fullStateChecker_(fullSI->getStateValidityChecker()), p2f_(fullSI->getStateManifold()), mi_(mi),
            fullStartStatePtr_(fullStartStatePtr), availableFullStartStateVersion_(fullStartStateVersion), maintainedFullStartStateVersion_(0)
        {
        }
        
        virtual ~DecoupledStateValidityChecker(void)
        {
        }
        
        virtual bool isValid(const base::State *state) const
        {
            if (maintainedFullStartStateVersion_ != *availableFullStartStateVersion_)
            {
                maintainedFullStartStateVersion_ = *availableFullStartStateVersion_;
                p2f_.useFullState(mi_, fullStartStatePtr_->get());
            }
            p2f_.applyPartialState(state);
            return fullStateChecker_->isValid(p2f_.getFullState());
        }
        
    protected:
        
        base::StateValidityCheckerPtr       fullStateChecker_;
        mutable PartialToFullStateConverter p2f_;
        ManifoldIndex                       mi_;
        const base::ScopedState<>          *fullStartStatePtr_;
        const unsigned int                 *availableFullStartStateVersion_;
        mutable unsigned int                maintainedFullStartStateVersion_;
    };

    // Extract a description of how to rewire a full state to include a partial state, given the partial & full manifolds 
    ManifoldIndex getManifoldIndex(const base::StateManifoldPtr &manifold, const base::StateManifoldPtr &fullManifold)
    {
        base::CompoundStateManifold *cm = static_cast<base::CompoundStateManifold*>(fullManifold.get());
        ManifoldIndex mi;
        try
        {
            unsigned int index = cm->getSubManifoldIndex(manifold->getName());
            mi.singleIndex = index;
        }
        catch(Exception&)
        {
            base::CompoundStateManifold *c = dynamic_cast<base::CompoundStateManifold*>(manifold.get());
            if (c)
            {
                for (unsigned int i = 0 ; i < c->getSubManifoldCount() ; ++i)
                {
                    try
                    {
                        unsigned int mindex = cm->getSubManifoldIndex(c->getSubManifold(i)->getName());
                        mi.multiIndex.push_back(mindex);
                    }
                    catch(Exception&)
                    {
                        throw Exception("Manifold '" + c->getSubManifold(i)->getName() + "' is not a component of manifold '" + cm->getName() + "'");
                    }
                }
            }
            else
                throw Exception("Manifold '" + manifold->getName() + "' is not a component of manifold '" + cm->getName() + "'");
        }
        return mi;
    }
}
/// @endcond

bool ompl::geometric::DecoupledPlanner::processInputStates(const base::PlannerTerminationCondition &ptc)
{
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
    
    if (!goal)
    {
	msg_.error("Unknown type of goal (or goal undefined)");
	return false;
    }
    
    unsigned int ss0 = startStates_.size();
    unsigned int gs0 = goalStates_.size();
    
    const base::StateManifoldPtr &fullM0 = components_[0].planner->getSpaceInformation()->getStateManifold();
    if (startStates_.empty())
    {
        // If we have no start states, get all the states and group
        // them based on the state values that the first component is
        // not planning for. Pick as starting states the largest group.
        
        std::vector< base::ScopedState<> > sts;
        while (const base::State *st = pis_.nextStart())
            sts.push_back(base::ScopedState<>(si_->getStateManifold(), st));
        
        if (sts.empty())
        {
            msg_.error("Motion planning start tree could not be initialized!");
            return false;
        }
        
        base::StateManifoldPtr rest0 = si_->getStateManifold() - fullM0;
        std::vector < std::pair < base::ScopedState<>, std::vector<base::ScopedState<> > > > groups;
        
        for (unsigned int i = 0 ; i < sts.size() ; ++i)
        {
            bool found = false;
            base::ScopedState<> r = sts[i][rest0];
            for (unsigned int j = 0 ; j < groups.size() ; ++j)
                if (groups[j].first == r)
                {
                    groups[j].second.push_back(sts[i]);
                    found = true;
                    break;
                }
            if (!found)
            {
                std::vector< base::ScopedState<> > e;
                e.push_back(sts[i]);
                groups.push_back(std::make_pair(r, e));
            }
        }
        
        unsigned int index = 0;
        unsigned int maxG = groups[0].second.size();
        for (unsigned int i = 1 ; i < groups.size() ; ++i)
            if (groups[i].second.size() > maxG)
            {
                maxG = groups[i].second.size();
                index = i;
            }
        for (unsigned int i = 0 ; i < groups.size() ; ++i)
            if (i != index)
                for (unsigned int j = 0 ; j < groups[i].second.size() ; ++j)
                    rejectedStartStates_.push_back(groups[i].second[j]);
        for (unsigned int j = 0 ; j < groups[index].second.size() ; ++j)
            startStates_.push_back(groups[index].second[j]);
    }
    else
    {
        // if we have a new start state, see if it matches the components we do not plan for, in the first component
        // if it does, it can be considered a new start state. if not, it is a rejected start state
        base::StateManifoldPtr rest0;
        base::ScopedStatePtr   restS0;
        while (const base::State *st = pis_.nextStart())
        {
            if (!rest0)
                rest0 = si_->getStateManifold() - fullM0;
            if (!restS0)
            {
                restS0.reset(new base::ScopedState<>(rest0));
                *restS0 = startStates_[0][rest0];
            }
            
            base::ScopedState<> add(si_->getStateManifold(), st);
            if (*restS0 == add[rest0])
                startStates_.push_back(add);
            else
                rejectedStartStates_.push_back(add);
        }
    }
    
    if (!goal->canSample())
    {
	msg_.error("Insufficient states in sampleable goal region");
	return false;
    }
    
    // how many more draws are we going to make
    unsigned int maxGoals = goal->maxSampleCount() < std::numeric_limits<unsigned int>::max() ? (goal->maxSampleCount() - gs0) : startStates_.size() * 2;
    for (unsigned int i = 0 ; i < maxGoals ; ++i)
    {
        const base::State *st = goalStates_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st)
            goalStates_.push_back(base::ScopedState<>(si_->getStateManifold(), st));
        else
            break;
    }

    if (goalStates_.empty())
    {
        msg_.error("Motion planning goal tree could not be initialized!");
        return false;
    }
    
    currentStartState_ = startStates_[0];
    startStateVersion_++;
    
    for (unsigned int i = ss0 ; i < startStates_.size() ; ++i)
        components_[0].pdef->addStartState(startStates_[i][fullM0]);

    for (unsigned int i = gs0 ; i < goalStates_.size() ; ++i)
        static_cast<base::GoalStates*>(components_[0].pdef->getGoal().get())->addState(goalStates_[i][fullM0]);
    
    return true;
}

bool ompl::geometric::DecoupledPlanner::solve(const base::PlannerTerminationCondition &ptc)
{
    pis_.checkValidity();
    
    if (!processInputStates(ptc))
        return false;
    
    base::StateManifoldPtr plannedFor(new base::CompoundStateManifold());
    for (unsigned int i = 0 ; i < components_.size() ; ++i)
    {
        if (ptc())
            return false;
        
        if (!components_[i].pdef->getGoal()->getSolutionPath())
        {
            msg_.debug("Begin planning for component %u using %d start states and %d goal states", i,
                       (int)components_[i].pdef->getStartStateCount(), (int)static_cast<base::GoalStates*>(components_[i].pdef->getGoal().get())->maxSampleCount());
            components_[i].planner->setup();
            if (components_[i].planner->solve(ptc))
            {
                if (components_[i].pdef->getGoal()->isApproximate())
                {
                    components_[i].planner->clear();
                    components_[i].pdef->getGoal()->clearSolutionPath();
                }
                else
                {
                    const base::StateManifoldPtr &partialMi = components_[i].planner->getSpaceInformation()->getStateManifold();
                    plannedFor = plannedFor + partialMi;
                    PathGeometric *sol = dynamic_cast<PathGeometric*>(components_[i].pdef->getGoal()->getSolutionPath().get());
                    if (!sol)
                        throw Exception("Internal error for decoupled planner. Please contact the developers");
                    currentStartState_ << base::ScopedState<>(partialMi, sol->states.back());
                    if (i + 1 < components_.size())
                    {
                        const base::StateManifoldPtr &partialMi1 = components_[i + 1].planner->getSpaceInformation()->getStateManifold();
                        components_[i + 1].pdef->addStartState(currentStartState_[partialMi1]);
                        base::GoalStates *gs = static_cast<base::GoalStates*>(components_[i + 1].pdef->getGoal().get());
                        base::ScopedState<> temp = currentStartState_[plannedFor];
                        for (unsigned int g = 0 ; g < goalStates_.size() ; ++g)
                            if (goalStates_[g][plannedFor] == temp)
                                gs->addState(goalStates_[g][partialMi1]);
                    }
                    startStateVersion_++;
                    msg_.debug("Solution found for component %u", i);
                }
            }
        }
    }
    
    if (components_.back().pdef->getGoal()->getSolutionPath())
    {
        PathGeometric *fullSol = new PathGeometric(si_);
        currentStartState_ = startStates_[0];
        for (unsigned int i = 0 ; i < components_.size() ; ++i)
        {
            PathGeometric *sol = static_cast<PathGeometric*>(components_[i].pdef->getGoal()->getSolutionPath().get());
            for (unsigned int j = 0 ; j < sol->states.size() ; ++j)
            {
                currentStartState_ << base::ScopedState<>(components_[i].planner->getSpaceInformation()->getStateManifold(), sol->states[j]);
                if ((j + 1 < sol->states.size()) || (i + 1 == components_.size()))
                    fullSol->states.push_back(si_->cloneState(currentStartState_.get()));
            }
        }
        pdef_->getGoal()->setSolutionPath(base::PathPtr(fullSol));
        return pdef_->getGoal()->isAchieved();
    }
    else
        return false;
}

/// @cond IGNORE
namespace ompl
{
    struct Group
    {
        Group(void) : d(0)
        {
        }
        
        std::vector<base::StateManifoldPtr> m;
        std::vector<double>                 w;
        unsigned int                        d;
    };
}
/// @endcond

void ompl::geometric::DecoupledPlanner::clearComponents(void)
{
    components_.clear();
}

void ompl::geometric::DecoupledPlanner::setComponentCount(unsigned int components)
{
    clearComponents();
    
    if (components < 1)
        return;
        
    base::CompoundStateManifold *cm = static_cast<base::CompoundStateManifold*>(si_->getStateManifold().get());
    std::vector<Group> g(components);
    
    for (unsigned int i = 0 ; i < cm->getSubManifoldCount() ; ++i)
    {
        unsigned int p = 0;
        for (unsigned int j = 1 ; j < g.size() ; ++j)
            if (g[j].d < g[p].d)
                p = j;
        g[p].m.push_back(cm->getSubManifold(i));
        g[p].w.push_back(cm->getSubManifoldWeight(i));
        g[p].d += cm->getSubManifold(i)->getDimension();
    }
    
    for (unsigned int i = 0 ; i < g.size() ; ++i)
    {
        if (g[i].m.empty())
            continue;
        if (g[i].m.size() == 1)
            addComponent(g[i].m[0]);
        else
            addComponent(base::StateManifoldPtr(new base::CompoundStateManifold(g[i].m, g[i].w)));
    }
}

void ompl::geometric::DecoupledPlanner::clear(void)
{
    Planner::clear();
    
    for (unsigned int i = 0 ; i < components_.size() ; ++i)
    {
        components_[i].planner->clear();
        components_[i].pdef->getGoal()->clearSolutionPath();
    }
}

void ompl::geometric::DecoupledPlanner::setup(void)
{
    Planner::setup();
    currentStartState_.random();
    if (components_.empty())
    {
        msg_.inform("Automatically decomposing state manifold ...");        
        setComponentCount(2);
        msg_.inform("Using %d components", (int)components_.size());
    }
    
    // make sure the components we have fully cover the compound manifold, disjointly
    base::CompoundStateManifold *cm = static_cast<base::CompoundStateManifold*>(si_->getStateManifold().get());
    unsigned int td = 0;
    for (unsigned int i = 0 ; i < components_.size() ; ++i)
    {
        base::StateManifold *s = components_[i].planner->getSpaceInformation()->getStateManifold().get();
        if (!cm->hasSubManifold(s->getName()))
        {
            base::CompoundStateManifold *c = dynamic_cast<base::CompoundStateManifold*>(s);
            if (!c)
                throw Exception("Manifold component '" + s->getName() + "' is not part of the full manifold '" + cm->getName() + "'");
            for (unsigned int j = 0 ; j < c->getSubManifoldCount() ; ++j)
                if (!cm->hasSubManifold(c->getSubManifold(j)->getName()))
                    throw Exception("Manifold component '" + c->getSubManifold(j)->getName() + "' is not part of the full manifold '" + cm->getName() + "'");
        }
        td += s->getDimension();
    }
    if (td < cm->getDimension())
        throw Exception("Only " + boost::lexical_cast<std::string>(td) + " of the " + boost::lexical_cast<std::string>(cm->getDimension()) +
                        " dimensions of manifold '" + cm->getName() + "' are planned for. More components need to be added.");
    if (td > cm->getDimension())
        throw Exception("Some of the components for manifold '" + cm->getName() + "' are planned for multiple times. Revise the added components.");
    
    clear();
}

void ompl::geometric::DecoupledPlanner::addComponent(const base::PlannerPtr &planner, const base::ProblemDefinitionPtr &pdef)
{
    planner->setProblemDefinition(pdef);
    Component c = { planner, pdef };
    components_.push_back(c);
}

void ompl::geometric::DecoupledPlanner::addComponent(const base::PlannerPtr &planner)
{
    base::ProblemDefinitionPtr pdef(new base::ProblemDefinition(planner->getSpaceInformation()));
    pdef->setGoal(base::GoalPtr(new base::GoalStates(planner->getSpaceInformation())));
    addComponent(planner, pdef);
}

void ompl::geometric::DecoupledPlanner::addComponent(const base::StateManifoldPtr &manifold, const base::PlannerAllocator &pa)
{
    ManifoldIndex mi = getManifoldIndex(manifold, si_->getStateManifold());
    base::SpaceInformationPtr si(new base::SpaceInformation(manifold));
    
    si->setStateValidityChecker(base::StateValidityCheckerPtr(new DecoupledStateValidityChecker(si, si_, mi, &currentStartState_, &startStateVersion_)));
    base::ProblemDefinitionPtr pdef(new base::ProblemDefinition(si));
    pdef->setGoal(base::GoalPtr(new base::GoalStates(si)));
    base::PlannerPtr planner;
    if (pa)
        planner = pa(si);
    else
        planner = getDefaultPlanner(pdef->getGoal());
    
    addComponent(planner, pdef);
}

void ompl::geometric::DecoupledPlanner::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
}
