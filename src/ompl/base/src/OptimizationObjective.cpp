/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Luis G. Torres */

#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/base/Goals/GoalRegion.h"

ompl::base::Cost ompl::base::OptimizationObjective::getCost(const Path &path) const
{
    // Cast path down to a PathGeometric
    const geometric::PathGeometric *pathGeom = dynamic_cast<const geometric::PathGeometric*>(&path);

    // Give up if this isn't a PathGeometric or if the path is empty.
    if (!pathGeom)
    {
	OMPL_ERROR("Could not cast Path* to PathGeometric* in ompl::base::OptimizationObjective::getCost()");
        return this->identityCost();
    }
    else
    {
        std::size_t numStates = pathGeom->getStateCount();
	if (numStates == 0)
        {
	    OMPL_ERROR("Cannot compute cost of an empty path.");
            return this->identityCost();
        }
	else
	{
	    // Compute path cost by accumulating the cost along the path
            Cost cost(this->identityCost());
	    for (std::size_t i = 1; i < numStates; ++i)
	    {
		const State *s1 = pathGeom->getState(i-1);
		const State *s2 = pathGeom->getState(i);
                cost = this->combineCosts(cost, this->motionCost(s1, s2));
	    }

            return cost;
	}
    }
}

ompl::base::Cost ompl::base::OptimizationObjective::averageStateCost(unsigned int numStates) const
{
    StateSamplerPtr ss = si_->allocStateSampler();
    State *state = si_->allocState();
    Cost totalCost(this->identityCost());

    for (unsigned int i = 0 ; i < numStates ; ++i)
    {
        ss->sampleUniform(state);
        this->combineCosts(totalCost, this->stateCost(state));
    }

    si_->freeState(state);

    return Cost(totalCost.v / (double)numStates);
}

const ompl::base::SpaceInformationPtr& 
ompl::base::OptimizationObjective::getSpaceInformation(void) const
{
    return si_;
}

// \TODO Make this more efficient. Right now it's easy to understand,
// but it computes state costs twice.
ompl::base::Cost ompl::base::StateCostIntegralObjective::motionCost(const State *s1, 
                                                                    const State *s2) const
{
    if (interpolateMotionCost_)
    {
        Cost totalCost = this->identityCost();

        // \TODO shouldn't we be able to use some method in SpaceInformation
        // for this instead?
        int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
  
        State *test1 = si_->cloneState(s1);
        if (nd > 1)
        {
            State *test2 = si_->allocState();
            for (int j = 1; j < nd; ++j)
            {
                si_->getStateSpace()->interpolate(s1, s2, (double) j / (double) nd, test2);
                totalCost = this->combineCosts(totalCost, this->trapezoid(test1, test2));
                si_->copyState(test1, test2);
            }
            si_->freeState(test2);
        }

        // Lastly, add s2
        totalCost = this->combineCosts(totalCost, this->trapezoid(test1, s2));

        si_->freeState(test1);

        return totalCost;
    }
    else
        return this->trapezoid(s1, s2);
}

void ompl::base::StateCostIntegralObjective::enableMotionCostInterpolation(void)
{
    interpolateMotionCost_ = true;
}

void ompl::base::StateCostIntegralObjective::disableMotionCostInterpolation(void)
{
    interpolateMotionCost_ = false;
}

bool ompl::base::StateCostIntegralObjective::isMotionCostInterpolationEnabled(void) const
{
    return interpolateMotionCost_;
}

ompl::base::Cost ompl::base::StateCostIntegralObjective::trapezoid(const State* s1, const State* s2) const
{
    return Cost(0.5 * si_->distance(s1, s2) * (this->stateCost(s1).v + this->stateCost(s2).v));
}

double ompl::base::MechanicalWorkOptimizationObjective::getPathLengthWeight(void) const
{
    return pathLengthWeight_;
}

void ompl::base::MechanicalWorkOptimizationObjective::setPathLengthWeight(double weight)
{
    pathLengthWeight_ = weight;
}

ompl::base::Cost ompl::base::MechanicalWorkOptimizationObjective::motionCost(const State *s1, 
                                                                             const State *s2) const
{
    // Only accrue positive changes in cost
    double positiveCostAccrued = std::max(stateCost(s2).v - stateCost(s1).v, 0.0);
    return Cost(positiveCostAccrued + pathLengthWeight_*si_->distance(s1,s2));
}

ompl::base::Cost ompl::base::MinimaxObjective::motionCost(const State *s1, const State *s2) const
{
    Cost worstCost = this->identityCost();

    // \TODO shouldn't we be able to use some method in SpaceInformation
    // for this instead?
    int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
  
    if (nd > 1)
    {
        State *test = si_->allocState();
        for (int j = 1; j < nd; ++j)
        {
            si_->getStateSpace()->interpolate(s1, s2, (double) j / (double) nd, test);
            Cost testStateCost = this->stateCost(test);
            if (this->isCostBetterThan(worstCost, testStateCost))
                worstCost = testStateCost;
        }
        si_->freeState(test);
    }

    // Lastly, check s2
    Cost lastCost = this->stateCost(s2);
    if (this->isCostBetterThan(worstCost, lastCost))
        worstCost = lastCost; 

    return worstCost;
}

ompl::base::Cost ompl::base::MinimaxObjective::combineCosts(Cost c1, Cost c2) const
{
    if (this->isCostBetterThan(c1, c2))
        return c2;
    else
        return c1;
}

ompl::base::Cost ompl::base::MaximizeMinClearanceObjective::stateCost(const State* s) const
{
    return Cost(si_->getStateValidityChecker()->clearance(s));
}

bool ompl::base::MaximizeMinClearanceObjective::isCostBetterThan(Cost c1, Cost c2) const
{
    return c1.v > c2.v;
}

ompl::base::Cost ompl::base::MaximizeMinClearanceObjective::identityCost(void) const
{
    return Cost(std::numeric_limits<double>::infinity());
}

ompl::base::Cost ompl::base::MaximizeMinClearanceObjective::infiniteCost(void) const
{
    return Cost(-std::numeric_limits<double>::infinity());
}

ompl::base::Cost 
ompl::base::GoalRegionDistanceCostToGo::operator()(const State* state, const Goal* goal) const
{
    const GoalRegion* goalRegion = goal->as<GoalRegion>();
    return Cost(std::max(goalRegion->distanceGoal(state) - goalRegion->getThreshold(),
                         0.0));
}

void ompl::base::MultiOptimizationObjective::addObjective(const OptimizationObjectivePtr& objective,
                                                          double weight)
{
    if (locked_)
    {    
        throw Exception("This optimization objective is locked. No further objectives can be added.");
    }
    else
        components_.push_back(Component(objective, weight));
}

std::size_t ompl::base::MultiOptimizationObjective::getObjectiveCount(void) const
{
    return components_.size();
}

const ompl::base::OptimizationObjectivePtr& 
ompl::base::MultiOptimizationObjective::getObjective(unsigned int idx) const
{
    if (components_.size() > idx)
        return components_[idx].objective;
    else
        throw Exception("Objective index does not exist.");
}

double ompl::base::MultiOptimizationObjective::getObjectiveWeight(unsigned int idx) const
{
    if (components_.size() > idx)
        return components_[idx].weight;
    else
        throw Exception("Objective index does not exist.");
}

void ompl::base::MultiOptimizationObjective::setObjectiveWeight(unsigned int idx, 
                                                                double weight)
{
    if (components_.size() > idx)
        components_[idx].weight = weight;
    else
        throw Exception("Objecitve index does not exist.");
}

void ompl::base::MultiOptimizationObjective::lock(void)
{
    locked_ = true;
}

bool ompl::base::MultiOptimizationObjective::isLocked(void) const
{
    return locked_;
}

ompl::base::Cost ompl::base::MultiOptimizationObjective::stateCost(const State* s) const
{
    Cost c = this->identityCost();
    for (std::vector<Component>::const_iterator comp = components_.begin();
         comp != components_.end();
         ++comp)
    {
        c.v += comp->weight*(comp->objective->stateCost(s).v);
    }
    
    return c;
}

ompl::base::Cost ompl::base::MultiOptimizationObjective::motionCost(const State* s1,
                                                                    const State* s2) const
{
    Cost c = this->identityCost();
     for (std::vector<Component>::const_iterator comp = components_.begin();
         comp != components_.end();
         ++comp)
     {
         c.v += comp->weight*(comp->objective->motionCost(s1, s2).v);
     }
     
     return c;
}

ompl::base::OptimizationObjectivePtr ompl::base::operator+(const OptimizationObjectivePtr &a,
                                                           const OptimizationObjectivePtr &b)
{
    std::vector<MultiOptimizationObjective::Component> components;

    if (a)
    {
        if (MultiOptimizationObjective* mult = dynamic_cast<MultiOptimizationObjective*>(a.get()))
        {
            for (std::size_t i = 0; i < mult->getObjectiveCount(); ++i)
            {
                components.push_back(MultiOptimizationObjective::
                                     Component(mult->getObjective(i),
                                               mult->getObjectiveWeight(i)));
            }
        }
        else
            components.push_back(MultiOptimizationObjective::Component(a, 1.0));            
    }
    
    if (b)
    {
        if (MultiOptimizationObjective* mult = dynamic_cast<MultiOptimizationObjective*>(b.get()))
        {
            for (std::size_t i = 0; i < mult->getObjectiveCount(); ++i)
            {
                components.push_back(MultiOptimizationObjective::
                                     Component(mult->getObjective(i),
                                               mult->getObjectiveWeight(i)));
            }
        }
        else
            components.push_back(MultiOptimizationObjective::Component(b, 1.0));
    }

    MultiOptimizationObjective* multObj = new MultiOptimizationObjective(a->getSpaceInformation());

    for (std::vector<MultiOptimizationObjective::Component>::const_iterator comp = components.begin();
         comp != components.end();
         ++comp)
    {
        multObj->addObjective(comp->objective, comp->weight);
    }

    return OptimizationObjectivePtr(multObj);
}

ompl::base::OptimizationObjectivePtr ompl::base::operator*(double weight,
                                                           const OptimizationObjectivePtr &a)
{
    std::vector<MultiOptimizationObjective::Component> components;

    if (a)
    {
        if (MultiOptimizationObjective* mult = dynamic_cast<MultiOptimizationObjective*>(a.get()))
        {
            for (std::size_t i = 0; i < mult->getObjectiveCount(); ++i)
            {
                components.push_back(MultiOptimizationObjective
                                     ::Component(mult->getObjective(i),
                                                 weight * mult->getObjectiveWeight(i)));
            }
        }
        else
            components.push_back(MultiOptimizationObjective::Component(a, weight));
    }

    MultiOptimizationObjective* multObj = new MultiOptimizationObjective(a->getSpaceInformation());

    for (std::vector<MultiOptimizationObjective::Component>::const_iterator comp = components.begin();
         comp != components.end();
         ++comp)
    {
        multObj->addObjective(comp->objective, comp->weight);
    }

    return OptimizationObjectivePtr(multObj);
}

ompl::base::OptimizationObjectivePtr ompl::base::operator*(const OptimizationObjectivePtr &a,
                                                           double weight)
{
    return weight * a;
}
