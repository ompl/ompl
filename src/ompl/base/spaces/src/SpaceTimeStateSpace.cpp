/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Technische Universität Berlin (TU Berlin)
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
*   * Neither the name of the TU Berlin nor the names of its
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

/* Author: Francesco Grothe */

#include "ompl/base/spaces/SpaceTimeStateSpace.h"

ompl::base::SpaceTimeStateSpace::SpaceTimeStateSpace(const StateSpacePtr& spaceComponent, double vMax,
                                                     double timeWeight) : vMax_(vMax)
{
    if (timeWeight < 0 || timeWeight > 1)
        throw ompl::Exception("Error in SpaceTimeStateSpace Construction: Time weight must be between 0 and 1");

    setName("SpaceTimeStateSpace" + getName());
    addSubspace(spaceComponent, (1 - timeWeight)); // space component
    addSubspace(std::make_shared<TimeStateSpace>(), timeWeight); // time component
    lock();
}

double ompl::base::SpaceTimeStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    double deltaSpace = distanceSpace(state1, state2);
    double deltaTime = distanceTime(state1, state2);

    if (deltaSpace / vMax_ > deltaTime + eps_) return std::numeric_limits<double>::infinity();

    return weights_[0] * deltaSpace + weights_[1] * deltaTime;

}

/*
 * Direction-independent distance in space
 */
double ompl::base::SpaceTimeStateSpace::distanceSpace(const ompl::base::State *state1,
                                                      const ompl::base::State *state2) const
{
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);

    return components_[0]->distance(cstate1->components[0], cstate2->components[0]);
}

/*
 * Direction-independent distance in time
 */
double ompl::base::SpaceTimeStateSpace::distanceTime(const ompl::base::State *state1,
                                                     const ompl::base::State *state2) const
{
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);

    return components_[1]->distance(cstate1->components[1], cstate2->components[1]);
}

void ompl::base::SpaceTimeStateSpace::setTimeBounds(double lb, double ub)
{
    as<TimeStateSpace>(1)->setBounds(lb, ub);
}

double ompl::base::SpaceTimeStateSpace::getVMax() const
{
    return vMax_;
}

void ompl::base::SpaceTimeStateSpace::setVMax(double vMax)
{
    vMax_ = vMax;
}

double ompl::base::SpaceTimeStateSpace::timeToCoverDistance(const ompl::base::State *state1,
                                                            const ompl::base::State *state2) const
{
    double deltaSpace = distanceSpace(state1, state2);
    return deltaSpace / vMax_;
}

ompl::base::StateSpacePtr ompl::base::SpaceTimeStateSpace::getSpaceComponent()
{
    return components_[0];
}

ompl::base::TimeStateSpace * ompl::base::SpaceTimeStateSpace::getTimeComponent()
{
    return components_[1]->as<TimeStateSpace>();
}

bool ompl::base::SpaceTimeStateSpace::isMetricSpace() const
{
    return false;
}

double ompl::base::SpaceTimeStateSpace::getMaximumExtent() const
{
    return std::numeric_limits<double>::infinity();
}

void ompl::base::SpaceTimeStateSpace::updateEpsilon()
{
    auto extent = getTimeComponent()->isBounded() ? getTimeComponent()->getMaximumExtent() :
                                                    getSpaceComponent()->getMaximumExtent() / vMax_;
    eps_ = std::numeric_limits<float>::epsilon() * std::pow(10, std::ceil(std::log10(extent)));
}

double ompl::base::SpaceTimeStateSpace::getStateTime(const ompl::base::State *state)
{
    return state->as<CompoundState>()->as<TimeStateSpace::StateType>(1)->position;
}
