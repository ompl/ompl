/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, University of Santa Cruz Hybrid Systems Laboratory
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
 *   * Neither the name of the University of Santa Cruz nor the names of 
 *     its contributors may be used to endorse or promote products derived
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

/* Author: Beverly Xu */

#include "ompl/base/spaces/HybridStateSpace.h"

ompl::base::HybridStateSpace::HybridStateSpace(const StateSpacePtr& spaceComponent)
{
    setName("HybridStateSpace" + getName());
    addSubspace(spaceComponent, 1); // space component
    addSubspace(std::make_shared<HybridTimeStateSpace>(), 1); // time component
    lock();
}

double ompl::base::HybridStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    double deltaSpace = distanceSpace(state1, state2);

    return deltaSpace;
}

/*
 * Direction-independent distance in space
 */
double ompl::base::HybridStateSpace::distanceSpace(const ompl::base::State *state1,
                                                      const ompl::base::State *state2) const
{
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);

    return components_[0]->distance(cstate1->components[0], cstate2->components[0]);
}

/*
 * Direction-independent distance in time
 */
double ompl::base::HybridStateSpace::distanceTime(const ompl::base::State *state1,
                                                     const ompl::base::State *state2) const
{
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);

    return components_[1]->distance(cstate1->components[1], cstate2->components[1]);
}

ompl::base::StateSpacePtr ompl::base::HybridStateSpace::getSpaceComponent()
{
    return components_[0];
}

ompl::base::HybridTimeStateSpace * ompl::base::HybridStateSpace::getTimeComponent()     // change to hybrid time
{
    return components_[1]->as<HybridTimeStateSpace>();
}

bool ompl::base::HybridStateSpace::isMetricSpace() const
{
    return false;
}

double ompl::base::HybridStateSpace::getMaximumExtent() const
{
    return std::numeric_limits<double>::infinity();
}

double ompl::base::HybridStateSpace::getStateTime(const ompl::base::State *state)
{
    return state->as<CompoundState>()->as<HybridTimeStateSpace::StateType>(1)->position;
}

int ompl::base::HybridStateSpace::getStateJumps(const ompl::base::State *state)
{
    return state->as<CompoundState>()->as<HybridTimeStateSpace::StateType>(1)->jumps;
}

void ompl::base::HybridStateSpace::setStateTime(ompl::base::State *state, double position)
{
    state->as<CompoundState>()->as<HybridTimeStateSpace::StateType>(1)->position = position;
}

void ompl::base::HybridStateSpace::setStateJumps(ompl::base::State *state, int jumps)
{
    state->as<CompoundState>()->as<HybridTimeStateSpace::StateType>(1)->jumps = jumps;
}