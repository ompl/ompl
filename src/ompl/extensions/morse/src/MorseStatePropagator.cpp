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

/* Author: Caleb Voss */

#include "ompl/extensions/morse/MorseStatePropagator.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/extensions/morse/MorseStateSpace.h"
#include "ompl/util/Exception.h"

ompl::control::MorseStatePropagator::MorseStatePropagator(const SpaceInformationPtr &si) : StatePropagator(si)
{
    if (base::MorseStateSpace *mss = dynamic_cast<base::MorseStateSpace *>(si->getStateSpace().get()))
        env_ = mss->getEnvironment();
    else
        throw Exception("MORSE State Space needed for MorseStatePropagator");
}

void ompl::control::MorseStatePropagator::propagate(const base::State *state, const Control *control,
                                                    double duration, base::State *result) const
{
    std::lock_guard<std::mutex> lock(env_->mutex_);

    // place the MORSE world at the start state
    si_->getStateSpace()->as<base::MorseStateSpace>()->writeState(state);

    // convert control into vector of doubles
    std::vector<double> controlVec;
    const double *conVals = control->as<RealVectorControlSpace::ControlType>()->values;
    for (unsigned int i = 0; i < env_->controlDim_; i++)
        controlVec.push_back(conVals[i]);

    // apply the controls
    env_->applyControl(controlVec);

    // propagate one step forward
    env_->worldStep(duration);

    // read the final state from the MORSE world
    si_->getStateSpace()->as<base::MorseStateSpace>()->readState(result);
}

bool ompl::control::MorseStatePropagator::canPropagateBackward() const
{
    return false;
}
