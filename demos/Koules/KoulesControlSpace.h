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

/* Author: Beck Chen, Mark Moll */

#ifndef DEMOS_KOULES_CONTROLSPACE_
#define DEMOS_KOULES_CONTROLSPACE_

#include <ompl/control/spaces/RealVectorControlSpace.h>

// Control sampler for KoulesControlSpace
class KoulesControlSampler : public ompl::control::ControlSampler
{
public:
    KoulesControlSampler(const ompl::control::ControlSpace *space) : ompl::control::ControlSampler(space)
    {
    }
    // Sample random velocity with magnitude between vmin and vmax and
    // orientation uniformly random over [0, 2*pi].
    // (This method is not actually ever called.)
    virtual void sample(ompl::control::Control *control);
    // sample random velocity with magnitude between vmin and vmax and
    // direction given by the normalized vector from the current position
    // in state and a random point in the workspace
    virtual void sample(ompl::control::Control *control, const ompl::base::State *state);
    virtual void sampleNext(ompl::control::Control *control, const ompl::control::Control * /* previous */,
        const ompl::base::State *state)
    {
        sample(control, state);
    }
    virtual void steer(ompl::control::Control *control, const ompl::base::State *state, double x, double y);

protected:
    ompl::RNG rng_;
};

class KoulesControlSpace : public ompl::control::RealVectorControlSpace
{
public:
    KoulesControlSpace(unsigned int numKoules);

    virtual ompl::control::ControlSamplerPtr allocDefaultControlSampler() const
    {
        return std::make_shared<KoulesControlSampler>(this);
    }
};



#endif
