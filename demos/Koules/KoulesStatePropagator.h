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

#ifndef DEMOS_KOULES_STATEPROPAGATOR_
#define DEMOS_KOULES_STATEPROPAGATOR_

#include "KoulesConfig.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/StatePropagator.h>

// State propagator for KoulesSetup.
class KoulesStatePropagator : public ompl::control::StatePropagator
{
public:

    KoulesStatePropagator(const ompl::control::SpaceInformationPtr &si) :
        ompl::control::StatePropagator(si), timeStep_(integrationStepSize),
        numDimensions_(si->getStateSpace()->getDimension()),
        numKoules_((numDimensions_ - 5) / 4),
        q(numDimensions_), qdot(numDimensions_), hasCollision(numKoules_ + 1)
    {
    }

    virtual void propagate(const ompl::base::State *start, const ompl::control::Control* control,
        const double duration, ompl::base::State *result) const;

protected:

    void ode(double* u) const;

    void update(double dt) const;

    // check collision among object i and j
    // compute elastic collision response if i and j collide
    // see http://en.wikipedia.org/wiki/Elastic_collision
    bool checkCollision(unsigned int i, unsigned int j, double dt) const;

    double timeStep_;
    unsigned int numDimensions_;
    unsigned int numKoules_;
    // The next three elements are scratch space. This is normally a very BAD
    // idea, since planners can be multi-threaded. However, none of the
    // planners used here are multi-threaded, so it's safe. This way the
    // propagate function doesn't have to allocate memory upon each call.
    mutable std::vector<double> q;
    mutable std::vector<double> qdot;
    mutable std::vector<bool> hasCollision;
};

#endif

