/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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

/* Author: Zachary Kingston */

#include "ompl/base/spaces/ProjectedStateSpace.h"

#include <eigen3/Eigen/Core>

/// ProjectedStateSpace

/// Public

void ompl::base::ProjectedStateSpace::checkSpace(const SpaceInformation *si)
{
    if (dynamic_cast<ProjectedStateSpace *>(si->getStateSpace().get()) == nullptr)
        throw ompl::Exception("ompl::base::ProjectedStateSpace(): "
                              "si needs to use an ProjectedStateSpace!");
}

bool ompl::base::ProjectedStateSpace::traverseManifold(const State *from, const State *to, const bool interpolate,
                                                       std::vector<State *> *stateList) const
{
    // We can move along the manifold if we were never there in the first place
    if (!constraint_->isSatisfied(from))
        return false;

    // Save a copy of the from state.
    if (stateList != nullptr)
    {
        stateList->clear();
        stateList->push_back(cloneState(from));
    }

    // No need to traverse the manifold if we are already there
    if (validSegmentCount(from, to) == 0)
        return true;

    const StateValidityCheckerPtr &svc = si_->getStateValidityChecker();
    double dist = distance(from, to);

    State *previous = cloneState(from);
    State *scratch = allocState();

    const double tolerance = delta_ + std::numeric_limits<double>::epsilon();
    do
    {
        // Compute the parameterization for interpolation
        RealVectorStateSpace::interpolate(previous, to, delta_ / dist, scratch);

        // Project new state onto constraint manifold
        const bool onManifold = constraint_->project(scratch);
        const bool valid = interpolate || svc->isValid(scratch);
        const bool deviated = distance(previous, scratch) > 2.0 * delta_;
        if (!onManifold || !valid || deviated)
            break;

        // Check if we are no closer than before
        const double newDist = distance(scratch, to);
        if (newDist >= dist)
            break;

        dist = newDist;
        copyState(previous, scratch);

        // Store the new state
        if (stateList != nullptr)
            stateList->push_back(cloneState(scratch));

    } while (dist > tolerance);

    freeState(scratch);
    freeState(previous);

    return dist < tolerance;
}
