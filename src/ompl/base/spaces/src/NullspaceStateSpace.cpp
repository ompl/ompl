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

#include "ompl/base/spaces/NullspaceStateSpace.h"

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"

#include <eigen3/Eigen/Core>

/// NullspaceStateSpace

/// Public

void ompl::base::NullspaceStateSpace::checkSpace(const SpaceInformation *si)
{
    if (dynamic_cast<NullspaceStateSpace *>(si->getStateSpace().get()) == nullptr)
        throw ompl::Exception("ompl::base::NullspaceStateSpace(): "
                              "si needs to use an NullspaceStateSpace!");
}

bool ompl::base::NullspaceStateSpace::traverseManifold(const State *from, const State *to, const bool interpolate,
                                                       std::vector<State *> *stateList) const
{
    // We can't move along the manifold if we were never there in the first place
    if (!constraint_->isSatisfied(from))
        return false;

    // Save a copy of the from state.
    if (stateList != nullptr)
    {
        stateList->clear();
        stateList->push_back(cloneState(from));
    }

    const double tolerance = delta_ + std::numeric_limits<double>::epsilon();

    double distToGo = distance(from, to);
    double distTraveled = 0;
    const double distMax = distToGo;

    // No need to traverse the manifold if we are already there
    if (distToGo < tolerance)
        return true;

    // Get vector representations
    const StateType *fromAsType = from->as<StateType>();
    const StateType *toAsType = to->as<StateType>();
    Eigen::Ref<const Eigen::VectorXd> x_from = fromAsType->constVectorView();
    Eigen::Ref<const Eigen::VectorXd> x_to = toAsType->constVectorView();

    const StateValidityCheckerPtr &svc = si_->getStateValidityChecker();

    State *previous = cloneState(from);
    State *scratch = allocState();

    Eigen::Map<const Eigen::VectorXd> x_prev = previous->as<StateType>()->constVectorView();
    Eigen::Map<Eigen::VectorXd> x_scratch = scratch->as<StateType>()->vectorView();

    Eigen::MatrixXd j(n_ - k_, n_);
kj
    do
    {
        WrapperStateSpace::interpolate(previous, to, delta_ / dist, scratch);
        constraint_->jacobian(x_prev, j);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd = j.jacobiSvd(Eigen::ComputeFullV);
        x_scratch += svd.matrixV() * svd.matrixV().transpose() * (x_scratch - x_prev);

        double step = distance(previous, scratch);

        const bool valid = interpolate || svc->isValid(scratch);
        const bool deviated = step > 2.0 * delta_;
        const bool wandering = (distTravled += step) > 2.0 * distMax;
        const bool 
        if (!valid || deviated || wandering)
            break;

        // Check if we are no closer than before
        double newDist = distance(scratch, to);
        if (newDist >= dist)
            break;

        const bool toFarFromManifold = constraint_->distance(x_scratch) > epsilon_;
        if (toFarFromManifold || newDist < tolerance)
        {
            if (!constraint_->project(scratch))
                break;

            newDist = distance(scratch, to);
        }

        distToGo = newDist;

        copyState(previous, scratch);

        // Store the new state
        if (stateList != nullptr)
            stateList->push_back(cloneState(scratch));

    } while (dist > tolerance);

    freeState(scratch);
    freeState(previous);

    return dist < tolerance;
}

ompl::base::State *ompl::base::NullspaceStateSpace::piecewiseInterpolate(const std::vector<State *> &stateList,
                                                                         const double t) const
{
    StateType *state = ConstrainedStateSpace::piecewiseInterpolate(stateList, t)->as<StateType>();
    if (!constraint_->project(state))
        return nullptr;

    return state;
}
