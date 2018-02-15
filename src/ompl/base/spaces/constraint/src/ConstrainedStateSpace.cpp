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

#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include "ompl/util/Exception.h"

/// ConstrainedMotionValidator

/// Public

ompl::base::ConstrainedMotionValidator::ConstrainedMotionValidator(SpaceInformation *si)
  : MotionValidator(si), ss_(*si->getStateSpace()->as<ConstrainedStateSpace>())
{
    ConstrainedStateSpace::checkSpace(si);
}

ompl::base::ConstrainedMotionValidator::ConstrainedMotionValidator(const SpaceInformationPtr &si)
  : MotionValidator(si), ss_(*si->getStateSpace()->as<ConstrainedStateSpace>())
{
    ConstrainedStateSpace::checkSpace(si.get());
}

bool ompl::base::ConstrainedMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    return ss_.getConstraint()->isSatisfied(s1) && ss_.getConstraint()->isSatisfied(s2) && ss_.traverseManifold(s1, s2);
}

bool ompl::base::ConstrainedMotionValidator::checkMotion(const State *s1, const State *s2,
                                                         std::pair<State *, double> &lastValid) const
{
    // Invoke the manifold-traversing algorithm to save intermediate states
    std::vector<ompl::base::State *> stateList;
    bool reached = ss_.traverseManifold(s1, s2, false, &stateList);

    // We are supposed to be able to assume that s1 is valid. However, it's not
    // on rare occasions, and I don't know why. This makes stateList empty.
    if (stateList.empty())
    {
        if (lastValid.first != nullptr)
            ss_.copyState(lastValid.first, s1);
        lastValid.second = 0;
        return false;
    }

    double distanceTraveled = 0;
    for (std::size_t i = 0; i < stateList.size() - 1; i++)
    {
        if (!reached)
            distanceTraveled += ss_.distance(stateList[i], stateList[i + 1]);
        ss_.freeState(stateList[i]);
    }

    if (!reached && (lastValid.first != nullptr))
    {
        // Check if manifold traversal stopped early and set its final state as
        // lastValid.
        ss_.copyState(lastValid.first, stateList.back());
        // Compute the interpolation parameter of the last valid
        // state. (Although if you then interpolate, you probably won't get this
        // exact state back.)
        double approxDistanceRemaining = ss_.distance(lastValid.first, s2);
        lastValid.second = distanceTraveled / (distanceTraveled + approxDistanceRemaining);
    }

    ss_.freeState(stateList.back());

    return ss_.getConstraint()->isSatisfied(s1) && ss_.getConstraint()->isSatisfied(s2) && reached;
}

ompl::base::ConstrainedStateSpace::ConstrainedStateSpace(const StateSpacePtr space, const ConstraintPtr constraint)
  : WrapperStateSpace(space)
  , si_(nullptr)
  , constraint_(std::move(constraint))
  , n_(space->getDimension())
  , k_(constraint_->getManifoldDimension())
  , setup_(false)
  , caching_(true)
{
    setDelta(magic::CONSTRAINED_STATE_SPACE_DELTA);
}

void ompl::base::ConstrainedStateSpace::checkSpace(const SpaceInformation *si)
{
    if (dynamic_cast<ConstrainedStateSpace *>(si->getStateSpace().get()) == nullptr)
        throw ompl::Exception("ompl::base::ConstrainedStateSpace(): "
                              "si needs to use an ConstrainedStateSpace!");
}

void ompl::base::ConstrainedStateSpace::setSpaceInformation(SpaceInformation *si)
{
    // Check that the object is valid
    if (si == nullptr)
        throw ompl::Exception("ompl::base::ConstrainedStateSpace::setSpaceInformation(): "
                              "si is nullptr.");
    if (si->getStateSpace().get() != this)
        throw ompl::Exception("ompl::base::ConstrainedStateSpace::setSpaceInformation(): "
                              "si for ConstrainedStateSpace must be constructed from the same state space object.");

    si_ = si;
    si_->setStateValidityCheckingResolution(delta_);
}

void ompl::base::ConstrainedStateSpace::setup()
{
    if (setup_)
        return;

    if (si_ == nullptr)
        throw ompl::Exception("ompl::base::ConstrainedStateSpace::setup(): "
                              "Must associate a SpaceInformation object to the ConstrainedStateSpace via "
                              "setStateInformation() before use.");

    WrapperStateSpace::setup();

    setup_ = true;
    setDelta(delta_);  // This makes some setup-related calls
}

void ompl::base::ConstrainedStateSpace::clear()
{
}

ompl::base::State *ompl::base::ConstrainedStateSpace::allocState() const
{
    auto *state = new StateType(space_->allocState(), n_);
    state->setValues(space_->getValueAddressAtIndex(state->getState(), 0));
    return state;
}

void ompl::base::ConstrainedStateSpace::interpolate(const State *from, const State *to, const double t,
                                                    State *state) const
{
    // Get the list of intermediate states along the manifold.
    std::vector<State *> stateList;
    auto temp = from;

    if (traverseManifold(from, to, true, &stateList))
    {
        stateList.push_back(cloneState(to));
        temp = piecewiseInterpolate(stateList, t);
    }

    copyState(state, temp);

    for (auto s : stateList)
        freeState(s);
}

ompl::base::State *ompl::base::ConstrainedStateSpace::piecewiseInterpolate(const std::vector<State *> &stateList,
                                                                           const double t) const
{
    unsigned int n = stateList.size();
    double d[n];

    // Compute partial sums of distances between intermediate states.
    d[0] = 0.;
    for (unsigned int i = 1; i < n; ++i)
        d[i] = d[i - 1] + distance(stateList[i - 1], stateList[i]);

    // Find the two adjacent states that t lies between.
    const double last = d[n - 1];
    if (last <= std::numeric_limits<double>::epsilon())
        return stateList[0];

    else
    {
        unsigned int i = 0;
        while (i < (n - 1) && (d[i] / last) <= t)
            i++;

        const double t1 = d[i] / last - t;
        const double t2 = (i <= n - 2) ? d[i + 1] / last - t : 1;

        return (t1 < t2) ? stateList[i] : stateList[i + 1];
    }
}
