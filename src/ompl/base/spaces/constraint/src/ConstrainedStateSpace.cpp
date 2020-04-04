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

#include "ompl/tools/config/MagicConstants.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include "ompl/util/Exception.h"

/// ConstrainedMotionValidator

/// Public

ompl::base::ConstrainedMotionValidator::ConstrainedMotionValidator(SpaceInformation *si)
  : MotionValidator(si), ss_(*si->getStateSpace()->as<ConstrainedStateSpace>())
{
}

ompl::base::ConstrainedMotionValidator::ConstrainedMotionValidator(const SpaceInformationPtr &si)
  : MotionValidator(si), ss_(*si->getStateSpace()->as<ConstrainedStateSpace>())
{
}

bool ompl::base::ConstrainedMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    return ss_.getConstraint()->isSatisfied(s2) && ss_.discreteGeodesic(s1, s2, false);
}

bool ompl::base::ConstrainedMotionValidator::checkMotion(const State *s1, const State *s2,
                                                         std::pair<State *, double> &lastValid) const
{
    // Invoke the manifold-traversing algorithm to save intermediate states
    std::vector<ompl::base::State *> stateList;
    bool reached = ss_.discreteGeodesic(s1, s2, false, &stateList);

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
    return ss_.getConstraint()->isSatisfied(s2) && reached;
}

ompl::base::ConstrainedStateSpace::ConstrainedStateSpace(const StateSpacePtr &space, const ConstraintPtr &constraint)
  : WrapperStateSpace(space)
  , constraint_(constraint)
  , n_(space->getDimension())
  , k_(constraint_->getManifoldDimension())
{
    setDelta(magic::CONSTRAINED_STATE_SPACE_DELTA);
}

void ompl::base::ConstrainedStateSpace::constrainedSanityChecks(unsigned int flags) const
{
    auto *s1 = allocState()->as<StateType>();
    State *s2 = allocState();
    StateSamplerPtr ss = allocStateSampler();

    bool isTraversable = false;

    for (unsigned int i = 0; i < ompl::magic::TEST_STATE_COUNT; ++i)
    {
        bool satisfyGeodesics = false;
        bool continuityGeodesics = false;

        ss->sampleUniform(s1);

        // Verify that the provided Jacobian routine for the constraint is close
        // to the numerical approximation.
        if (flags & CONSTRAINED_STATESPACE_JACOBIAN)
        {
            Eigen::MatrixXd j_a(n_ - k_, n_), j_n(n_ - k_, n_);

            constraint_->jacobian(*s1, j_a);              // Provided routine
            constraint_->Constraint::jacobian(*s1, j_n);  // Numerical approximation

            if ((j_a - j_n).norm() > constraint_->getTolerance())
                throw Exception("Constraint Jacobian deviates from numerical approximation.");
        }

        ss->sampleUniformNear(s2, s1, 10 * delta_);

        // Check that samplers are returning constraint satisfying samples.
        if (flags & CONSTRAINED_STATESPACE_SAMPLERS && (!constraint_->isSatisfied(s1) || !constraint_->isSatisfied(s2)))
            throw Exception("State samplers generate constraint unsatisfying states.");

        std::vector<State *> geodesic;
        // Make sure that the manifold is traversable at least once.
        if ((isTraversable |= discreteGeodesic(s1, s2, true, &geodesic)))
        {
            // Verify that geodesicInterpolate returns a constraint satisfying state.
            if (flags & CONSTRAINED_STATESPACE_GEODESIC_INTERPOLATE &&
                !constraint_->isSatisfied(geodesicInterpolate(geodesic, 0.5)))
                throw Exception("Geodesic interpolate returns unsatisfying configurations.");

            State *prev = nullptr;
            for (auto s : geodesic)
            {
                // Make sure geodesics contain only constraint satisfying states.
                if (flags & CONSTRAINED_STATESPACE_GEODESIC_SATISFY)
                    satisfyGeodesics |= !constraint_->isSatisfied(s);

                // Make sure geodesics have some continuity.
                if (flags & CONSTRAINED_STATESPACE_GEODESIC_CONTINUITY && prev != nullptr)
                    continuityGeodesics |= distance(prev, s) > lambda_ * delta_;

                prev = s;
            }

            for (auto s : geodesic)
                freeState(s);
        }

        if (satisfyGeodesics)
            throw Exception("Discrete geodesic computation generates invalid states.");

        if (continuityGeodesics)
            throw Exception("Discrete geodesic computation generates non-continuous states.");
    }

    freeState(s1);
    freeState(s2);

    if (!isTraversable)
        throw Exception("Unable to compute discrete geodesic on constraint.");
}

void ompl::base::ConstrainedStateSpace::sanityChecks() const
{
    constrainedSanityChecks(~0);

    double zero = std::numeric_limits<double>::epsilon();
    double eps = std::numeric_limits<double>::epsilon();
    unsigned int flags = STATESPACE_DISTANCE_DIFFERENT_STATES | STATESPACE_DISTANCE_SYMMETRIC |
                         STATESPACE_DISTANCE_BOUND | STATESPACE_RESPECT_BOUNDS | STATESPACE_ENFORCE_BOUNDS_NO_OP;

    StateSpace::sanityChecks(zero, eps, flags);
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
}

void ompl::base::ConstrainedStateSpace::setDelta(double delta)
{
    if (delta <= 0)
        throw ompl::Exception("ompl::base::ConstrainedStateSpace::setDelta(): "
                              "delta must be positive.");
    delta_ = delta;

    if (setup_)
    {
        setLongestValidSegmentFraction(delta_ / getMaximumExtent());
        si_->setStateValidityCheckingResolution(delta_);
    }
}

void ompl::base::ConstrainedStateSpace::setup()
{
    if (setup_)
        return;

    if (si_ == nullptr)
        throw ompl::Exception("ompl::base::ConstrainedStateSpace::setup(): "
                              "Must associate a SpaceInformation object to the ConstrainedStateSpace via"
                              "setStateInformation() before use.");

    WrapperStateSpace::setup();

    setDelta(delta_);  // This makes some setup-related calls
    setup_ = true;

    setDelta(delta_);

    // Call again to make sure information propagates properly to both wrapper
    // and underlying space.
    WrapperStateSpace::setup();

    // Check if stride length of underlying state variables is 1
    auto *state = space_->allocState();
    bool flag = true;
    for (unsigned int i = 1; i < space_->getDimension() && flag; ++i)
    {
        std::size_t newStride = space_->getValueAddressAtIndex(state, i) - space_->getValueAddressAtIndex(state, i - 1);
        flag = newStride == 1;
    }
    space_->freeState(state);

    if (!flag)
        throw ompl::Exception("ompl::base::ConstrainedStateSpace::setup(): "
                              "Stride length of member variables != 1, cannot translate into dense vector.");
}

void ompl::base::ConstrainedStateSpace::clear()
{
}

ompl::base::State *ompl::base::ConstrainedStateSpace::allocState() const
{
    return new StateType(this);
}

void ompl::base::ConstrainedStateSpace::interpolate(const State *from, const State *to, const double t,
                                                    State *state) const
{
    // Get the list of intermediate states along the manifold.
    std::vector<State *> geodesic;

    // Default to returning `from' if traversal fails.
    auto temp = from;
    if (discreteGeodesic(from, to, true, &geodesic))
        temp = geodesicInterpolate(geodesic, t);

    copyState(state, temp);

    for (auto s : geodesic)
        freeState(s);
}

ompl::base::State *ompl::base::ConstrainedStateSpace::geodesicInterpolate(const std::vector<State *> &geodesic,
                                                                          const double t) const
{
    unsigned int n = geodesic.size();
    auto *d = new double[n];

    // Compute partial sums of distances between intermediate states.
    d[0] = 0.;
    for (unsigned int i = 1; i < n; ++i)
        d[i] = d[i - 1] + distance(geodesic[i - 1], geodesic[i]);

    // Find the two adjacent states that t lies between, and return the closer.
    const double last = d[n - 1];
    if (last <= std::numeric_limits<double>::epsilon())
    {
        delete[] d;
        return geodesic[0];
    }
    else
    {
        unsigned int i = 0;
        while (i < (n - 1) && (d[i] / last) <= t)
            i++;

        const double t1 = d[i] / last - t;
        const double t2 = (i <= n - 2) ? d[i + 1] / last - t : 1;

        delete[] d;
        assert((t1 < t2 || std::abs(t1 - t2) < std::numeric_limits<double>::epsilon())  ? (i < geodesic.size()) : (i + 1 < geodesic.size()));
        return (t1 < t2 || std::abs(t1 - t2) < std::numeric_limits<double>::epsilon()) ? geodesic[i] : geodesic[i + 1];
    }
}
