/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

#include "ompl/base/spaces/constraint/TangentBundleStateSpace.h"
#include "ompl/base/spaces/constraint/AtlasChart.h"

#include "ompl/util/Exception.h"

#include <Eigen/Core>

#include <cmath>

ompl::base::TangentBundleStateSpace::TangentBundleStateSpace(const StateSpacePtr &ambientSpace,
                                                             const ConstraintPtr &constraint)
  : AtlasStateSpace(ambientSpace, constraint, false)
{
    setName("TangentBundle" + space_->getName());
    setBiasFunction([&](AtlasChart *c) -> double {
        double d = 0;
        for (auto anchor : anchors_)
            d = std::max(d, distance(anchor, c->getOrigin()));

        return d;
    });
}

void ompl::base::TangentBundleStateSpace::sanityChecks() const
{
    constrainedSanityChecks(CONSTRAINED_STATESPACE_GEODESIC_CONTINUITY | CONSTRAINED_STATESPACE_SAMPLERS);

    double zero = std::numeric_limits<double>::epsilon();
    double eps = std::numeric_limits<double>::epsilon();
    unsigned int flags = STATESPACE_DISTANCE_DIFFERENT_STATES | STATESPACE_DISTANCE_SYMMETRIC |
                         STATESPACE_DISTANCE_BOUND | STATESPACE_RESPECT_BOUNDS | STATESPACE_ENFORCE_BOUNDS_NO_OP;

    StateSpace::sanityChecks(zero, eps, flags);
}

bool ompl::base::TangentBundleStateSpace::discreteGeodesic(const State *from, const State *to, bool interpolate,
                                                           std::vector<State *> *geodesic) const
{
    // We can't traverse the manifold if we don't start on it.
    if (!constraint_->isSatisfied(from))
        return false;

    auto afrom = from->as<StateType>();
    auto ato = to->as<StateType>();

    // Try to get starting chart from `from` state.
    AtlasChart *c = getChart(afrom);
    if (c == nullptr)
        return false;

    // Save a copy of the from state.
    if (geodesic != nullptr)
    {
        geodesic->clear();
        geodesic->push_back(cloneState(from));
    }

    auto &&svc = si_->getStateValidityChecker();

    // No need to traverse the manifold if we are already there
    const double tolerance = delta_;
    const double distTo = distance(from, to);
    if (distTo <= tolerance)
        return true;

    // Traversal stops if the ball of radius distMax centered at from is left
    const double distMax = lambda_ * distTo;

    // Create a scratch state to use for movement.
    auto scratch = cloneState(from)->as<StateType>();
    auto temp = cloneState(from)->as<StateType>();

    // Project from and to points onto the chart
    Eigen::VectorXd u_j(k_), u_b(k_);
    c->psiInverse(*scratch, u_j);
    c->psiInverse(*ato, u_b);

    bool done = false;
    std::size_t chartsCreated = 0;
    double dist = 0;

    const double sqDelta = delta_ * delta_;
    do
    {
        // Take a step towards the final state
        u_j += delta_ * (u_b - u_j).normalized();
        c->phi(u_j, *temp);

        const double step = distance(temp, scratch);

        if (step < std::numeric_limits<double>::epsilon())
            break;

        dist += step;

        if (!(interpolate || svc->isValid(scratch))                    // valid
            || distance(temp, from) > distMax || !std::isfinite(dist)  // exceed max dist
            || dist > distMax                                          // exceed wandering
            || chartsCreated > maxChartsPerExtension_)                 // exceed chart limit
            break;

        done = (u_b - u_j).squaredNorm() <= sqDelta;
        // Find or make a new chart if new state is off of current chart
        if (done || !c->inPolytope(u_j)                  // outside polytope
            || constraint_->distance(*temp) > epsilon_)  // to far from manifold
        {
            const bool onManifold = c->psi(u_j, *temp);
            if (!onManifold)
                break;

            copyState(scratch, temp);
            scratch->setChart(c);

            bool created = false;
            if ((c = getChart(scratch, true, &created)) == nullptr)
            {
                OMPL_ERROR("Treating singularity as an obstacle.");
                break;
            }
            chartsCreated += created;

            // Re-project onto the next chart.
            c->psiInverse(*scratch, u_j);
            c->psiInverse(*ato, u_b);

            done = (u_b - u_j).squaredNorm() <= sqDelta;
        }

        copyState(scratch, temp);

        // Keep the state in a list, if requested.
        if (geodesic != nullptr)
            geodesic->push_back(cloneState(scratch));

    } while (!done);

    const bool ret = distance(to, scratch) <= delta_;
    freeState(scratch);
    freeState(temp);

    return ret;
}

ompl::base::State *ompl::base::TangentBundleStateSpace::geodesicInterpolate(const std::vector<State *> &geodesic,
                                                                            const double t) const
{
    auto state = ConstrainedStateSpace::geodesicInterpolate(geodesic, t)->as<StateType>();
    if (!project(state))
        return geodesic[0];

    return state;
}

bool ompl::base::TangentBundleStateSpace::project(State *state) const
{
    auto astate = state->as<StateType>();
    auto &&svc = si_->getStateValidityChecker();

    Eigen::VectorXd u(k_);
    AtlasChart *chart = getChart(astate, true);
    chart->psiInverse(*astate, u);

    if (chart->psi(u, *astate)   // On manifold
        && svc->isValid(state))  // Valid state
        return true;

    return false;
}
