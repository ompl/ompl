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

#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "ompl/base/spaces/constraint/AtlasChart.h"

#include "ompl/util/Exception.h"

#include <eigen3/Eigen/Core>

#include <cmath>

bool ompl::base::TangentBundleStateSpace::traverseManifold(const State *from, const State *to, bool interpolate,
                                                           std::vector<ompl::base::State *> *stateList,
                                                           bool endpoints) const
{
    // We can't traverse the manifold if we don't start on it.
    if (!constraint_->isSatisfied(from))
        return false;

    auto &&fromAsType = from->as<StateType>();
    auto &&toAsType = to->as<StateType>();

    // Try to get starting chart from `from` state.
    AtlasChart *c = getChart(fromAsType);
    if (c == nullptr)
        return false;

    // Save a copy of the from state.
    if (stateList != nullptr)
    {
        stateList->clear();

        if (endpoints)
            stateList->push_back(cloneState(from));
    }

    auto &&svc = si_->getStateValidityChecker();

    // No need to traverse the manifold if we are already there
    const double tolerance = delta_;
    if (distance(from, to) <= tolerance)
        return true;

    // Get vector representations
    auto &&x_from = fromAsType->constVectorView();
    auto &&x_to = toAsType->constVectorView();

    // Traversal stops if the ball of radius distMax centered at x_from is left
    const double distMax = (x_from - x_to).norm();

    // Create a scratch state to use for movement.
    auto &&scratch = cloneState(from)->as<StateType>();
    auto &&x_scratch = scratch->vectorView();
    Eigen::VectorXd x_temp(n_);

    // Project from and to points onto the chart
    Eigen::VectorXd u_j(k_), u_b(k_);
    c->psiInverse(x_scratch, u_j);
    c->psiInverse(x_to, u_b);

    bool done = false;
    std::size_t chartsCreated = 0;
    double dist = 0;

    const double sqDelta = std::pow(delta_, 2);
    do
    {
        // Take a step towards the final state
        u_j += delta_ * (u_b - u_j).normalized();
        c->phi(u_j, x_temp);

        const double step = (x_temp - x_scratch).norm();
        dist += step;

        const bool valid = interpolate || svc->isValid(scratch);
        const bool exceedMaxDist = (x_temp - x_from).norm() > distMax || !std::isfinite(dist);
        const bool exceedWandering = dist > (lambda_ * distMax);
        const bool exceedChartLimit = chartsCreated > maxChartsPerExtension_;
        if (!valid || exceedMaxDist || exceedWandering || exceedChartLimit)
            break;

        const bool outsidePolytope = !c->inPolytope(u_j);
        const bool toFarFromManifold = constraint_->distance(x_temp) > epsilon_;

        done = (u_b - u_j).squaredNorm() <= sqDelta;

        // Find or make a new chart if new state is off of current chart
        if (outsidePolytope || toFarFromManifold || done)
        {
            const bool onManifold = c->psi(u_j, x_temp);
            if (!onManifold)
                break;

            x_scratch = x_temp;
            scratch->setChart(c);

            bool created = false;
            if ((c = getChart(scratch, true, &created)) == nullptr)
            {
                OMPL_ERROR("ompl::base::TangentBundleStateSpace::traverseManifold(): Treating singularity as an "
                           "obstacle.");
                break;
            }
            chartsCreated += created;

            // Re-project onto the next chart.
            c->psiInverse(x_scratch, u_j);
            c->psiInverse(x_to, u_b);

            done = (u_b - u_j).squaredNorm() <= sqDelta;
        }

        x_scratch = x_temp;

        // Keep the state in a list, if requested.
        if (stateList != nullptr && ((endpoints && !done) || !endpoints))
            stateList->push_back(cloneState(scratch));

    } while (!done);

    const bool ret = done && (x_to - x_scratch).squaredNorm() <= sqDelta;
    freeState(scratch);

    return ret;
}

ompl::base::State *ompl::base::TangentBundleStateSpace::piecewiseInterpolate(const std::vector<State *> &stateList,
                                                                             const double t) const
{
    auto state = ConstrainedStateSpace::piecewiseInterpolate(stateList, t)->as<StateType>();
    auto &&svc = si_->getStateValidityChecker();

    Eigen::VectorXd u(k_);
    auto chart = getChart(state);
    chart->psiInverse(state->constVectorView(), u);

    if (!chart->psi(u, state->vectorView()) && !svc->isValid(state))
        return nullptr;

    return state;
}
