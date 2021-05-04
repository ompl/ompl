/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey, Sohaib Akbar */

#include <ompl/multilevel/planners/sparse/SMLRImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"
#include <boost/math/constants/constants.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::multilevel;

SMLRImpl::SMLRImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("SMLRImpl" + std::to_string(id_));
    randomWorkStates_.resize(5);
    getBundle()->allocStates(randomWorkStates_);

    setMetric("geodesic");
    setGraphSampler("visibilityregion");

    firstRun_ = true;
    isInfeasible_ = false;
}

void SMLRImpl::clear()
{
    BaseT::clear();
    firstRun_ = true;
    isInfeasible_ = false;
}

SMLRImpl::~SMLRImpl()
{
    getBundle()->freeStates(randomWorkStates_);
}

double SMLRImpl::getImportance() const
{
    return 1.0 / (consecutiveFailures_ + 1);
}

void SMLRImpl::grow()
{
    if (firstRun_)
    {
        init();
        firstRun_ = false;

        getGoalPtr()->sampleGoal(qGoal_->state);
        addConfiguration(qGoal_);
        goalConfigurations_.push_back(qGoal_);

        findSection();
    }

    // if(pis_.getSampledGoalsCount() < getGoalPtr()->maxSampleCount())
    // {
    //     const base::State *state = pis_.nextGoal();
    //     Configuration qgoal = new Configuration(getBundle(), state);
    //     qgoal->isGoal = true;
    //     goalConfigurations_.push_back(qgoal);
    // }

    if (!sampleBundleValid(xRandom_->state))
    {
        return;
    }

    Configuration *xNew = new Configuration(getBundle(), xRandom_->state);

    addConfigurationConditional(xNew);

    if (!hasSolution_)
    {
        bool same_component = sameComponent(getStartIndex(), getGoalIndex());
        if (same_component)
        {
            hasSolution_ = true;
        }
    }
}

bool SMLRImpl::hasConverged()
{
    bool progressFailure = (consecutiveFailures_ >= maxFailures_);
    if (progressFailure)
    {
        OMPL_INFORM("Converged with probability %f (no valid samples for %d rounds).",
        (1.0 - 1.0 / (double)consecutiveFailures_), 
        consecutiveFailures_);
    }
    return progressFailure;
}

bool SMLRImpl::isInfeasible()
{
    bool progressFailure = ((consecutiveFailures_ >= maxFailures_) && !hasSolution_);
    if (progressFailure)
    {
        OMPL_INFORM("Infeasibility detected with probability %f (no valid samples for %d rounds).",
                    (1.0 - 1.0 / (double)consecutiveFailures_), consecutiveFailures_);
        isInfeasible_ = true;
    }
    return progressFailure;
}
