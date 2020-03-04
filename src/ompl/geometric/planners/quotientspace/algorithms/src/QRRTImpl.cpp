/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
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

/* Author: Andreas Orthey */
#include <ompl/geometric/planners/quotientspace/algorithms/QRRTImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

ompl::geometric::QRRTImpl::QRRTImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("QRRTImpl" + std::to_string(id_));
}

ompl::geometric::QRRTImpl::~QRRTImpl()
{
    deleteConfiguration(xRandom_);
}

void ompl::geometric::QRRTImpl::grow()
{
    //(0) If first run, add start configuration
    if (firstRun_)
    {
        init();
        firstRun_ = false;
    }

    //(1) Get Random Sample
    sampleBundleGoalBias(xRandom_->state, goalBias_);

    //(2) Get Nearest in Tree
    const Configuration *xNearest = nearest(xRandom_);

    //(3) Connect Nearest to Random
    Configuration *xNext = extendGraphTowards(xNearest, xRandom_);

    //(4) If extension was successful, check if we reached goal
    if(xNext)
    {
        // double dist = 0;
        // bool satisfied = goal_->isSatisfied(xNext->state, &dist);
        // if(dist < bestDist_)
        // {
        //     bestDist_ = dist;
        //     xApproximateNearest_ = xNext;
        // }
        bool satisfied = goal_->isSatisfied(xNext->state);
        if (satisfied)
        {
            // std::cout << bestDist_ << std::endl;
            vGoal_ = addConfiguration(qGoal_);
            addEdge(xNext->index, vGoal_);
            hasSolution_ = true;
        }
    }

}
