/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
    qRandom_ = new Configuration(Bundle);
}

ompl::geometric::QRRTImpl::~QRRTImpl()
{
    deleteConfiguration(qRandom_);
}

bool ompl::geometric::QRRTImpl::getSolution(base::PathPtr &solution)
{
    if (hasSolution_)
    {
        bool baset_sol = BaseT::getSolution(solution);
        if (baset_sol)
        {
            shortestPathVertices_ = shortestVertexPath_;
        }
        return baset_sol;
    }
    else
    {
        return false;
    }
}

void ompl::geometric::QRRTImpl::grow()
{
    if (firstRun_)
    {
        init();
        firstRun_ = false;
    }

    //(1) Get Random Sample
    sampleBundleGoalBias(qRandom_->state, goalBias_);

    //(2) Get Nearest in Tree
    const Configuration *q_nearest = nearest(qRandom_);

    //(3) Connect Nearest to Random
    double d = Bundle->distance(q_nearest->state, qRandom_->state);
    if (d > maxDistance_)
    {
        Bundle->getStateSpace()->interpolate(q_nearest->state, qRandom_->state, maxDistance_ / d, qRandom_->state);
    }

    //(4) Check if Motion is correct
    if (Bundle->checkMotion(q_nearest->state, qRandom_->state))
    {
        Configuration *q_next = new Configuration(Bundle, qRandom_->state);
        Vertex v_next = addConfiguration(q_next);

        if (!hasSolution_)
        {
            // only add edge if no solution exists
            addEdge(q_nearest->index, v_next);

            double dist = 0.0;
            bool satisfied = goal_->isSatisfied(q_next->state, &dist);
            if (satisfied)
            {
                vGoal_ = addConfiguration(qGoal_);
                addEdge(q_nearest->index, vGoal_);
                hasSolution_ = true;
            }
        }
    }
}

double ompl::geometric::QRRTImpl::getImportance() const
{
    // Should depend on
    // (1) level : The higher the level, the more importance
    // (2) total samples: the more we already sampled, the less important it
    // becomes
    // (3) has solution: if it already has a solution, we should explore less
    // (only when nothing happens on other levels)
    // (4) vertices: the more vertices we have, the less important (let other
    // levels also explore)
    //
    // exponentially more samples on level i. Should depend on ALL levels.
    // const double base = 2;
    // const double normalizer = powf(base, level);
    // double N = (double)GetNumberOfVertices()/normalizer;
    double N = (double)getNumberOfVertices();
    return 1.0 / (N + 1);
}
