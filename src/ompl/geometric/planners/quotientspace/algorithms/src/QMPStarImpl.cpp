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

/* Author: Andreas Orthey, Sohaib Akbar */

#include <ompl/geometric/planners/quotientspace/algorithms/QMPStarImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#define foreach BOOST_FOREACH

ompl::geometric::QMPStarImpl::QMPStarImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("QMPStarImpl" + std::to_string(id_));

    double d = (double)Bundle->getStateDimension();
    double e = boost::math::constants::e<double>();
    kPRMStarConstant_ = e + (e / d);
    
    randomWorkStates_.resize(5);
    Bundle->allocStates(randomWorkStates_);
}

ompl::geometric::QMPStarImpl::~QMPStarImpl()
{
    si_->freeStates(randomWorkStates_);
    deleteConfiguration(xRandom_);
}

void ompl::geometric::QMPStarImpl::grow()
{
    if (firstRun_)
    {
        init();
        vGoal_ = addConfiguration(qGoal_);
        firstRun_ = false;
    }

    //TODO: Why not use expand() also in QMP?
    if( ++growExpandCounter_ % 2 == 0)
    {
        expand();
        return;
    }

    sampleBundleGoalBias(xRandom_->state, goalBias_);
    addMileStone(xRandom_->state);
}

void ompl::geometric::QMPStarImpl::expand()
{
    PDF pdf;

    foreach (Vertex v, boost::vertices(graph_))
    {
        const unsigned long int t = graph_[v]->total_connection_attempts;
        pdf.add(graph_[v], (double)(t - graph_[v]->successful_connection_attempts) / (double)t);
    }

    if (pdf.empty())
        return;

    
    Configuration *q = pdf.sample(rng_.uniform01());

    int s = si_->randomBounceMotion(Bundle_sampler_, q->state, randomWorkStates_.size(), randomWorkStates_, false);
    if(s > 0)
    {
        Configuration *prev = q;
        Configuration *last = addMileStone(randomWorkStates_[--s]);
        for (int i = 0; i < s; i++)
        {
            Configuration *tmp = new Configuration(Bundle, randomWorkStates_[i]);
            addConfiguration(tmp);

            ompl::geometric::BundleSpaceGraph::addEdge(prev->index, tmp->index);
            prev = tmp;
        }
        if(!sameComponent(prev->index, last->index))
            ompl::geometric::BundleSpaceGraph::addEdge(prev->index, last->index);
    }
}

//TODO: Why not use same as in QMP?
ompl::geometric::BundleSpaceGraph::Configuration *ompl::geometric::QMPStarImpl::addMileStone(ompl::base::State *q_state)
{
    // add sample to graph
    Configuration *q_next = new Configuration(Bundle, q_state);
    Vertex v_next = addConfiguration(q_next);

    // Calculate K
    unsigned int k = static_cast<unsigned int>(ceil(kPRMStarConstant_ * log((double) boost::num_vertices(graph_))));

    // check for close neighbors
    std::vector<Configuration*> r_nearest_neighbors;
    BaseT::nearestDatastructure_->nearestK(q_next , k , r_nearest_neighbors);
    
    for(unsigned int i=0 ; i< r_nearest_neighbors.size(); i++)
    {
        Configuration* q_neighbor = r_nearest_neighbors.at(i);
        
        q_next->total_connection_attempts++;
        q_neighbor->total_connection_attempts++;
        
        if (Bundle->checkMotion(q_neighbor->state, q_next->state)) 
        {
            addEdge(q_neighbor->index, v_next);
            q_next->successful_connection_attempts++;
            q_neighbor->successful_connection_attempts++;

            if (/*q_neighbor->isGoal && */!hasSolution_)
            {
                if (sameComponent(vStart_, vGoal_))
                {
                    hasSolution_ = true;
                }
            }
        }

    }
    return q_next;
}

