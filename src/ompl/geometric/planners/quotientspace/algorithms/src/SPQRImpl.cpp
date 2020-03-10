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

/* Author: Andreas Orthey, Sohaib Akbar */

#include <ompl/geometric/planners/quotientspace/algorithms/SPQRImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"

#define foreach BOOST_FOREACH

ompl::geometric::SPQRImpl::SPQRImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("SPQRImpl" + std::to_string(id_));
    randomWorkStates_.resize(5);
    getBundle()->allocStates(randomWorkStates_);
}

ompl::geometric::SPQRImpl::~SPQRImpl()
{
    getBundle()->freeStates(randomWorkStates_);
    deleteConfiguration(xRandom_);
}

void ompl::geometric::SPQRImpl::grow()
{
    if (firstRun_)
    {
        Init();
        firstRun_ = false;
    }
    if( ++iterations_ % 2 == 0)
    {
        expand();
        return;
    }
    sampleBundleGoalBias(xRandom_->state, goalBias_);

    addMileStone(xRandom_);
}

void ompl::geometric::SPQRImpl::expand()
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
    
    int s = getBundle()->randomBounceMotion(Bundle_sampler_, q->state, randomWorkStates_.size(), randomWorkStates_, false);
    for (int i = 0; i < s; i++)
    {
        Configuration *tmp = new Configuration(getBundle(), randomWorkStates_[i]);
        addMileStone(tmp);
        if(boost::edge(q->index, tmp->index, graph_).second)
            ompl::geometric::BundleSpaceGraph::addEdge(q->index, tmp->index);
        //q = tmp;
    }
}

void ompl::geometric::SPQRImpl::addMileStone(Configuration *q_random)
{
    Configuration *q_next = addConfigurationDense(q_random);

    findGraphNeighbors(q_next, graphNeighborhood, visibleNeighborhood);

    if (!checkAddCoverage(q_next, visibleNeighborhood))
        if (!checkAddConnectivity(q_next, visibleNeighborhood))
            if (!checkAddInterface(q_next, graphNeighborhood, visibleNeighborhood))
            {
                if (!checkAddPath(q_next))
                    ++consecutiveFailures_;
            }
    
    if (!hasSolution_)
    {
        bool same_component = sameComponentSparse(v_start_sparse, v_goal_sparse);
        if(!same_component) 
        {
            return;
        }
        hasSolution_ = true;
    }
}

ompl::geometric::BundleSpaceGraph::Configuration * ompl::geometric::SPQRImpl::addConfigurationDense(Configuration *q_random)
{
    Configuration *q_next = new Configuration(getBundle(), q_random->state);
    Vertex v_next = ompl::geometric::BundleSpaceGraph::addConfiguration(q_next);
    // totalNumberOfSamples_++;
    // totalNumberOfFeasibleSamples_++;

    // Calculate K
    unsigned int k = static_cast<unsigned int>(ceil(kPRMStarConstant_ * log((double) boost::num_vertices(graph_))));

    // find nearest neighbors to be conected to new sample
    std::vector<Configuration *> r_nearest_neighbors;
    ompl::geometric::BundleSpaceGraph::nearestDatastructure_->nearestK(q_next, k, r_nearest_neighbors);
    
    for (unsigned int i = 0; i < r_nearest_neighbors.size(); i++)
    {
        q_next->total_connection_attempts++;
        Configuration *q_neighbor = r_nearest_neighbors.at(i);
        q_neighbor->total_connection_attempts++;

        if (getBundle()->checkMotion(q_neighbor->state, q_random->state))
        {
            ompl::geometric::BundleSpaceGraph::addEdge(q_neighbor->index, v_next);
            q_next->successful_connection_attempts++;
            q_neighbor->successful_connection_attempts++;
        }
    }

    // Update its representative and interface nodes
    std::vector<Configuration *> graphNeighborhood;
    nearestSparse_->nearestR(q_next, sparseDelta_, graphNeighborhood); // Sparse Neighbors

    for (Configuration *qn : graphNeighborhood)
        if (getBundle()->checkMotion(q_next->state, qn->state))
        {
            q_next->representativeIndex = qn->index;
            break;
        }
    // return if rep not found
    if ( q_next->representativeIndex < 0 )
        return q_next;


    std::vector<Vertex> interfaceNeighborhood;
    std::set<Vertex> interfaceRepresentatives;

    getInterfaceNeighborRepresentatives(q_next, interfaceRepresentatives);
    getInterfaceNeighborhood(q_next, interfaceNeighborhood);
    addToRepresentatives(v_next, q_next->representativeIndex, interfaceRepresentatives);
    
    foreach (Vertex qp, interfaceNeighborhood)
    {
        normalized_index_type qp_rep = graph_[qp]->representativeIndex;
        if ( qp_rep == -1 )
            continue;
        removeFromRepresentatives(graph_[qp]);
        getInterfaceNeighborRepresentatives(graph_[qp], interfaceRepresentatives);
        addToRepresentatives(qp, qp_rep, interfaceRepresentatives);
    }
    return q_next;
}

bool ompl::geometric::SPQRImpl::getPlannerTerminationCondition()
{
    return hasSolution_ || consecutiveFailures_ > maxFailures_;
}
