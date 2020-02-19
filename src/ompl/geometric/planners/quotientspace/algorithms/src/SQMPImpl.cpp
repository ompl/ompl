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

#include <ompl/geometric/planners/quotientspace/algorithms/SQMPImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"

#define foreach BOOST_FOREACH

ompl::geometric::SQMPImpl::SQMPImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("SQMPImpl" + std::to_string(id_));
    randomWorkStates_.resize(5);
    Bundle->allocStates(randomWorkStates_);
}

ompl::geometric::SQMPImpl::~SQMPImpl()
{
    getBundle()->freeStates(randomWorkStates_);
    deleteConfiguration(xRandom_);
}

void ompl::geometric::SQMPImpl::grow()
{
    if (firstRun_)
    {
        Init();
        firstRun_ = false;
    }
    if( ++growExpandCounter_ % 5 == 0)
    {
        expand();
        return;
    }
    sampleBundleGoalBias(xRandom_->state, goalBias_);

    addMileStone(xRandom_);
}

void ompl::geometric::SQMPImpl::expand()
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
    
    //TODO: That seems weird. Wouldn't it change the State
    //of a given configuration in the graph?
    sampleBundle(q->state);
    addMileStone(q);
    
    int s = getBundle()->randomBounceMotion(Bundle_sampler_, q->state, randomWorkStates_.size(), randomWorkStates_, false);
    for (int i = 0; i < s; i++)
    {
        Configuration *tmp = new Configuration(Bundle, randomWorkStates_[i]);
        addMileStone(tmp);
        if(boost::edge(q->index, tmp->index, graph_).second)
            ompl::geometric::BundleSpaceGraph::addEdge(q->index, tmp->index);
    }
    
    /*foreach (Vertex v, boost::vertices(graphSparse_))
    {
        if(graphSparse_[v]->successful_connection_attempts == 0)
        {
            std::vector<Configuration *> r_nearest_neighbors;
            nearestSparse_->nearestR(graphSparse_[v], sparseDelta_, r_nearest_neighbors);

            for (unsigned int i = 0; i < r_nearest_neighbors.size(); i++)
            {
                Configuration *q_neighbor = r_nearest_neighbors.at(i);
                if (Bundle->checkMotion(q_neighbor->state, graphSparse_[v]->state))
                {
                    addEdgeSparse(q_neighbor->index, v);
                }
            }
        }
    }*/
}

void ompl::geometric::SQMPImpl::addMileStone(Configuration *q_random)
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
    //TODO: Why check for dense solution? Why not directly same component?
    if (isDenseFoundSolution_)
    {
        bool same_component = sameComponentSparse(v_start_sparse, v_goal_sparse);
        if(!same_component) {
            return;
        }
        hasSolution_ = true;
    }
}

ompl::geometric::BundleSpaceGraph::Configuration * ompl::geometric::SQMPImpl::addConfigurationDense(Configuration *q_random)
{
    Configuration *q_next = new Configuration(Bundle, q_random->state);
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

        if (Bundle->checkMotion(q_neighbor->state, q_random->state))
        {
            ompl::geometric::BundleSpaceGraph::addEdge(q_neighbor->index, v_next);
            q_next->successful_connection_attempts++;
            q_neighbor->successful_connection_attempts++;
            
            if (/*q_neighbor->isGoal && */!isDenseFoundSolution_)
            {
                bool same_component = sameComponent(vStart_, vGoal_);
                if (!same_component)
                {
                    isDenseFoundSolution_ = false;
                }
                else
                {
                    isDenseFoundSolution_ = true;
                }
            }
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

double ompl::geometric::SQMPImpl::getImportance() const
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

bool ompl::geometric::SQMPImpl::getPlannerTerminationCondition()
{
    return hasSolution_ || consecutiveFailures_ > maxFailures_;
}
