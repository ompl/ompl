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

#include <ompl/multilevel/planners/spqr/SPQRImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"
#include <boost/math/constants/constants.hpp>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>

#define foreach BOOST_FOREACH

ompl::multilevel::SPQRImpl::SPQRImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("SPQRImpl" + std::to_string(id_));
    randomWorkStates_.resize(5);
    getBundle()->allocStates(randomWorkStates_);

    setMetric("geodesic");
    setGraphSampler("randomvertex");
    setImportance("exponential");

    double d = (double)getBundle()->getStateDimension();
    double e = boost::math::constants::e<double>();
    kPRMStarConstant_ = e + (e / d);

    firstRun_ = true;

}

ompl::multilevel::SPQRImpl::~SPQRImpl()
{
    getBundle()->freeStates(randomWorkStates_);
}

void ompl::multilevel::SPQRImpl::grow()
{
    if (firstRun_)
    {
        init();
        firstRun_ = false;

        // if (hasBaseSpace())
        // {
        //   printConfiguration(qStart_);
        //   printConfiguration(qGoal_);
        //   std::cout << qStart_->index << std::endl;
        //   std::cout << qGoal_->index << std::endl;
        //     if (getPathRestriction()->hasFeasibleSection(qStart_, qGoal_))
        //     {
        //         if (sameComponentSparse(v_start_sparse, v_goal_sparse))
        //         {
        //             hasSolution_ = true;
        //         }
        //     }
        // }
    }
    // std::cout << "GROW" << getLevel() << std::endl;

    if (!sampleBundleValid(xRandom_->state))
    {
        return;
    }

    Configuration *xNew = new Configuration(getBundle(), xRandom_->state);

    addConfiguration(xNew);

    // connectNeighbors(xNew);

    // expand();

    if (!hasSolution_)
    {
        bool same_component = sameComponentSparse(v_start_sparse, v_goal_sparse);
        if (same_component)
        {
            hasSolution_ = true;
        }
    }
}

void ompl::multilevel::SPQRImpl::expand()
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

    int s =
        getBundle()->randomBounceMotion(Bundle_sampler_, q->state, randomWorkStates_.size(), randomWorkStates_, false);
    for (int i = 0; i < s; i++)
    {
        Configuration *tmp = new Configuration(getBundle(), randomWorkStates_[i]);
        addConfiguration(tmp);
        addEdge(q->index, tmp->index);
    }
}

// void ompl::multilevel::SPQRImpl::connectNeighbors(Configuration *q)
// {

//     // Calculate K
//     unsigned int k = static_cast<unsigned int>(ceil(kPRMStarConstant_ * log((double)boost::num_vertices(graph_))));

//     // DENSE GRAPH: find nearest neighbors to be conected to new sample
//     std::vector<Configuration *> r_nearest_neighbors;
//     nearestDatastructure_->nearestK(q, k, r_nearest_neighbors);

//     for (unsigned int i = 0; i < r_nearest_neighbors.size(); i++)
//     {
//         Configuration *q_neighbor = r_nearest_neighbors.at(i);
//         q->total_connection_attempts++;
//         q_neighbor->total_connection_attempts++;

//         if (getBundle()->checkMotion(q_neighbor->state, q->state))
//         {
//             addEdge(q_neighbor->index, q->index);
//             q->successful_connection_attempts++;
//             q_neighbor->successful_connection_attempts++;
//         }
//     }

//     // SPARSE GRAPH: Update its representative and interface nodes
//     std::vector<Configuration *> sparseGraphNeighborhood;
//     nearestSparse_->nearestR(q, sparseDelta_, sparseGraphNeighborhood);

//     for (Configuration *qn : sparseGraphNeighborhood)
//     {
//         if (getBundle()->checkMotion(q->state, qn->state))
//         {
//             q->representativeIndex = qn->index;
//             break;
//         }
//     }

//     if (q->representativeIndex >= 0)
//     {
//         std::vector<Vertex> interfaceNeighborhood;
//         std::set<Vertex> interfaceRepresentatives;

//         getInterfaceNeighborRepresentatives(q, interfaceRepresentatives);
//         getInterfaceNeighborhood(q, interfaceNeighborhood);
//         addToRepresentatives(q->index, q->representativeIndex, interfaceRepresentatives);

//         foreach (Vertex qp, interfaceNeighborhood)
//         {
//             normalized_index_type qp_rep = graph_[qp]->representativeIndex;
//             if (qp_rep < 0)
//                 continue;
//             removeFromRepresentatives(graph_[qp]);
//             getInterfaceNeighborRepresentatives(graph_[qp], interfaceRepresentatives);
//             addToRepresentatives(qp, qp_rep, interfaceRepresentatives);
//         }
//     }
// }

bool ompl::multilevel::SPQRImpl::isInfeasible()
{
    bool progressFailure = ((consecutiveFailures_ >= maxFailures_) && !hasSolution_);
    if (progressFailure)
    {
        OMPL_INFORM("Infeasibility detected with probability %f (no valid samples for %d rounds).",
                    1.0 - 1.0 / (double)consecutiveFailures_, consecutiveFailures_);
    }
    return progressFailure;
}
