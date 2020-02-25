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

#include <ompl/geometric/planners/quotientspace/algorithms/QRRTStarImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#define foreach BOOST_FOREACH

ompl::geometric::QRRTStarImpl::QRRTStarImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("QRRTStarImpl" + std::to_string(id_));
    double d = (double)getBundle()->getStateDimension();
    double e = boost::math::constants::e<double>();
    // k > 2^(d + 1) * e * (1 + 1 / d).
    k_rrt_Constant_ = 1.1 * (std::pow(2, d + 1) * e * (1.0 + 1.0 / d));
}

ompl::geometric::QRRTStarImpl::~QRRTStarImpl()
{
    deleteConfiguration(xRandom_);
}

void ompl::geometric::QRRTStarImpl::grow()
{
    if (firstRun_)
    {
        init();
        firstRun_ = false;
    }

    //(1) Get Random Sample
    sampleBundleGoalBias(xRandom_->state, goalBias_);

    //(2) Get Nearest in Tree
    const Configuration *q_nearest = nearest(xRandom_);

    //(3) Connect Nearest to Random
    double d = distance(q_nearest, xRandom_);
    if (d > maxDistance_)
    {
        getBundle()->getStateSpace()->interpolate(q_nearest->state, xRandom_->state, maxDistance_ / d, xRandom_->state);
    }

    //(4) Check if Motion is correct
    if (getBundle()->checkMotion(q_nearest->state, xRandom_->state))
    {
        Configuration *q_new = new Configuration(getBundle(), xRandom_->state);
        Vertex v_next = addConfiguration(q_new);

        // Find nearby neighbors of the new motion
        std::vector<Configuration *> nearestNbh;
        
        // calculate k
        unsigned int k = std::ceil(k_rrt_Constant_ * log((double) boost::num_vertices(graph_)));
        nearestDatastructure_->nearestK(q_new, k, nearestNbh);

        // Find neighbor with minimum Cost
        const Configuration *q_min = q_nearest;

        base::Cost min_line_cost = getOptimizationObjectivePtr()->motionCost(q_nearest->state, q_new->state);
        ompl::base::Cost min_cost = getOptimizationObjectivePtr()->combineCosts(q_nearest->cost, line_cost);
        
        // if valid neighbors in first time than no need to check motion in rewire step for ith neighbor {-1, 0, 1}
        int validNeighbor[nearestNbh.size()];
        validNeighbor[0] = 1; // nearest is already valid

        /* By default, neighborhood states are sorted by distance, so start from 1 because first will be nearest, sort in ascending distance */
        for(std::siz_t i=1; i< nearestNbh.size(); i++)
        {
            Configuration* q_near = nearestNbh.at(i);
            validNeighbor[i] = 0;

            base::Cost line_cost = getOptimizationObjectivePtr()->motionCost(q_near->state, q_new->state);
            base::Cost new_cost = getOptimizationObjectivePtr()->combineCosts(q_near->cost, line_cost);

            if (getBundle()->isCostBetterThan(new_cost , min_cost))
            {
                if(distance(q_near, q_new) < maxDistance_ && getBundle()->checkMotion(q_near->state, q_new->state))
                {
                    q_min = q_near;
                    min_line_cost = line_cost;
                    min_cost = new_cost;
                    validNeighbor[i] = 1;
                }
                else validNeighbor[i] = -1;
            }
        }

        // (5) add edge assign cost
        addEdge(q_min->index, v_next);
        q_new->lineCost = min_line_cost;
        q_new->cost = min_cost;
        q_new->parent = q_min->index;
        q_min->children.push_back(q_new);
        
        // (6) Rewire the tree
        for (std::siz_t i=0 ; i< nearestNbh.size(); i++)
        {
            Configuration* q_near = nearestNbh.at(i);
            if (q_near->index != q_new->parent)
            {
                base::Cost line_cost = getBundle()->motionCost(q_new->state, q_near->state);
                base::Cost new_cost = getBundle()->combineCosts(q_new->cost, line_cost);
                
                if (getBundle()->isCostBetterThan(new_cost, q_near->cost))
                {
                    // check neighbor validity if it wasn´t checked before
                    if (validNeighbor[i] == 0)
                    {
                        if (distance(q_near, q_new) < maxDistance_ && getBundle()->checkMotion(q_near->state, q_new->state))
                            validNeighbor[i] = 1;
                    }
                    if (validNeighbor[i] == 1)
                    {
                        // remove node from children of its parent node
                        for (auto it = q_near->parent->children.begin(); it != q_near->parent->children.end(); ++it)
                        {
                            if (*it == q_near)
                            {
                                q_near->parent->children.erase(it);
                                break;
                            }
                        }

                        // remove the edge with old parent
                        boost::remove_edge(q_near->parent, q_near->index, graph_);
                        addEdge(q_new->index, q_near->index);
                        
                        // update node parent
                        q_near->parent = q_new;
                        q_near->lineCost = line_cost;
                        q_near->cost = new_cost;
                        q_new->children.push_back(q_near);
                        
                        // update node's children costs
                        updateChildCosts(q_near);
                    }
                }
            }
        }
        
        if (!hasSolution_ || !hasChild())
        {
            // (7) check if this sample satisfies the goal

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

void ompl::geometric::QRRTStarImpl::updateChildCosts(Configuration *q)
{
    for (std::size_t i = 0; i < q->children.size(); ++i)
    {
        q->children[i]->cost = getOptimizationObjectivePtr()->combineCosts(q->cost, q->children[i]->lineCost);
        updateChildCosts(q->children[i]);
    }
}
