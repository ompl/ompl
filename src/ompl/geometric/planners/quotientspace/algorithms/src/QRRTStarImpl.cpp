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
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#define foreach BOOST_FOREACH

ompl::geometric::QRRTStarImpl::QRRTStarImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("QRRTStarImpl" + std::to_string(id_));
    Planner::declareParam<double>(
        "useKNearest_", 
        this, 
        &ompl::geometric::QRRTStarImpl::setKNearest, 
        &ompl::geometric::QRRTStarImpl::getKNearest, 
        "0,1");
    double d = (double)Bundle->getStateDimension();
    double e = boost::math::constants::e<double>();
    // k > 2^(d + 1) * e * (1 + 1 / d).
    k_rrt_Constant_ = std::pow(2, d + 1) * e * (1.0 + 1.0 / d);
    symmetric_ = Bundle->getStateSpace()->hasSymmetricInterpolate();
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
    Configuration *q_nearest = nearestDatastructure_->nearest(xRandom_);

    //(3) Connect Nearest to Random
    double d = distance(q_nearest, xRandom_);
    if (d > maxDistance_)
    {
        Bundle->getStateSpace()->interpolate(q_nearest->state, xRandom_->state, maxDistance_ / d, xRandom_->state);
    }

    //(4) Check if Motion is correct
    if (Bundle->checkMotion(q_nearest->state, xRandom_->state))
    {
        // Find nearby neighbors of the new motion
        std::vector<Configuration *> nearestNbh;
        
        if (useKNearest_)
        {
            // calculate k
            unsigned int k = std::ceil(k_rrt_Constant_ * log((double) boost::num_vertices(graph_)));
            nearestDatastructure_->nearestK(xRandom_, k, nearestNbh);
        }
        else {
            double r = std::min(maxDistance_, 
                r_rrt_Constant_ * 
                std::pow(log((double) boost::num_vertices(graph_)) / (double) boost::num_vertices(graph_),
                    1 / static_cast<double>(Bundle->getStateDimension())));
            nearestDatastructure_->nearestR(xRandom_, r, nearestNbh);
        }

        // nearest neighbor cost
        ompl::base::Cost nn_line_cost = opt_->motionCost(q_nearest->state, xRandom_->state);
        ompl::base::Cost nn_cost = opt_->combineCosts(q_nearest->cost, nn_line_cost);
        
        // Find neighbor with minimum Cost
        Configuration *q_min = q_nearest;
        ompl::base::Cost min_line_cost = nn_line_cost;
        ompl::base::Cost min_cost = nn_cost;
        
        // if valid neighbors in first step than no need to check motion in rewire step for ith neighbor. valid values are {-1, 0, 1}
        int validNeighbor[nearestNbh.size()];

        // store the connection cost for later use, if space is symmetric
        std::vector<ompl::base::Cost> lineCosts;

        if (symmetric_)
        {
            lineCosts.resize(nearestNbh.size());
        }

        for(unsigned int i=0; i< nearestNbh.size(); i++)
        {
            Configuration* q_near = nearestNbh.at(i);

            // nearest neighbor
            if (q_nearest->index == q_near->index)
            {
                validNeighbor[i] = 1;
                if (symmetric_) {
                    lineCosts[i] = nn_line_cost;
                }
                continue;
            }
            validNeighbor[i] = 0;

            ompl::base::Cost line_cost = opt_->motionCost(q_near->state, xRandom_->state);
            ompl::base::Cost new_cost = opt_->combineCosts(q_near->cost, line_cost);
            
            if (symmetric_)
            {
                lineCosts[i] = line_cost;
            }

            if (opt_->isCostBetterThan(new_cost , min_cost))
            {
                if(distance(q_near, xRandom_) < maxDistance_ && Bundle->checkMotion(q_near->state, xRandom_->state))
                {
                    q_min = q_near;
                    min_line_cost = line_cost;
                    min_cost = new_cost;
                    validNeighbor[i] = 1;
                }
                else validNeighbor[i] = -1;
            }
        }

        // (4) Add sample
        Configuration *q_new = new Configuration(Bundle, xRandom_->state);
        Vertex v_next = addConfiguration(q_new);

        // (5) add edge assign cost
        this->addEdge(q_min->index, v_next, min_line_cost);
        q_new->lineCost = min_line_cost;
        q_new->cost = min_cost;
        q_new->parent = q_min->index;
        q_min->children.push_back(q_new->index);
        
        // (6) Rewire the tree
        for (unsigned int i=0 ; i< nearestNbh.size(); i++)
        {
            Configuration* q_near = nearestNbh.at(i);
            
            if (q_near->index != q_new->parent)
            {
                base::Cost line_cost;
                if(symmetric_) {
                    line_cost = lineCosts[i];
                }
                else {
                    line_cost = opt_->motionCost(q_new->state, q_near->state);
                }
                base::Cost new_cost = opt_->combineCosts(q_new->cost, line_cost);
                
                if (opt_->isCostBetterThan(new_cost, q_near->cost))
                {
                    bool valid = (validNeighbor[i] == 1);
                    // check neighbor validity if it wasn´t checked before
                    if (validNeighbor[i] == 0)
                    {
                        valid = (distance(q_near, q_new) < maxDistance_ && Bundle->checkMotion(q_near->state, q_new->state));
                    }
                    if (valid)
                    {
                        // remove node from children of its parent node
                        for (auto it = graph_[q_near->parent]->children.begin(); it != graph_[q_near->parent]->children.end(); ++it)
                        {
                            if (*it == q_near->index)
                            {
                                graph_[q_near->parent]->children.erase(it);
                                break;
                            }
                        }
                        // add with new parent
                        this->addEdge(q_new->index, q_near->index, line_cost);
                        
                        // update node parent
                        q_near->parent = q_new->index;
                        q_near->lineCost = line_cost;
                        q_near->cost = new_cost;
                        q_new->children.push_back(q_near->index);
                        
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
            bool satisfied = goal_->isSatisfied(q_new->state, &dist);
            if (satisfied)
            {
                vGoal_ = addConfiguration(qGoal_);
                BaseT::addEdge(q_nearest->index, vGoal_);
                hasSolution_ = true;
            }
        }
    }
}

void ompl::geometric::QRRTStarImpl::updateChildCosts(Configuration *q)
{
    for (std::size_t i = 0; i < q->children.size(); ++i)
    {
        graph_[q->children[i]]->cost = opt_->combineCosts(q->cost, graph_[q->children[i]]->lineCost);
        updateChildCosts(graph_[q->children[i]]);
    }
}

void ompl::geometric::QRRTStarImpl::addEdge(const Vertex a, const Vertex b, base::Cost weight)
{
    EdgeInternalState properties(weight);
    boost::add_edge(a, b, properties, graph_);
    uniteComponents(a, b);
}