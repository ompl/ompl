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
#include "ompl/util/GeometricEquations.h"
#include <memory>


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
    d_ = (double)getBundle()->getStateDimension();
    double e = boost::math::constants::e<double>();
    // k > 2^(d + 1) * e * (1 + 1 / d).
    k_rrt_Constant_ = std::pow(2, d_ + 1) * e * (1.0 + 1.0 / d_);
    // γRRG > γRRG ∗ = 2*( 1 + 1/d)^1/d * ( μ( Xfree) / ζd)^1/d
    r_rrt_Constant_ = std::pow(2 * (1.0 + 1.0 / d_) * (getBundle()->getSpaceMeasure() / unitNBallMeasure(d_)), 1.0 / d_);
    symmetric_ = getBundle()->getStateSpace()->hasSymmetricInterpolate();

    setImportance("greedy");
    setGraphSampler("randomvertex");
    setMetric("geodesic");
}

ompl::geometric::QRRTStarImpl::~QRRTStarImpl()
{
}

void ompl::geometric::QRRTStarImpl::clear()
{
    BaseT::clear();
    bestCost_ = opt_->infiniteCost();
    std::cout << "Clear " << getName() << std::endl;
}

void ompl::geometric::QRRTStarImpl::grow()
{
    if (firstRun_)
    {
        init();
        goal_ = pdef_->getGoal().get();
        firstRun_ = false;
    }

    //(1) Get Random Sample
    sampleBundleGoalBias(xRandom_->state, goalBias_);

    //(2) Get Nearest in Tree
    Configuration *q_nearest = nearestDatastructure_->nearest(xRandom_);

    //(3) Steer toward random
    // Configuration *q_new = steerTowards_Range(q_nearest, xRandom_);

    double d = distance(q_nearest, xRandom_);
    if (d > maxDistance_)
    {
        getBundle()->getStateSpace()->interpolate(
            q_nearest->state,
            xRandom_->state, 
            maxDistance_ / d,
        xRandom_->state);
    }

    if(getBundle()->checkMotion(q_nearest->state, xRandom_->state))
    {
        Configuration *q_new = new Configuration(getBundle(), xRandom_->state);

        // (1) Find all neighbors of the new configuration in graph
        std::vector<Configuration *> nearestNbh;
        getNearestNeighbors(q_new, nearestNbh);

        // (2) Find neighbor with minimum Cost
        q_new->lineCost = opt_->motionCost(q_nearest->state, q_new->state);
        q_new->cost = opt_->combineCosts(q_nearest->cost, q_new->lineCost);
        q_new->parent = q_nearest;

        // (3) Rewire Tree
        base::Cost cost_nearest = q_new->cost;
        
        int validNeighbor[nearestNbh.size()];

        // store the connection cost for later use, if space is symmetric
        std::vector<ompl::base::Cost> lineCosts;

        if (symmetric_)
        {
            lineCosts.resize(nearestNbh.size());
        }

        for(unsigned int i = 0; i < nearestNbh.size(); i++)
        {
            Configuration* q_near = nearestNbh.at(i);

            if(q_near == q_nearest)
            {
                validNeighbor[i] = 1;
                if (symmetric_) 
                {
                    lineCosts[i] = cost_nearest;
                }
                continue;
            }
            validNeighbor[i] = 0;

            ompl::base::Cost line_cost = opt_->motionCost(q_near->state, q_new->state);
            ompl::base::Cost new_cost = opt_->combineCosts(q_near->cost, line_cost);
            
            if (symmetric_)
            {
                lineCosts[i] = line_cost;
            }

            if (opt_->isCostBetterThan(new_cost, q_new->cost))
            {
                if((!useKNearest_ || distance(q_near, q_new) < maxDistance_) && getBundle()->checkMotion(q_near->state, q_new->state))
                {
                    q_new->lineCost = line_cost;
                    q_new->cost = new_cost;
                    q_new->parent = q_near;
                    validNeighbor[i] = 1;
                }
                else validNeighbor[i] = -1;
            }
        }
        //(4) Connect to minimum cost neighbor 
        addConfiguration(q_new);
        q_new->parent->children.push_back(q_new);
        addEdge(q_new->parent->index, q_new->index);


        bool checkForSolution = false;

        // (5) Rewire the tree (if from q_new to q_near is lower cost)
        for (unsigned int i=0; i < nearestNbh.size(); i++)
        {
            Configuration* q_near = nearestNbh.at(i);
            
            if (q_near != q_new->parent)
            {
                // (7a) compute cost q_new to q_near
                base::Cost line_cost;
                if(symmetric_) {
                    line_cost = lineCosts[i];
                }
                else {
                    line_cost = opt_->motionCost(q_new->state, q_near->state);
                }
                base::Cost new_cost = opt_->combineCosts(q_new->cost, line_cost);
                
                // (7b) check if new cost better than q_near->cost (over old
                // pathway)
                if (opt_->isCostBetterThan(new_cost, q_near->cost))
                {
                    bool valid = (validNeighbor[i] == 1);
                    // check neighbor validity if it wasn´t checked before
                    if (validNeighbor[i] == 0)
                    {
                        valid = ((!useKNearest_ || distance(q_near, q_new) < maxDistance_) && getBundle()->checkMotion(q_new->state, q_near->state));
                    }

                  // (7c) q_new to q_near is better way to reach q_near. Remove
                  // previous connection of q_near to old parent and set it to
                  // q_new
                  // pathway)
                    if (valid)
                    {
                        // (7d) remove q_near from children of its old parent node
                        removeFromParent(q_near);

                        // (7e) remove edge old parent to q_near
                        boost::remove_edge(q_near->parent->index, q_near->index, graph_);

                        // (7f) add new edge q_new to q_near
                        addEdge(q_new->index, q_near->index);

                        // (7g) update costs of q_near
                        q_near->lineCost = line_cost;
                        q_near->cost = new_cost;
                        q_near->parent = q_new;
                        q_near->parent->children.push_back(q_near);
                        
                        // (7h) update node's children costs
                        updateChildCosts(q_near);
                        checkForSolution = true;

                    }
                }
            }
        }

        // (8) check if this sample satisfies the goal
        double dist = 0.0;
        bool satisfied = goal_->isSatisfied(q_new->state, &dist);
        if (satisfied)
        {
            // if(goalConfigurations_.empty())
            // {
            //     vGoal_ = addConfiguration(qGoal_);
            //     addEdge(q_new->index, vGoal_);
            //     qGoal_->lineCost = opt_->motionCost(q_new->state, qGoal_->state);
            //     qGoal_->cost = opt_->combineCosts(q_new->cost, qGoal_->lineCost);
            //     qGoal_->parent = q_new;
            // }
            goalConfigurations_.push_back(q_new);
            checkForSolution = true;
        }

        if(checkForSolution)
        {
            bool updatedSolution = false;
            if(!goalConfigurations_.empty() && qGoal_ == nullptr)
            {
                qGoal_ = q_new;
                vGoal_ = qGoal_->index;
                bestCost_ = qGoal_->cost;
                updatedSolution = true;
            }else{
                for(uint k = 0; k < goalConfigurations_.size(); k++)
                {
                    Configuration *qk = goalConfigurations_.at(k);

                    if (opt_->isCostBetterThan(qk->cost, bestCost_))
                    {
                        // bestGoalConfiguration_ = qk;
                        // bestCost_ = bestGoalConfiguration_->cost;
                        qGoal_ = qk;
                        vGoal_ = qGoal_->index;
                        bestCost_ = qGoal_->cost;
                        updatedSolution = true;
                    }
                }
            }
            if(updatedSolution)
            {
                std::cout << "Found path with cost " << qGoal_->cost 
                  << " (level " << getLevel() << ")" << std::endl;

                // qGoal_->lineCost = opt_->motionCost(bestGoalConfiguration_->state, qGoal_->state);
                // qGoal_->cost = opt_->combineCosts(bestGoalConfiguration_->cost, qGoal_->lineCost);
                // qGoal_->parent = bestGoalConfiguration_;
                hasSolution_ = true;
            }
        }
    }
}

void ompl::geometric::QRRTStarImpl::updateChildCosts(Configuration *q)
{
    for (std::size_t i = 0; i < q->children.size(); ++i)
    {
        q->children.at(i)->cost = opt_->combineCosts(q->cost, q->children.at(i)->lineCost);
        updateChildCosts(q->children.at(i));
    }
}
void ompl::geometric::QRRTStarImpl::removeFromParent(Configuration *q)
{
    for (auto it = q->parent->children.begin(); it != q->parent->children.end(); ++it)
    {
        if (*it == q)
        {
            q->parent->children.erase(it);
            break;
        }
    }
}

bool ompl::geometric::QRRTStarImpl::getSolution(base::PathPtr &solution)
{
    if (hasSolution_)
    {
        solutionPath_ = std::make_shared<PathGeometric>(getBundle());

        Configuration *intermediate_node = qGoal_;
        int ctr = 0;
        while (intermediate_node != nullptr)
        {
            std::static_pointer_cast<PathGeometric>(solutionPath_)->append(intermediate_node->state);
            intermediate_node = intermediate_node->parent;

            //detect and handle loops
            if(ctr++ > 100){
              std::cout << "DETECTED LOOP" << std::endl;
              int idx = intermediate_node->index;
              for(uint k = 0; k < 10; k++)
              {
                if(intermediate_node->parent)
                {
                    std::cout << intermediate_node->index << " to " 
                      << intermediate_node->parent->index << std::endl;
                }else{
                  break;
                }
                intermediate_node = intermediate_node->parent;
                if(intermediate_node->index == idx){
                  break;
                }
              }
              break;
            }
        }
        std::static_pointer_cast<PathGeometric>(solutionPath_)->reverse();
        solution = solutionPath_;
        return true;
    }
    else
    {
      return false;
    }
}

void ompl::geometric::QRRTStarImpl::getPlannerData(base::PlannerData &data) const
{
    OMPL_DEBUG("Roadmap has %d vertices", nearestDatastructure_->size());
    BaseT::getPlannerData(data);

    // std::vector<Configuration *> motions;
    // if (nearestDatastructure_)
    //     nearestDatastructure_->list(motions);

    // if (bestGoalConfiguration_)
    //     data.addGoalVertex(base::PlannerDataVertex(bestGoalConfiguration_->state));

    // for (int i = 0; i < motions.size(); i++)
    // {
    //     if (motions[i]->parent == nullptr)
    //         data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
    //     else
    //         data.addEdge(base::PlannerDataVertex(motions[i]->parent->state), base::PlannerDataVertex(motions[i]->state));
    // }

    //data.addGoalVertex(base::PlannerDataVertex(qGoal_->state));
    //data.addStartVertex(base::PlannerDataVertex(qStart_->state));

    //addChildrenToPlannerData(qStart_, data);
}

void ompl::geometric::QRRTStarImpl::addChildrenToPlannerData(Configuration* q, base::PlannerData &data) const
{
    if (q != nullptr)
    {
        if (!q->children.empty())
        {
            /*for (std::size_t i = 0; i < q->children.size(); ++i)
            {
                // data.addEdge(base::PlannerDataVertex(q->state), base::PlannerDataVertex(q->children[i]->state));
                // addChildrenToPlannerData(q->children[i], data);
            }*/
        }
    }
}

void ompl::geometric::QRRTStarImpl::getNearestNeighbors(Configuration *x, std::vector<Configuration *> &nearest)
{
    auto cardDbl = static_cast<double>(nearestDatastructure_->size() + 1u);

    if (useKNearest_)
    {
        // calculate k
        unsigned int k = std::ceil(k_rrt_Constant_ * log(cardDbl));
        nearestDatastructure_->nearestK(x, k, nearest);
    }else
    {
        double r = std::min(maxDistance_, 
            r_rrt_Constant_ * 
            std::pow(log(cardDbl) / cardDbl, 1 / d_ ));
        nearestDatastructure_->nearestR(x, r, nearest);
    }
}
