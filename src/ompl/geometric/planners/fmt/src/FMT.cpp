/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Autonomous Systems Laboratory, Stanford University
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
*   * Neither the name of Stanford University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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

/* Authors: Ashley Clark (Stanford) and Wolfgang Pointner (AIT) */
/* Co-developers: Brice Rebsamen (Stanford) and Tim Wheeler (Stanford) */
/* Algorithm design: Lucas Janson (Stanford) and Marco Pavone (Stanford) */
/* Acknowledgements for insightful comments: Edward Schmerling (Stanford), Oren Salzman (Tel Aviv University), and Joseph Starek (Stanford) */

#include <limits>
#include <iostream>

#include <boost/math/constants/constants.hpp>

#include <ompl/datastructures/BinaryHeap.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/tools/config/SelfConfig.h>
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

#include "FMT.h"


ompl::geometric::FMT::FMT(const base::SpaceInformationPtr &si)
    : base::Planner(si, "FMT")
    , numSamples_(1000)
    , obstacleCoverage_(0.0)
    , radiusMultiplier_(1.1)
{
    specs_.approximateSolutions = false;
    specs_.directed = false;

    ompl::base::Planner::declareParam<unsigned int>("num_samples", this, &FMT::setNumSamples, &FMT::getNumSamples, ">0");
    ompl::base::Planner::declareParam<double>("radius_multiplier", this, &FMT::setRadiusMultiplier, &FMT::getRadiusMultiplier, ">1.0");
    ompl::base::Planner::declareParam<double>("obstacle_coverage", this, &FMT::setObstacleCoverage, &FMT::getObstacleCoverage, "0.:.05:1.");
}

ompl::geometric::FMT::~FMT(void)
{
    freeMemory();
}

void ompl::geometric::FMT::setup(void)
{
    ompl::base::Planner::setup();

    if (!nn_)
        nn_.reset(new NearestNeighborsGNAT<Motion*>());
    nn_->setDistanceFunction(boost::bind(&FMT::distanceFunction, this, _1, _2));

    /* Setup the optimization objective. If no optimization objective was 
       specified, then default to optimizing path length as computed by the 
       distance() function in the state space */
    if ( pdef_->hasOptimizationObjective() )
        opt_ = pdef_->getOptimizationObjective();
    else
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.", getName().c_str());
        opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
}

void ompl::geometric::FMT::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        motions.reserve(nn_->size());
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            si_->freeState(motions[i]->getState());
            delete motions[i];
        }
    }
}

void ompl::geometric::FMT::clear(void)
{
    ompl::base::Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::geometric::FMT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    std::vector<Motion*> motions;
    nn_->list(motions);

    unsigned int size = motions.size();
    for(unsigned int k = 0; k < size; ++k)
    {
        Motion* motion = motions[k];
        data.addEdge (base::PlannerDataVertex (motion->getParent() ? motion->getParent()->getState() : NULL), base::PlannerDataVertex (motion->getState()));
    }
}

void ompl::geometric::FMT::saveNeighborhood(Motion* m, const double r)
{
    // Check to see if neighborhood has not been saved yet
    if (neighborhoods_.find(m) == neighborhoods_.end())
    {
        std::vector<Motion*> nbh;
        nn_->nearestR(m, r, nbh);
        if (!nbh.empty())
        {
            // Save the neighborhood but skip the first element, since it will be motion m
            neighborhoods_[m] = std::vector<Motion*>(nbh.size()-1, 0);
            std::copy(nbh.begin()+1, nbh.end(), neighborhoods_[m].begin());
        }
        else
        {
            // Save an empty neighborhood
            neighborhoods_[m] = std::vector<Motion*>(0);
        }
    } // If neighborhood hadn't been saved yet
}

// Calculate the unit ball volume for a given dimension
double ompl::geometric::FMT::calculateUnitBallVolume(const unsigned int dimension) const
{
    if ( dimension == 0 )
        return 1.0;
    else if( dimension == 1 )
        return 2.0;
    return 2.0 * boost::math::constants::pi<double>() / dimension
            * calculateUnitBallVolume(dimension-2);
}

double ompl::geometric::FMT::calculateRadius(const unsigned int dimension, const unsigned int n) const
{
    double a = 1.0/(double)dimension;
    double lebesqueMeasure = 1.0 - obstacleCoverage_;
    double unitBallVolume = calculateUnitBallVolume(dimension);

    return  radiusMultiplier_ * 2.0 * pow(1.0+a,a) * pow(lebesqueMeasure/unitBallVolume, a) * pow(log((double)n)/(double)n,a);
}

void ompl::geometric::FMT::sampleFree(const base::PlannerTerminationCondition &ptc)
{
    unsigned int nodeCount = 0;
    Motion *motion = new Motion(opt_,si_);
 
    // Sample numSamples_ number of nodes from the free configuration space
    while(nodeCount < numSamples_ && !ptc)
    {
        sampler_->sampleUniform(motion->getState());

        bool collision_free = si_->isValid(motion->getState());

        if(collision_free)
        {
            nodeCount++;
            nn_->add(motion);
            motion = new Motion(opt_,si_);
        } // If collision free
    } // While nodeCount < numSamples
    si_->freeState(motion->getState());
    delete motion;
}

void ompl::geometric::FMT::assureGoalIsSampled(const ompl::base::Goal *goal)
{
    const base::GoalSampleableRegion *goal_s = dynamic_cast<const base::GoalSampleableRegion *>(goal);
    
    Motion *gMotion;
    if(goal_s && goal_s->canSample())
    {
        std::vector<Motion*> nearGoal;
        int numGoals = goal_s->maxSampleCount();
        
        // Ensure that there is at least one node near each goal
        for(int i=0; i<numGoals; ++i)
        {
            gMotion = new Motion(opt_,si_);
            goal_s->sampleGoal(gMotion->getState());
            nn_->nearestR(gMotion, goal_s->getThreshold(), nearGoal);
            
            // If there is no node in the goal region, insert one
            if(nearGoal.empty())
            {
                OMPL_DEBUG("No state inside goal region");
                if(si_->getStateValidityChecker()->isValid(gMotion->getState()))
                {
                    gMotion->setSetType(Motion::SET_W);
                    nn_->add(gMotion);
                }
                else
                {
                    delete gMotion;
                }
            }
            else
            {
                si_->freeState(gMotion->getState());
                delete gMotion;
            }
        } // For each goal
    } // If there is a sampleable goal region
}

ompl::base::PlannerStatus ompl::geometric::FMT::solve(const base::PlannerTerminationCondition& ptc)
{
    checkValidity();
    base::Goal                     *goal   = pdef_->getGoal().get();
    
    if (!goal)
    {
        OMPL_ERROR("Goal undefined");
        return base::PlannerStatus::INVALID_GOAL;
    }

    Motion *z;
    Motion *initMotion;
    
    // Add start states to V (nn_) and H
    while (const base::State *st = pis_.nextStart())
    {
        initMotion = new Motion(opt_,si_);
        si_->copyState(initMotion->getState(), st);
        hElements_[initMotion] = H_.insert(initMotion);
        initMotion->setSetType(Motion::SET_H);
        initMotion->setCost(opt_->initialCost(initMotion->getState()));
        nn_->add(initMotion);                           // V <-- {x_init}
    }
    z = initMotion;                                     // z <-- xinit
        
    // Sample N free states in the configuration space
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    sampleFree(ptc);
    assureGoalIsSampled(goal);
    OMPL_INFORM("%s: Starting with %u states", getName().c_str(), numSamples_);

    // Calculate the nearest neighbor search radius
    double r = calculateRadius(si_->getStateDimension(), nn_->size());
    OMPL_DEBUG("Using radius of %f", r);

    // Flag all nodes as in set W
    std::vector<Motion*> vNodes;
    vNodes.reserve(nn_->size());
    nn_->list(vNodes);
    unsigned int vNodesSize = vNodes.size();
    for(unsigned int i = 0; i < vNodesSize; ++i)
    {
        vNodes[i]->setSetType(Motion::SET_W);
    }
    vNodes.clear();
    
    // Execute the planner, and return early if the planner returns a failure
    bool plannerSuccess = false;
    bool successfulExpansion = false;
    z->setSetType(Motion::SET_H);
    saveNeighborhood(z, r);
    
    while(!ptc && !(plannerSuccess = goal->isSatisfied(z->getState())))
    {
        successfulExpansion = expandTreeFromNode(z, r);
        if(!successfulExpansion) 
            return base::PlannerStatus(false,false);
    } // While not at goal

    if(!ptc)
    {
        // Return the path to z, since if we have reached this point in the code, 
        // then the planner successfully found a solution and z is in the goal region
        std::vector<Motion*> mpath;
        Motion* solution = z;

        // Construct the solution path
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->getParent();
        }

        // Set the solution path
        PathGeometric *path = new PathGeometric(si_);
        int mPathSize = mpath.size();
        for (int i = mPathSize - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->getState());
        pdef_->addSolutionPath(base::PathPtr(path), false, z->getCost().v);

        OMPL_DEBUG("Final path cost: %f\n", z->getCost().v);
        return base::PlannerStatus(true, false);
    } // if !ptc
    else
    {
        // Planner terminated without accomplishing goal
        return base::PlannerStatus(false, false);
    }
}

bool ompl::geometric::FMT::expandTreeFromNode(Motion *&z, const double r)
{
    std::vector<Motion*> xNear, yNear, H_new;
        
    // Find all nodes that are near z, and also in set W
    
    const std::vector<Motion*> &zNeighborhood = neighborhoods_[z];
    unsigned int zNeighborhoodSize = zNeighborhood.size();
    xNear.reserve(zNeighborhoodSize);

    for(unsigned int i = 0; i < zNeighborhoodSize; ++i)
    {
        if(zNeighborhood[i]->getSetType() == Motion::SET_W)
            xNear.push_back(zNeighborhood[i]);
    }

    // For each node near z and in set W, attempt to connect it to set H
    Motion *x;
    unsigned int xNearSize = xNear.size();
    for (unsigned int i = 0 ; i < xNearSize; ++i)
    {
        x = xNear.at(i);

        // Find all nodes that are near x and in set H
        saveNeighborhood(x,r);
        const std::vector<Motion*> &xNeighborhood = neighborhoods_[x];

        unsigned int xNeighborhoodSize = xNeighborhood.size();
        yNear.reserve(xNeighborhoodSize);
        for(unsigned int j = 0; j < xNeighborhoodSize; ++j)
        {
            if(xNeighborhood[j]->getSetType() == Motion::SET_H)
                yNear.push_back(xNeighborhood[j]);
        }

        // Find the lowest cost-to-come connection from H to x
        Motion* yMin = NULL;
        base::Cost cMin(std::numeric_limits<double>::infinity());
        base::Cost cNew;
        base::State* yState;

        unsigned int yNearSize = yNear.size();
        for (unsigned int j = 0; j < yNearSize; ++j)
        {
            yState = yNear.at(j)->getState();
            base::Cost dist = opt_->motionCost(yState, x->getState());
            cNew = opt_->combineCosts(yNear.at(j)->getCost(), dist);

            if (opt_->isCostBetterThan(cNew, cMin))
            {
                yMin = yNear.at(j);
                cMin = cNew;
            }
        }

        // If an optimal connection from H to x was found
        if (yMin != NULL) 
        {
            bool collision_free = si_->checkMotion(yMin->getState(), x->getState()); 

            if (collision_free)
            {
                // Add edge from yMin to x
                x->setParent(yMin);
                x->setCost(cMin);
                yMin->getChildren()->push_back(x);
                // Add x to H_new
                H_new.push_back(x);	
                // Remove x from W
                x->setSetType(Motion::SET_NULL);
            }
        } // An optimal connection from H to x was found
    } // For each node near z and in set W, try to connect it to set H

    // Remove motion z from the binary heap and from the map
    MotionBinHeap::Element* zElement = hElements_[z];
    H_.remove(zElement);
    hElements_.erase(z);
    z->setSetType(Motion::SET_NULL);

    // Add the nodes in H_new to H
    unsigned int hNewSize = H_new.size();
    for (unsigned int i = 0; i < hNewSize; i++) 
    {
        hElements_[H_new.at(i)] = H_.insert(H_new.at(i));
        H_new.at(i)->setSetType(Motion::SET_H);
    }

    H_new.clear();

    if(H_.empty())
    {
        OMPL_INFORM("H is empty before path was found --> no feasible path exists");
        return false;
    }

    // Take the top of H as the new z
    z = H_.top()->data;

    return true;
}