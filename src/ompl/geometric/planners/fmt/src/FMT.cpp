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
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/fmt/FMT.h>


ompl::geometric::FMT::FMT(const base::SpaceInformationPtr &si)
    : base::Planner(si, "FMT")
    , numSamples_(1000)
    , radiusMultiplier_(1.1)
{
    freeSpaceVolume_ = std::pow(si_->getMaximumExtent() / std::sqrt(si_->getStateDimension()), (int)si_->getStateDimension());
    lastGoalMotion_ = NULL;

    specs_.approximateSolutions = false;
    specs_.directed = false;

    ompl::base::Planner::declareParam<unsigned int>("num_samples", this, &FMT::setNumSamples, &FMT::getNumSamples, "10:10:10000");
    ompl::base::Planner::declareParam<double>("radius_multiplier", this, &FMT::setRadiusMultiplier, &FMT::getRadiusMultiplier, "0.9:0.05:5.");
    ompl::base::Planner::declareParam<double>("free_space_volume", this, &FMT::setFreeSpaceVolume, &FMT::getFreeSpaceVolume, "1.:10:1000000.");
}

ompl::geometric::FMT::~FMT()
{
    freeMemory();
}

void ompl::geometric::FMT::setup()
{
    Planner::setup();

    /* Setup the optimization objective. If no optimization objective was
       specified, then default to optimizing path length as computed by the
       distance() function in the state space */
    if (pdef_->hasOptimizationObjective())
        opt_ = pdef_->getOptimizationObjective();
    else
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.", getName().c_str());
        opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
    H_.getComparisonOperator().opt_ = opt_.get();

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&FMT::distanceFunction, this, _1, _2));
}

void ompl::geometric::FMT::freeMemory()
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

void ompl::geometric::FMT::clear()
{
    Planner::clear();
    lastGoalMotion_ = NULL;
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    H_.clear();
    hElements_.clear();
    neighborhoods_.clear();
}

void ompl::geometric::FMT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    std::vector<Motion*> motions;
    nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->getState()));

    unsigned int size = motions.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        if (motions[i]->getParent() == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->getState()));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->getParent()->getState()),
                         base::PlannerDataVertex(motions[i]->getState()));
    }
}

void ompl::geometric::FMT::saveNeighborhood(Motion *m, const double r)
{
    // Check to see if neighborhood has not been saved yet
    if (neighborhoods_.find(m) == neighborhoods_.end())
    {
        std::vector<Motion*> nbh;
        nn_->nearestR(m, r, nbh);
        if (!nbh.empty())
        {
            // Save the neighborhood but skip the first element, since it will be motion m
            neighborhoods_[m] = std::vector<Motion*>(nbh.size() - 1, 0);
            std::copy(nbh.begin() + 1, nbh.end(), neighborhoods_[m].begin());
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
    if (dimension == 0)
        return 1.0;
    else if (dimension == 1)
        return 2.0;
    return 2.0 * boost::math::constants::pi<double>() / dimension
            * calculateUnitBallVolume(dimension - 2);
}

double ompl::geometric::FMT::calculateRadius(const unsigned int dimension, const unsigned int n) const
{
    double a = 1.0 / (double)dimension;
    double unitBallVolume = calculateUnitBallVolume(dimension);

    return radiusMultiplier_ * 2.0 * std::pow(a, a) * std::pow(freeSpaceVolume_ / unitBallVolume, a) * std::pow(log((double)n) / (double)n, a);
}

void ompl::geometric::FMT::sampleFree(const base::PlannerTerminationCondition &ptc)
{
    unsigned int nodeCount = 0;
    Motion *motion = new Motion(si_);

    // Sample numSamples_ number of nodes from the free configuration space
    while (nodeCount < numSamples_ && !ptc)
    {
        sampler_->sampleUniform(motion->getState());

        bool collision_free = si_->isValid(motion->getState());

        if (collision_free)
        {
            nodeCount++;
            nn_->add(motion);
            motion = new Motion(si_);
        } // If collision free
    } // While nodeCount < numSamples
    si_->freeState(motion->getState());
    delete motion;
}

void ompl::geometric::FMT::assureGoalIsSampled(const ompl::base::GoalSampleableRegion *goal)
{
    // Ensure that there is at least one node near each goal
    while (const base::State *goalState = pis_.nextGoal())
    {
        Motion *gMotion = new Motion(si_);
        si_->copyState(gMotion->getState(), goalState);

        std::vector<Motion*> nearGoal;
        nn_->nearestR(gMotion, goal->getThreshold(), nearGoal);

        // If there is no node in the goal region, insert one
        if (nearGoal.empty())
        {
            OMPL_DEBUG("No state inside goal region");
            if (si_->getStateValidityChecker()->isValid(gMotion->getState()))
            {
                gMotion->setSetType(Motion::SET_W);
                nn_->add(gMotion);
            }
            else
            {
                si_->freeState(gMotion->getState());
                delete gMotion;
            }
        }
        else // There is already a sample in the goal region
        {
            si_->freeState(gMotion->getState());
            delete gMotion;
        }
    } // For each goal
}

ompl::base::PlannerStatus ompl::geometric::FMT::solve(const base::PlannerTerminationCondition &ptc)
{
    if (lastGoalMotion_) {
        OMPL_INFORM("solve() called before clear(); returning previous solution");
        traceSolutionPathThroughTree(lastGoalMotion_);
        OMPL_DEBUG("Final path cost: %f", lastGoalMotion_->getCost().value());
        return base::PlannerStatus(true, false);
    }
    else if (hElements_.size() > 0)
    {
        OMPL_INFORM("solve() called before clear(); no previous solution so starting afresh");
        clear();
    }

    checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
    Motion *initMotion = NULL;

    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add start states to V (nn_) and H
    while (const base::State *st = pis_.nextStart())
    {
        initMotion = new Motion(si_);
        si_->copyState(initMotion->getState(), st);
        hElements_[initMotion] = H_.insert(initMotion);
        initMotion->setSetType(Motion::SET_H);
        initMotion->setCost(opt_->initialCost(initMotion->getState()));
        nn_->add(initMotion); // V <-- {x_init}
    }

    if (!initMotion)
    {
        OMPL_ERROR("Start state undefined");
        return base::PlannerStatus::INVALID_START;
    }

    // Sample N free states in the configuration space
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    sampleFree(ptc);
    assureGoalIsSampled(goal);
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    // Calculate the nearest neighbor search radius
    double r = calculateRadius(si_->getStateDimension(), nn_->size());
    OMPL_DEBUG("Using radius of %f", r);

    // Flag all nodes as in set W
    std::vector<Motion*> vNodes;
    vNodes.reserve(nn_->size());
    nn_->list(vNodes);
    unsigned int vNodesSize = vNodes.size();
    for (unsigned int i = 0; i < vNodesSize; ++i)
    {
        vNodes[i]->setSetType(Motion::SET_W);
    }

    // Execute the planner, and return early if the planner returns a failure
    bool plannerSuccess = false;
    bool successfulExpansion = false;
    Motion *z = initMotion; // z <-- xinit
    z->setSetType(Motion::SET_H);
    saveNeighborhood(z, r);

    while (!ptc && !(plannerSuccess = goal->isSatisfied(z->getState())))
    {
        successfulExpansion = expandTreeFromNode(z, r);
        if (!successfulExpansion)
            return base::PlannerStatus(false, false);
    } // While not at goal

    if (plannerSuccess)
    {
        // Return the path to z, since by definition of planner success, z is in the goal region
        lastGoalMotion_ = z;
        traceSolutionPathThroughTree(lastGoalMotion_);

        OMPL_DEBUG("Final path cost: %f", lastGoalMotion_->getCost().value());
        return base::PlannerStatus(true, false);
    } // if plannerSuccess
    else
    {
        // Planner terminated without accomplishing goal
        return base::PlannerStatus(false, false);
    }
}

void ompl::geometric::FMT::traceSolutionPathThroughTree(Motion *goalMotion)
{
    std::vector<Motion*> mpath;
    Motion *solution = goalMotion;

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
    pdef_->addSolutionPath(base::PathPtr(path), false, -1.0, getName());
}

bool ompl::geometric::FMT::expandTreeFromNode(Motion *&z, const double r)
{
    // Find all nodes that are near z, and also in set W

    std::vector<Motion*> xNear;
    const std::vector<Motion*> &zNeighborhood = neighborhoods_[z];
    unsigned int zNeighborhoodSize = zNeighborhood.size();
    xNear.reserve(zNeighborhoodSize);

    for (unsigned int i = 0; i < zNeighborhoodSize; ++i)
    {
        if (zNeighborhood[i]->getSetType() == Motion::SET_W)
            xNear.push_back(zNeighborhood[i]);
    }

    // For each node near z and in set W, attempt to connect it to set H
    std::vector<Motion*> yNear;
    std::vector<Motion*> H_new;
    unsigned int xNearSize = xNear.size();
    for (unsigned int i = 0 ; i < xNearSize; ++i)
    {
        Motion *x = xNear[i];

        // Find all nodes that are near x and in set H
        saveNeighborhood(x,r);
        const std::vector<Motion*> &xNeighborhood = neighborhoods_[x];

        unsigned int xNeighborhoodSize = xNeighborhood.size();
        yNear.reserve(xNeighborhoodSize);
        for (unsigned int j = 0; j < xNeighborhoodSize; ++j)
        {
            if (xNeighborhood[j]->getSetType() == Motion::SET_H)
                yNear.push_back(xNeighborhood[j]);
        }

        // Find the lowest cost-to-come connection from H to x
        Motion *yMin = NULL;
        base::Cost cMin(std::numeric_limits<double>::infinity());
        unsigned int yNearSize = yNear.size();
        for (unsigned int j = 0; j < yNearSize; ++j)
        {
            base::State *yState = yNear[j]->getState();
            base::Cost dist = opt_->motionCost(yState, x->getState());
            base::Cost cNew = opt_->combineCosts(yNear[j]->getCost(), dist);

            if (opt_->isCostBetterThan(cNew, cMin))
            {
                yMin = yNear[j];
                cMin = cNew;
            }
        }
        yNear.clear();

        // If an optimal connection from H to x was found
        if (yMin != NULL)
        {
            bool collision_free = si_->checkMotion(yMin->getState(), x->getState());

            if (collision_free)
            {
                // Add edge from yMin to x
                x->setParent(yMin);
                x->setCost(cMin);
                // Add x to H_new
                H_new.push_back(x);
                // Remove x from W
                x->setSetType(Motion::SET_NULL);
            }
        } // An optimal connection from H to x was found
    } // For each node near z and in set W, try to connect it to set H

    // Remove motion z from the binary heap and from the map
    H_.remove(hElements_[z]);
    hElements_.erase(z);
    z->setSetType(Motion::SET_NULL);

    // Add the nodes in H_new to H
    unsigned int hNewSize = H_new.size();
    for (unsigned int i = 0; i < hNewSize; i++)
    {
        hElements_[H_new[i]] = H_.insert(H_new[i]);
        H_new[i]->setSetType(Motion::SET_H);
    }

    H_new.clear();

    if (H_.empty())
    {
        OMPL_INFORM("H is empty before path was found --> no feasible path exists");
        return false;
    }

    // Take the top of H as the new z
    z = H_.top()->data;

    return true;
}
