/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Javier V. Gómez 
 * Adapted from geometric FMT version. */


#include <ompl/control/planners/fmt/FMT.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <boost/math/constants/constants.hpp>
#include <limits>

#include <fstream>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

ompl::control::FMT::FMT(const SpaceInformationPtr &si) 
    : base::Planner(si, "FMT")
    , numSamples_(1000)
    , radiusMultiplier_(1.1)
{
    siC_ = si.get();
    maxSampleAttempts_ = 10 * numSamples_;
    freeSpaceVolume_ = std::pow(si_->getMaximumExtent() / std::sqrt(si_->getStateDimension()), si_->getStateDimension());
    lastGoalMotion_ = NULL;

    specs_.approximateSolutions = false;
    specs_.directed = false;

    ompl::base::Planner::declareParam<unsigned int>("num_samples", this, &FMT::setNumSamples, &FMT::getNumSamples, "10:10:10000");
    ompl::base::Planner::declareParam<double>("radius_multiplier", this, &FMT::setRadiusMultiplier, &FMT::getRadiusMultiplier, "0.9:0.05:5.");
    ompl::base::Planner::declareParam<double>("free_space_volume", this, &FMT::setFreeSpaceVolume, &FMT::getFreeSpaceVolume, "1.:10:1000000.");
}

ompl::control::FMT::~FMT()
{
    freeMemory();
}

void ompl::control::FMT::setup()
{
    Planner::setup();

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

void ompl::control::FMT::clear()
{
    Planner::clear();
    lastGoalMotion_ = NULL;
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    H_.clear();
    hElements_.clear();
    neighborhoods_.clear();
}

void ompl::control::FMT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->getState())
                si_->freeState(motions[i]->getState());
            if (motions[i]->getControl())
                siC_->freeControl(motions[i]->getControl());

            delete motions[i];
        }
    }
}

void ompl::control::FMT::saveNeighborhood(Motion *m, const double r)
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
double ompl::control::FMT::calculateUnitBallVolume(const unsigned int dimension) const
{
    if (dimension == 0)
        return 1.0;
    else if (dimension == 1)
        return 2.0;
    return 2.0 * boost::math::constants::pi<double>() / dimension
            * calculateUnitBallVolume(dimension - 2);
}

double ompl::control::FMT::calculateRadius(const unsigned int dimension, const unsigned int n) const
{
    double a = 1.0 / (double)dimension;
    double unitBallVolume = calculateUnitBallVolume(dimension);

    return radiusMultiplier_ * 2.0 * std::pow(a, a) * std::pow(freeSpaceVolume_ / unitBallVolume, a) * std::pow(log((double)n) / (double)n, a);
}

void ompl::control::FMT::sampleFree(const base::PlannerTerminationCondition &ptc)
{
    unsigned int nodeCount = 0;
    unsigned int sampleAttempts = 0;
    Motion *motion = new Motion(siC_);

    // Sample numSamples_ number of nodes from the free configuration space
    while (nodeCount < numSamples_ && sampleAttempts < maxSampleAttempts_ && !ptc)
    {
        sampler_->sampleUniform(motion->getState());
        sampleAttempts++;

        bool collision_free = si_->isValid(motion->getState());

        if (collision_free)
        {
            nodeCount++;
            nn_->add(motion);
            motion = new Motion(siC_);
        } // If collision free
    } // While nodeCount < numSamples
    si_->freeState(motion->getState());
    siC_->freeControl(motion->getControl());
    delete motion;
}

void ompl::control::FMT::assureGoalIsSampled(const ompl::base::GoalSampleableRegion *goal)
{
    // Ensure that there is at least one node near each goal
    while (const base::State *goalState = pis_.nextGoal())
    {
        Motion *gMotion = new Motion(siC_);
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
                siC_->freeControl(gMotion->getControl());
                delete gMotion;
            }
        }
        else // There is already a sample in the goal region
        {
            si_->freeState(gMotion->getState());
            siC_->freeControl(gMotion->getControl());
            delete gMotion;
        }
    } // For each goal
}

ompl::base::PlannerStatus ompl::control::FMT::solve(const base::PlannerTerminationCondition &ptc)
{
    if (lastGoalMotion_) {
        OMPL_INFORM("solve() called before clear(); returning previous solution");
        traceSolutionPathThroughTree(lastGoalMotion_);
        OMPL_DEBUG("Final path cost: %f", lastGoalMotion_->getCost().v);
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

    while (const base::State *st = pis_.nextStart())
    {
        initMotion = new Motion(siC_);
        si_->copyState(initMotion->getState(), st);
        siC_->nullControl(initMotion->getControl());
        hElements_[initMotion] = H_.insert(initMotion);
        initMotion->setSetType(Motion::SET_H);
        initMotion->setCost(opt_->initialCost(initMotion->getState()));
        nn_->add(initMotion); // V <-- {x_init}
    }

    if (nn_->size() == 0 || !initMotion)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Sample N free states in the configuration space
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    sampleFree(ptc);
    assureGoalIsSampled(goal);

    saveInitialTree();

    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    // Calculate the nearest neighbor search radius
    // TODO: check the kinoFMT paper the real radius.
    double r = calculateRadius(si_->getStateDimension(), nn_->size());
    OMPL_DEBUG("Using radius of %f", r);

    // Flag all nodes as in set W
    std::vector<Motion*> vNodes;
    //vNodes.reserve(nn_->size()); // list() alread reserves the size.
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

        OMPL_DEBUG("Final path cost: %f", lastGoalMotion_->getCost().v);
        return base::PlannerStatus(true, false);
    } // if plannerSuccess
    else
    {
        // Planner terminated without accomplishing goal
        return base::PlannerStatus(false, false);
    }
}

void ompl::control::FMT::traceSolutionPathThroughTree(Motion *goalMotion)
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
    PathControl *path = new PathControl(si_);
    for (int i = mpath.size() - 1 ; i >= 0 ; --i)
        if (mpath[i]->getParent())
            path->append(mpath[i]->getState(), mpath[i]->getControl(), mpath[i]->getSteps() * siC_->getPropagationStepSize());
        else
            path->append(mpath[i]->getState());

    pdef_->addSolutionPath(base::PathPtr(path), false, lastGoalMotion_->getCost().v, getName());
}


void ompl::control::FMT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->getState()));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        const Motion *m = motions[i];
        if (m->getParent())
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->getParent()->getState()),
                             base::PlannerDataVertex(m->getState()),
                             control::PlannerDataEdgeControl(m->getControl(), m->getSteps() * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->getParent()->getState()),
                             base::PlannerDataVertex(m->getState()));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->getState()));
    }
}

bool ompl::control::FMT::expandTreeFromNode(Motion *&z, const double r)
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
            // See if those 2 states can be connected with sampled controls.
            Control *steer_ctrl = siC_->allocControl(); // Take these out of the loop.
            base::State *steer_state = si_->cloneState(x->getState());
            int cd = controlSampler_->sampleTo(steer_ctrl, yMin->getState(), steer_state);

            //if (cd > 0 && si_->distance(steer_state, x->getState()) < 0.3)
            if (cd > 0)
            {
                /*si_->printState(yMin->getState());
                si_->printState(x->getState());
                si_->printState(steer_state);
                std::cout << si_->distance(yMin->getState(), x->getState()) << std::endl;
                std::cout << "---" << si_->distance(steer_state, x->getState()) << std::endl;*/

                // Add edge from yMin to x
                si_->copyState(x->getState(), steer_state);
                siC_->copyControl(x->getControl(), steer_ctrl);
                x->setParent(yMin);
                x->setCost(cMin);
                x->setSteps(cd);
                // Add x to H_new
                H_new.push_back(x);
                // Remove x from W
                x->setSetType(Motion::SET_NULL); 
            }
            si_->freeState(steer_state);
            siC_->freeControl(steer_ctrl);


            //bool collision_free = si_->checkMotion(yMin->getState(), x->getState());
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


void ompl::control::FMT::saveTree()
{
    const char *filename = "fmtree.txt";
    OMPL_INFORM("Saving into %s", filename);

    std::vector<Motion*> tree;
    nn_->list(tree);

    std::fstream fs;
    fs.open (filename, std::fstream::out | std::fstream::trunc);

    fs << tree.size() << "\t" << siC_->getPropagationStepSize() << "\t";

    for (size_t i = 0; i < 19; ++i)
      fs << 0 << "\t";
    fs << std::endl;

    Motion *root = tree[0];
    while (root->getParent() != 0)
        root = root->getParent();

    // First node has no parent.
    fs << root->getState()->as<base::SE2StateSpace::StateType>()->getX() << "\t"
       << root->getState()->as<base::SE2StateSpace::StateType>()->getY() << "\t"
       << root->getState()->as<base::SE2StateSpace::StateType>()->getYaw() << "\t";

    for (size_t i = 0; i < 18; ++i)
        fs << 0 << "\t";
    fs << std::endl;

    for (size_t i = 0; i < tree.size(); ++i) 
    {
        if (tree[i] != root && tree[i]->getSetType() == Motion::SET_NULL) 
        {
            fs << tree[i]->getState()->as<base::SE2StateSpace::StateType>()->getX() << "\t"
               << tree[i]->getState()->as<base::SE2StateSpace::StateType>()->getY() << "\t"
               << tree[i]->getState()->as<base::SE2StateSpace::StateType>()->getYaw() << "\t"
               << tree[i]->getParent()->getState()->as<base::SE2StateSpace::StateType>()->getX() << "\t"
               << tree[i]->getParent()->getState()->as<base::SE2StateSpace::StateType>()->getY() << "\t"
               << tree[i]->getParent()->getState()->as<base::SE2StateSpace::StateType>()->getYaw() << "\t";

            size_t j;
            for(j = 0; j < tree[i]->getControl()->actions.size(); ++j)
            {
                fs << tree[i]->getControl()->actions[j].first->as<RealVectorControlSpace::ControlType>()->values[0] << "\t"
                   << tree[i]->getControl()->actions[j].first->as<RealVectorControlSpace::ControlType>()->values[1] << "\t"
                   << tree[i]->getControl()->actions[j].second << "\t";
            }

            for(; j < 5; ++j)
            {
                fs << 0 << "\t"
                   << 0 << "\t"
                   << 0 << "\t";
            }

            fs << std::endl;
        }
    }

    fs.close();
}


void ompl::control::FMT::saveInitialTree()
{
    const char *filename = "fmtree_initial.txt";
    OMPL_INFORM("Saving inital tree into %s", filename);

    std::vector<Motion*> tree;
    nn_->list(tree);

    std::fstream fs;
    fs.open (filename, std::fstream::out | std::fstream::trunc);

    fs << tree.size() << "\t" << siC_->getPropagationStepSize() << "\t" << 0 << std::endl;

    for (size_t i = 0; i < tree.size(); ++i) 
    {
        fs << tree[i]->getState()->as<base::SE2StateSpace::StateType>()->getX() << "\t"
           << tree[i]->getState()->as<base::SE2StateSpace::StateType>()->getY() << "\t"
           << tree[i]->getState()->as<base::SE2StateSpace::StateType>()->getYaw() << std::endl;
    }

    fs.close();
}

