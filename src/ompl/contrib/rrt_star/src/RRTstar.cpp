/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Ioan Sucan */

#include "ompl/contrib/rrt_star/RRTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include <algorithm>
#include <limits>
#include <map>
#include <boost/math/constants/constants.hpp>

ompl::geometric::RRTstar::RRTstar(const base::SpaceInformationPtr &si) : base::Planner(si, "RRTstar")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    delayCC_ = true;
    iterations_ = 0;
    lastGoalMotion_ = NULL;

    Planner::declareParam<double>("range", this, &RRTstar::setRange, &RRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTstar::setGoalBias, &RRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTstar::setDelayCC, &RRTstar::getDelayCC, "0,1");
}

ompl::geometric::RRTstar::~RRTstar(void)
{
    freeMemory();
}

void ompl::geometric::RRTstar::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _1, _2));
}

void ompl::geometric::RRTstar::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    iterations_ = 0;
    lastGoalMotion_ = NULL;
    goalMotions_.clear();
}

ompl::base::PlannerStatus ompl::geometric::RRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                  *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    base::OptimizationObjective *opt    = pdef_->getOptimizationObjective().get();

    // when no optimization objective is specified, we create a temporary one (we should not modify the ProblemDefinition)
    boost::scoped_ptr<base::OptimizationObjective> temporaryOptimizationObjective;

    if (opt && !dynamic_cast<base::PathLengthOptimizationObjective*>(opt))
    {
        opt = NULL;
        OMPL_WARN("Optimization objective '%s' specified, but such an objective is not appropriate for %s. Only path length can be optimized.", getName().c_str(), opt->getDescription().c_str());
    }

    if (!opt)
    {
        // by default, optimize path length and run until completion
        opt = new base::PathLengthOptimizationObjective(si_, std::numeric_limits<double>::epsilon());
        temporaryOptimizationObjective.reset(opt);
        OMPL_INFORM("No optimization objective specified. Defaulting to optimization of path length for the allowed planning time.");
    }

    if (!goal)
    {
        OMPL_ERROR("Goal undefined");
        return base::PlannerStatus::INVALID_GOAL;
    }

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("Starting with %u states", nn_->size());

    Motion *solution       = lastGoalMotion_;
    Motion *approximation  = NULL;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    Motion *rmotion        = new Motion(si_);
    base::State *rstate    = rmotion->state;
    base::State *xstate    = si_->allocState();

    // e+e/d.  K-nearest RRT*
    double k_rrg           = boost::math::constants::e<double>() + (boost::math::constants::e<double>()/(double)si_->getStateSpace()->getDimension());

    std::vector<Motion*> nbh;
    std::vector<double>  dists;
    std::vector<int>     valid;
    unsigned int         rewireTest = 0;
    unsigned int         statesGenerated = 0;

    if(solution)
        OMPL_INFORM("Starting with existing solution of cost %.5f", solution->cost);
    OMPL_INFORM("Initial k-nearest value of %u", (unsigned int)std::ceil(k_rrg * log(nn_->size()+1)));

    while (ptc == false)
    {
        iterations_++;
        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        base::State *dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        // Check if the motion between the nearest state and the state to add is valid
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // Duplicate the sampled motion
            double distN = si_->distance(dstate, nmotion->state);
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);

            // Find nearby neighbors of the new motion - k-nearest RRT*
            unsigned int k = std::ceil(k_rrg * log(nn_->size()+1));
            nn_->nearestK(motion, k, nbh);
            rewireTest += nbh.size();
            statesGenerated++;

            // cache for distance computations
            dists.resize(nbh.size());
            // cache for motion validity
            valid.resize(nbh.size());
            std::fill(valid.begin(), valid.end(), 0);

            // Finding the nearest neighbor to connect to
            // By default, neighborhood states are sorted by distance, and collision checking
            // is performed in increasing order of distance
            if (delayCC_)
            {
                // calculate all costs and distances
                for (unsigned int i = 0; i < nbh.size(); ++i)
                    nbh[i]->cost += si_->distance(nbh[i]->state, motion->state);

                // sort the nodes
                std::sort(nbh.begin(), nbh.end(), compareMotion);

                for (unsigned int i = 0; i < nbh.size(); ++i)
                {
                    dists[i] = si_->distance(nbh[i]->state, motion->state);
                    nbh[i]->cost -= dists[i];
                }

                // Collision check until a valid motion is found
                // The first one found is the min, since the neighbors are sorted
                for (unsigned int i = 0; i < nbh.size(); ++i)
                {
                    if (nbh[i] == nmotion || si_->checkMotion(nbh[i]->state, motion->state))
                    {
                        motion->cost = nbh[i]->cost + dists[i];
                        motion->parent = nbh[i];
                        valid[i] = 1;
                        break;
                    }
                    else
                    {
                        valid[i] = -1;
                    }
                }
            }
            else
            {
                // find which one we connect the new state to
                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        dists[i] = si_->distance(nbh[i]->state, dstate);
                        double c = nbh[i]->cost + dists[i];
                        if (c < motion->cost)
                        {
                            if (si_->checkMotion(nbh[i]->state, dstate))
                            {
                                motion->cost = c;
                                motion->parent = nbh[i];
                                valid[i] = 1;
                            }
                            else
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        valid[i] = 1;
                        dists[i] = distN;
                    }
                }
            }

            // add motion to the tree
            nn_->add(motion);
            motion->parent->children.push_back(motion);

            // rewire tree if needed
            bool checkForSolution = false;
            for (unsigned int i = 0; i < nbh.size(); ++i)
            {
                if (nbh[i] == motion->parent) continue;

                double newcost = motion->cost + dists[i];
                if (newcost < nbh[i]->cost)
                {
                    // Check if the motion to the neighbor is valid
                    bool v = (valid[i] == 0 ? si_->checkMotion(nbh[i]->state, motion->state) : valid[i] == 1);
                    if (v)
                    {
                        // Need to subtract the difference in cost from all of the node's in the neighbor's subtree
                        double delta = newcost - nbh[i]->cost;
                        // Remove the neighbor node from it's parent's child list
                        removeFromParent(nbh[i]);

                        // Add the neighbor node as a child of motion
                        nbh[i]->parent = motion;
                        nbh[i]->cost = newcost;
                        motion->children.push_back(nbh[i]);

                        updateChildCosts(nbh[i], delta);
                        checkForSolution = true;
                    }
                }
            }

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            for (size_t i = 0; i < goalMotions_.size() && checkForSolution; ++i)
            {
                sufficientlyShort = opt->isSatisfied(goalMotions_[i]->cost);
                if (sufficientlyShort)
                {
                    solution = goalMotions_[i];
                    break;
                }
                else if (!solution || goalMotions_[i]->cost < solution->cost)
                {
                    solution = goalMotions_[i];
                }
            }

            // Checking for approximate solution (closest state found to the goal)
            if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist)
            {
                approximation = motion;
                approximatedist = distanceFromGoal;
            }
        }

        // terminate if a sufficient solution is found
        if (solution && sufficientlyShort)
            break;
    }

    bool approximate = (solution == NULL);
    bool addedSolution = false;
    if (approximate)
        solution = approximation;
    else
        lastGoalMotion_ = solution;

    if (solution != NULL)
    {
        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        PathGeometric *geopath = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            geopath->append(mpath[i]->state);

        base::PathPtr path(geopath);
        // Add the solution path, whether it is approximate (not reaching the goal), and the
        // distance from the end of the path to the goal (-1 if satisfying the goal).
        base::PlannerSolution psol(path, approximate, approximate ? approximatedist : -1.0);
        // Does the solution satisfy the optimization objective?
        psol.optimized_ = sufficientlyShort;

        pdef_->addSolutionPath (psol);

        addedSolution = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("Created %u new states. Checked %lu rewire options. %u goal states in tree.", statesGenerated, rewireTest, goalMotions_.size());

    return base::PlannerStatus(addedSolution, approximate);
}

void ompl::geometric::RRTstar::removeFromParent(Motion *m)
{
    std::vector<Motion*>::iterator it = m->parent->children.begin ();
    while (it != m->parent->children.end ())
    {
        if (*it == m)
        {
            it = m->parent->children.erase(it);
            it = m->parent->children.end ();
        }
        else
            ++it;
    }
}

void ompl::geometric::RRTstar::updateChildCosts(Motion *m, double delta)
{
    for (size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost += delta;
        updateChildCosts(m->children[i], delta);
    }
}

void ompl::geometric::RRTstar::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void ompl::geometric::RRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }

    data.properties["iterations INTEGER"] = boost::lexical_cast<std::string>(iterations_);
}
