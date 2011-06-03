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

/* Authors: Alejandro Perez, Sertac Karaman, Ioan Sucan */

#include "ompl/contrib/rrt_star/src/RRTstar.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include <algorithm>
#include <limits>
#include <map>
#include <stdio.h>

void ompl::geometric::RRTstar::setup(void)
{
    Planner::setup();
    checkMotionLength(this, maxDistance_);

    ballRadiusMax_ = si_->getMaximumExtent();
    ballRadiusConst_ = maxDistance_ * sqrt(si_->getStateSpace()->getDimension());

    delayCC_ = true;

    if (!nn_)
        nn_.reset(new NearestNeighborsSqrtApprox<Motion*>());
    nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _1, _2));
}

void ompl::geometric::RRTstar::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

bool ompl::geometric::RRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    if (!goal)
    {
        msg_.error("Goal undefined");
        return false; }
    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        msg_.error("There are no valid initial states!");
        return false;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    msg_.inform("Starting with %u states", nn_->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    std::vector<Motion*> solCheck;
    std::vector<Motion*> nbh;
    std::vector<double>  dists;
    std::vector<int>     valid;
    long unsigned int    rewireTest = 0;
    int                  iter = 0;
    double               lowerBound = std::numeric_limits<double>::infinity();

    while (ptc() == false)
    {

        iter++;

        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            /* create a motion */
            double distN = si_->distance(dstate, nmotion->state);
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->cost = nmotion->cost + distN;

            /* find nearby neighbors */
            double r = std::min(ballRadiusConst_ * (sqrt(log((double)(1 + nn_->size())) / ((double)(nn_->size())))),
                                ballRadiusMax_);

            nn_->nearestR(motion, r, nbh);
            rewireTest += nbh.size();

            // cache for distance computations
            dists.resize(nbh.size());
            // cache for motion validity
            valid.resize(nbh.size());
            std::fill(valid.begin(), valid.end(), 0);

            if(delayCC_)
            {
                    // calculate all costs and distances
                    for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                    if (nbh[i] != nmotion)
                    {
                        double c = nbh[i]->cost + si_->distance(nbh[i]->state, dstate);
                        nbh[i]->cost = c;
                    }

                    // sort the nodes
                    std::sort(nbh.begin(), nbh.end(), compareMotion);

                    for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                    if (nbh[i] != nmotion)
                    {
                       dists[i] = si_->distance(nbh[i]->state, dstate);
                       nbh[i]->cost -= dists[i];
                    }

                    // collision check until a valid motion is found
                    for (unsigned int i = 0 ; i < nbh.size() ; ++i)
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
                                  break;
                         }
                         else
                                  valid[i] = -1;
                       }
                    }
                    else
                    {
                            valid[i] = 1;
                            dists[i] = distN;
                            break;
                    }

            }
            else{
                    /* find which one we connect the new state to*/
                    for (unsigned int i = 0 ; i < nbh.size() ; ++i)
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

            /* add motion to tree */
            nn_->add(motion);

            solCheck.resize(1);
            solCheck[0] = motion;

            /* rewire tree if needed */
            for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                if (nbh[i] != motion->parent)
                {
                    double c = motion->cost + dists[i];
                    if (c < nbh[i]->cost)
                    {
                        bool v = valid[i] == 0 ? si_->checkMotion(nbh[i]->state, dstate) : valid[i] == 1;
                        if (v)
                        {
                            nbh[i]->parent = motion;
                            nbh[i]->cost = c;
                            solCheck.push_back(nbh[i]);
                        }
                    }
                }

            /* check if  we found a solution */
            for (unsigned int i = 0 ; i < solCheck.size() ; ++i)
            {
                double dist = 0.0;
                bool solved = goal->isSatisfied(solCheck[i]->state, solCheck[i]->cost, &dist);
                if (solved)
                {
                    approxdif = dist;
                    if (solCheck[i]->cost < lowerBound)
                    {
                       lowerBound = solCheck[i]->cost;
                       solution = solCheck[i];
                    }
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = solCheck[i];
                }
            }

            /* terminate if a solution was found */
            if (solution != NULL)
                break;
        }
    }

    if (solution == NULL && approxsol != NULL && goal->isSatisfied(approxsol->state, &approxdif))
        solution = approxsol;

    bool approximate = false;

    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != NULL)
    {
        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
           for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->states.push_back(si_->cloneState(mpath[i]->state));
        goal->setDifference(approxdif);
        goal->setSolutionPath(base::PathPtr(path), approximate);

        if (approximate)
            msg_.warn("Found approximate solution");
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    msg_.inform("Created %u states. Checked %lu rewire options.", nn_->size(), rewireTest);

    return goal->isAchieved();
}

void ompl::geometric::RRTstar::freeMemory(void)
{
    if (nn_) {
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

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
        data.recordEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
}
