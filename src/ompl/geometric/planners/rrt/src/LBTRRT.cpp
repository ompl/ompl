/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Oren Salzman
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

/* Author: Oren Salzman, Sertac Karaman, Ioan Sucan, Mark Moll */

#include "ompl/geometric/planners/rrt/LBTRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include <limits>
#include <math.h>

const double ompl::geometric::LBTRRT::kRRG = 5.5;

ompl::geometric::LBTRRT::LBTRRT(const base::SpaceInformationPtr &si) :
    base::Planner(si, "LBTRRT"),
    goalBias_(0.05),
    maxDistance_(0.0),
    epsilon_(0.4),
    lastGoalMotion_(NULL),
    iterations_(0),
    bestCost_(std::numeric_limits<double>::quiet_NaN())
    {

    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &LBTRRT::setRange, &LBTRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &LBTRRT::setGoalBias, &LBTRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("epsilon", this, &LBTRRT::setApproximationFactor, &LBTRRT::getApproximationFactor, "0.:.1:10.");

    addPlannerProgressProperty("iterations INTEGER",
                               boost::bind(&LBTRRT::getIterationCount, this));
    addPlannerProgressProperty("best cost REAL",
                               boost::bind(&LBTRRT::getBestCost, this));

}

ompl::geometric::LBTRRT::~LBTRRT()
{
    freeMemory();
}

void ompl::geometric::LBTRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
    goalMotions_.clear();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::LBTRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&LBTRRT::distanceFunction, this, _1, _2));

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
            if (!dynamic_cast<base::PathLengthOptimizationObjective*>(opt_.get()))
                OMPL_WARN("%s: Asymptotic optimality has only been proven with path length optimizaton; convergence for other optimizaton objectives is not guaranteed.", getName().c_str());
        }
        else
            opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
}

void ompl::geometric::LBTRRT::freeMemory()
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

ompl::base::PlannerStatus ompl::geometric::LBTRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->costLb_ = motion->costApx_ = opt_->identityCost();
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution             = lastGoalMotion_;

    // \TODO Make this variable unnecessary, or at least have it
    // persist across solve runs
    base::Cost bestCost          = opt_->infiniteCost();
    Motion *approximation        = NULL;

    double  approximatedist      = std::numeric_limits<double>::infinity();
    bool sufficientlyShort       = false;

    Motion *rmotion              = new Motion(si_);
    base::State *rstate          = rmotion->state;
    base::State *xstate          = si_->allocState();
    unsigned int statesGenerated = 0;

    while (ptc() == false)
    {
        iterations_++;
        /* sample random state (with goal biasing) */
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s->canSample())
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
            statesGenerated++;
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);

            /* update fields */
            motion->parentLb_ = nmotion;
            motion->parentApx_ = nmotion;
            motion->incCost_ = costFunction(nmotion, motion);
            motion->costLb_ = opt_->combineCosts(nmotion->costLb_, motion->incCost_);
            motion->costApx_ = opt_->combineCosts(nmotion->costApx_, motion->incCost_);

            nmotion->childrenLb_.push_back(motion);
            nmotion->childrenApx_.push_back(motion);

            nn_->add(motion);

            bool checkForSolution = false;
            /* do lazy rewiring */
            unsigned int k = std::ceil(std::log(double(nn_->size())) * kRRG);
            std::vector<Motion *> nnVec;
            nn_->nearestK(rmotion, k, nnVec);

            CostCompare costCompare(*opt_, motion);
            std::sort(nnVec.begin(), nnVec.end(), costCompare);

            for (std::size_t i = 0; i < nnVec.size(); ++i)
                checkForSolution |= attemptNodeUpdate(motion, nnVec[i]);

            for (std::size_t i = 0; i < nnVec.size(); ++i)
                checkForSolution |= attemptNodeUpdate(nnVec[i], motion);

            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                for (size_t i = 0; i < goalMotions_.size(); ++i)
                {
                    if (opt_->isCostBetterThan(goalMotions_[i]->costApx_, bestCost))
                    {
                        bestCost = goalMotions_[i]->costApx_;
                        bestCost_ = bestCost;
                    }

                    sufficientlyShort = opt_->isSatisfied(goalMotions_[i]->costApx_);
                    if (sufficientlyShort)
                    {
                        solution = goalMotions_[i];
                        break;
                    }
                    else if (!solution ||
                             opt_->isCostBetterThan(goalMotions_[i]->costApx_, solution->costApx_))
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

    bool approximate = (solution == 0);
    bool addedSolution = false;
    if (approximate)
        solution = approximation;
    else
        lastGoalMotion_ = solution;

    if (solution != NULL)
    {
        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parentApx_;
        }

        /* set the solution path */
        PathGeometric *geoPath = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            geoPath->append(mpath[i]->state);

        base::PathPtr path(geoPath);
        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());
        if (approximate)
            psol.setApproximate(approximatedist);
        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, bestCost, sufficientlyShort);
        pdef_->addSolutionPath(psol);

        addedSolution = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states. %u goal states in tree.", getName().c_str(), statesGenerated, goalMotions_.size());

    return base::PlannerStatus(addedSolution, approximate);
}

bool ompl::geometric::LBTRRT::attemptNodeUpdate(Motion *potentialParent, Motion *child)
{
    base::Cost incCost = costFunction(potentialParent, child);
    base::Cost potentialLb = opt_->combineCosts(potentialParent->costLb_, incCost);
    base::Cost potentialApx = opt_->combineCosts(potentialParent->costApx_, incCost);

    if (!opt_->isCostBetterThan(potentialLb, child->costLb_))
        return false;

    if (opt_->isCostBetterThan(base::Cost((1.0 + epsilon_) *  potentialLb.value()), child->costApx_))
    {
        if (si_->checkMotion(potentialParent->state, child->state) == false)
            return false;

        removeFromParentLb(child);
        child->parentLb_ = potentialParent;
        potentialParent->childrenLb_.push_back(child);
        child->costLb_ = potentialLb;
        child->incCost_ = incCost;
        updateChildCostsLb(child);


        if (!opt_->isCostBetterThan(potentialApx, child->costApx_))
            return false;

        removeFromParentApx(child);
        child->parentApx_ = potentialParent;
        potentialParent->childrenApx_.push_back(child);
        child->costApx_ = potentialApx;
        updateChildCostsApx(child);

        if (opt_->isCostBetterThan(potentialApx, bestCost_))
            return true;
    }
    else //(child->costApx_ <= (1 + epsilon_) *  potentialLb)
    {
        removeFromParentLb(child);
        child->parentLb_ = potentialParent;
        potentialParent->childrenLb_.push_back(child);
        child->costLb_ = potentialLb;
        child->incCost_ = incCost;
        updateChildCostsLb(child);
    }
    return false;
}

void ompl::geometric::LBTRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parentApx_ == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parentApx_->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
}

void ompl::geometric::LBTRRT::updateChildCostsLb(Motion *m)
{
    for (std::size_t i = 0; i < m->childrenLb_.size(); ++i)
    {
        m->childrenLb_[i]->costLb_ = opt_->combineCosts(m->costLb_, m->childrenLb_[i]->incCost_);
        updateChildCostsLb(m->childrenLb_[i]);
    }
}
void ompl::geometric::LBTRRT::updateChildCostsApx(Motion *m)
{
    for (std::size_t i = 0; i < m->childrenApx_.size(); ++i)
    {
        m->childrenApx_[i]->costApx_ = opt_->combineCosts(m->costApx_, m->childrenApx_[i]->incCost_);
        updateChildCostsApx(m->childrenApx_[i]);
    }
}

void ompl::geometric::LBTRRT::removeFromParentLb(Motion *m)
{
    return removeFromParent(m, m->parentLb_->childrenLb_);
}
void ompl::geometric::LBTRRT::removeFromParentApx(Motion *m)
{
    return removeFromParent(m, m->parentApx_->childrenApx_);
}
void ompl::geometric::LBTRRT::removeFromParent(const Motion *m, std::vector<Motion*>& vec)
{
    for (std::vector<Motion*>::iterator it = vec.begin (); it != vec.end(); ++it)
        if (*it == m)
        {
            vec.erase(it);
            break;
        }
}
