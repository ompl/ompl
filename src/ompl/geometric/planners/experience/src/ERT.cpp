/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Rice University
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

/* Author: Èric Pairet */

#include "ompl/geometric/planners/experience/ERT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include <chrono>

ompl::geometric::ERT::ERT(const base::SpaceInformationPtr &si) :
    base::Planner(si, "ERT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("goal_bias", this, &ERT::setGoalBias, &ERT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("experience_fraction_min", this, &ERT::setExperienceFractionMin, &ERT::getExperienceFractionMin, "0.:.05:1.");
    Planner::declareParam<double>("experience_fraction_max", this, &ERT::setExperienceFractionMax, &ERT::getExperienceFractionMax, "0.:.05:1.");
    Planner::declareParam<double>("experience_tubular_radius", this, &ERT::setExperienceTubularRadius, &ERT::getExperienceTubularRadius, "0.:.05:10000.");
    Planner::declareParam<bool>("experience_initial_update", this, &ERT::setExperienceInitialUpdate, &ERT::getExperienceInitialUpdate, "0,1");
}

ompl::geometric::ERT::~ERT()
{
    freeMemory();

    /* in the deconstructor as MoveIt calls freeMemory() before planning */
    if (experience_)
    {
        for (auto &state : experience_->segment)
        if (state != nullptr)
        si_->freeState(state);
        experience_->segment.clear();
        delete experience_;
    }
}

void ompl::geometric::ERT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    pdf_.clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::ERT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            for (auto &state : motion->segment)
                if (state != nullptr)
                    si_->freeState(state);
            if (motion->state != nullptr)
                    si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::ERT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

bool ompl::geometric::ERT::isSegmentValid(const Motion *tmotion)
{
    /* check first state as next checkMotion assumes it valid */
    if (!si_->isValid(tmotion->segment[0]))
        return false;

    /* check the motions (segments) between states */
    for (unsigned int i = 0; i < tmotion->phase_span - 1; ++i)
        if (!si_->checkMotion(tmotion->segment[i], tmotion->segment[i + 1]))
            return false;
    return true;
}

void ompl::geometric::ERT::mapExperienceOntoProblem(const Motion *imotion, Motion *tmotion)
{
    const auto ss = si_->getStateSpace();
    const auto &locations = ss->getValueLocations();
    const unsigned int dimensionality = locations.size();
    std::vector<double> b(dimensionality), l(dimensionality);

    /* compute tranform parameters to connect */
    tmotion->phase_span = std::abs(int(tmotion->phase_end) - int(imotion->phase_end)) + 1;
    for (size_t i = 0; i < dimensionality; ++i)
    {
        b[i] = *(ss->getValueAddressAtLocation(imotion->state, locations[i])) - *(ss->getValueAddressAtLocation(experience_->segment[imotion->phase_end], locations[i]));
        l[i] = *(ss->getValueAddressAtLocation(tmotion->state, locations[i])) - (*(ss->getValueAddressAtLocation(experience_->segment[tmotion->phase_end], locations[i])) + b[i]);
    }

    /* transform segment */
    double t;
    for (size_t i = 0; i < tmotion->phase_span; ++i)
    {
        t = double(i) / double(tmotion->phase_span - 1);
        for (size_t j = 0; j < dimensionality; ++j)
            *(ss->getValueAddressAtLocation(tmotion->segment[i], locations[j])) = *(ss->getValueAddressAtLocation(experience_->segment[imotion->phase_end + i], locations[j])) + t * l[j] + b[j];
        ss->enforceBounds(tmotion->segment[i]);
    }
    si_->copyState(tmotion->state, tmotion->segment[tmotion->phase_span - 1]);
}

bool ompl::geometric::ERT::getValidSegment(const Motion *imotion, Motion *tmotion, const bool &connect_flag)
{
    /* when connecting the trees, the nearest motion might have the same phase */
    if (imotion->phase_end == tmotion->phase_end)
    {
        if (si_->checkMotion(imotion->state, tmotion->state))
        {
            si_->copyState(tmotion->segment[0], imotion->state);
            si_->copyState(tmotion->segment[1], tmotion->state);
            tmotion->phase_span = 2;
            return true;
        }
        return false;
    }

    const auto ss = si_->getStateSpace();
    const auto &locations = ss->getValueLocations();
    const unsigned int dimensionality = locations.size();
    std::vector<double> b(dimensionality), l(dimensionality);

    /* segment direction */
    int increment = int(tmotion->phase_end) - int(imotion->phase_end);
    int direction = (increment > 0) - (increment < 0);
    tmotion->phase_span = std::abs(increment) + 1;

    /* compute tranform parameters */
    if (connect_flag)
    {
        /* compute transform parameters to connect */
        for (size_t i = 0; i < dimensionality; ++i)
        {
            b[i] = *(ss->getValueAddressAtLocation(imotion->state, locations[i])) - *(ss->getValueAddressAtLocation(experience_->segment[imotion->phase_end], locations[i]));
            l[i] = *(ss->getValueAddressAtLocation(tmotion->state, locations[i])) - (*(ss->getValueAddressAtLocation(experience_->segment[tmotion->phase_end], locations[i])) + b[i]);
        }
    }
    else
    {
        /* compute transform parameters to explore */
        base::State *xstate = si_->allocState();
        double noise = (tmotion->phase_span - 1) * experienceTubularRadius_ / (experience_->phase_span - 1);
        for (size_t i = 0; i < dimensionality; ++i)
        {
            b[i] = *(ss->getValueAddressAtLocation(imotion->state, locations[i])) - *(ss->getValueAddressAtLocation(experience_->segment[imotion->phase_end], locations[i]));
            *(ss->getValueAddressAtLocation(xstate, locations[i])) = *(ss->getValueAddressAtLocation(experience_->segment[tmotion->phase_end], locations[i])) + b[i];
        }

        /* sample and validate target state */
        sampler_->sampleUniformNear(tmotion->state, xstate, noise); /* already enforcing bounds */
        if (!si_->isValid(tmotion->state))
        {
            si_->freeState(xstate);
            return false;
        }

        for (size_t i = 0; i < dimensionality; ++i)
            l[i] = *(ss->getValueAddressAtLocation(tmotion->state, locations[i])) - *(ss->getValueAddressAtLocation(xstate, locations[i]));

        si_->freeState(xstate);
    }

    /* transform segment while valid */
    double t;
    si_->copyState(tmotion->segment[0], imotion->state);
    for (size_t i = 1; i < tmotion->phase_span; ++i)
    {
        base::State *xstate = si_->allocState(); /* need to allocate memory per state as otherwise the validity flag of the previous check skips the checking of the new values */
        t = double(i) / double(tmotion->phase_span - 1);
        for (size_t j = 0; j < dimensionality; ++j)
            *(ss->getValueAddressAtLocation(xstate, locations[j])) = *(ss->getValueAddressAtLocation(experience_->segment[imotion->phase_end + i * direction], locations[j])) + t * l[j] + b[j];

        if (!si_->checkMotion(tmotion->segment[i - 1], xstate))
        {
            si_->freeState(xstate);
            return false;
        }

        si_->copyState(tmotion->segment[i], xstate);
        si_->freeState(xstate);
    }
    return true;
}

ompl::base::PlannerStatus ompl::geometric::ERT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_, 0);
        si_->copyState(motion->state, st);
        motion->phase_end = 0;
        motion->element = pdf_.add(motion, weightFunction(motion));
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal_s->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    /* map experience onto current planning problem; currently only one experience is created for a selected start and goal */
    auto *smotion = new Motion(si_, 0);
    smotion->state = pdef_->getStartState(0);
    smotion->phase_end = 0;

    auto *gmotion = new Motion(si_, 0);
    const base::State *st = pis_.nextGoal(ptc);
    if (st != nullptr)
    {
        si_->copyState(gmotion->state, st);
    }
    else
    {
        OMPL_ERROR("%s: Unable to sample any valid state in goal region", getName().c_str());
        return base::PlannerStatus::TIMEOUT;
    }

    if (!experience_)
    {
        OMPL_INFORM("%s: No experience provided. Setting straight experience", getName().c_str());

        experience_ = new Motion(si_, si_->getStateSpace()->validSegmentCount(smotion->state, gmotion->state) + 1);
        experience_->phase_end = experience_->phase_span - 1;
        gmotion->phase_end = experience_->phase_end;
        si_->getMotionStates(smotion->state, gmotion->state, experience_->segment, experience_->phase_span - 2, true, false);
    }
    else
    {
        /* update experience as its mapping onto the current planning problem */
        if (experienceInitialUpdate_)
        {
            Motion *tmotion = new Motion(si_, 0);
            si_->copyState(tmotion->state, gmotion->state);
            tmotion->phase_end = experience_->phase_end;
            tmotion->segment.resize(experience_->phase_span);
            tmotion->segment = experience_->segment; /* copy the pointers to update the experience_ */
            gmotion->phase_end = experience_->phase_end;

            mapExperienceOntoProblem(smotion, tmotion);
        }
    }
    OMPL_INFORM("%s: Updated experience has %u states", getName().c_str(), experience_->phase_span);

    /* check if the mapped experience is a solution */
    if (isSegmentValid(experience_))
    {
        OMPL_INFORM("%s: Updated experience solves the current problem defition!", getName().c_str());

        auto path(std::make_shared<PathGeometric>(si_));
        for (auto &state : experience_->segment)
            path->append(state);

        bool is_approximate = false;
        pdef_->addSolutionPath(path, is_approximate, si_->distance(gmotion->state, experience_->segment.back()), getName().c_str());
        return base::PlannerStatus(true, is_approximate);
    }

    /* proceed with the planning */
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    Motion *imotion;
    Motion *tmotion = new Motion(si_, experience_->phase_span); /* pre-allocate memory for candidate segments */

    bool connect_flag;
    if (segmentFractionMin_ > segmentFractionMax_)
    {
        OMPL_WARN("segmentFractionMin_ > segmentFractionMax_, swapping parameters.");
        double tmp = segmentFractionMin_;
        segmentFractionMin_ = segmentFractionMax_;
        segmentFractionMax_ = tmp;
    }
    unsigned int min_inc = std::max((unsigned int)(segmentFractionMin_ * experience_->phase_end), (unsigned int)(1));
    unsigned int max_inc = std::max((unsigned int)(segmentFractionMax_ * experience_->phase_end), (unsigned int)(min_inc + 1));

    while (!ptc)
    {
        /* pick a state from the tree */
        imotion = pdf_.sample(rng_.uniform01());
        imotion->selection_count++;
        pdf_.update(imotion->element, weightFunction(imotion));

        /* sample candidate segment to extend the tree */
        connect_flag = false;
        tmotion->phase_end = rng_.uniformInt(std::min(imotion->phase_end + min_inc, experience_->phase_end), std::min(imotion->phase_end + max_inc, experience_->phase_end));

        /* bias to goal with certain probability or when candidate segment has alpha_end = 1 */
        if (((goal_s != nullptr) && (rng_.uniform01() < goalBias_) && goal_s->canSample()) || (experience_->phase_end == tmotion->phase_end))
        {
            connect_flag = true;
            tmotion->phase_end = experience_->phase_end;
            goal_s->sampleGoal(tmotion->state);
        }

        /* attempt generating a valid segment to connect or explore */
        if (getValidSegment(imotion, tmotion, connect_flag))
        {
            Motion *motion = new Motion(si_, tmotion->phase_span);
            si_->copyState(motion->state, tmotion->segment[tmotion->phase_span - 1]);
            for (size_t i = 0; i < tmotion->phase_span; ++i)
                si_->copyState(motion->segment[i], tmotion->segment[i]);
            motion->phase_span = tmotion->phase_span;
            motion->phase_end = tmotion->phase_end;
            motion->parent = imotion;
            motion->element = pdf_.add(motion, weightFunction(motion));
            nn_->add(motion);

            double dist = 0.0;
            if (goal_s->isSatisfied(motion->state, &dist))
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution)
        solved = true;
    else
    {
        if (approxsol)
        {
            approximate = true;
            solution = approxsol;
        }
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
        {
            if (!mpath[i]->parent)
                path->append(mpath[i]->state);
            else
                for (size_t j = 1; j < mpath[i]->segment.size(); ++j)
                    path->append(mpath[i]->segment[j]);
        }

        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    for (auto &state : tmotion->segment)
        si_->freeState(state);
    si_->freeState(tmotion->state);
    delete tmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    return solved ? base::PlannerStatus::EXACT_SOLUTION : (approximate ? base::PlannerStatus::APPROXIMATE_SOLUTION : base::PlannerStatus::TIMEOUT);
}

void ompl::geometric::ERT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state), ert::PlannerDataEdgeSegment(motion->segment));
   }
}
