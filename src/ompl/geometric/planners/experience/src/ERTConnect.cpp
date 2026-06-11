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

#include "ompl/geometric/planners/experience/ERTConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::ERTConnect::ERTConnect(const base::SpaceInformationPtr &si)
  : base::Planner(si, "ERTConnect")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("experience_fraction_min", this, &ERTConnect::setExperienceFractionMin, &ERTConnect::getExperienceFractionMin, "0.:.05:1.");
    Planner::declareParam<double>("experience_fraction_max", this, &ERTConnect::setExperienceFractionMax, &ERTConnect::getExperienceFractionMax, "0.:.05:1.");
    Planner::declareParam<double>("experience_tubular_radius", this, &ERTConnect::setExperienceTubularRadius, &ERTConnect::getExperienceTubularRadius, "0.:.05:10000.");
    Planner::declareParam<bool>("experience_initial_update", this, &ERTConnect::setExperienceInitialUpdate, &ERTConnect::getExperienceInitialUpdate, "0,1");
}

ompl::geometric::ERTConnect::~ERTConnect()
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

void ompl::geometric::ERTConnect::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();

    pdf_tStart_.clear();
    pdf_tGoal_.clear();
}

void ompl::geometric::ERTConnect::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
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

    if (tGoal_)
    {
        tGoal_->list(motions);
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

void ompl::geometric::ERTConnect::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

bool ompl::geometric::ERTConnect::isSegmentValid(const Motion *tmotion)
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

void ompl::geometric::ERTConnect::mapExperienceOntoProblem(const Motion *imotion, Motion *tmotion)
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

bool ompl::geometric::ERTConnect::getValidSegment(const Motion *imotion, Motion *tmotion, const bool &connect_flag)
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

ompl::base::PlannerStatus ompl::geometric::ERTConnect::solve(const base::PlannerTerminationCondition &ptc)
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
        motion->element = pdf_tStart_.add(motion, weightFunction(motion));
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
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
        gmotion->element = pdf_tGoal_.add(gmotion, weightFunction(gmotion));
        tGoal_->add(gmotion);
    }
    else
    {
        OMPL_ERROR("%s: Unable to sample any valid state for goal tree", getName().c_str());
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

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(), (int)(tStart_->size() + tGoal_->size()));

    bool solved = false;
    bool startTree = false;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    Motion *imotion;
    Motion *tmotion = new Motion(si_, experience_->phase_span); /* pre-allocate memory for candidate segments */
    Motion *addedMotion = nullptr;
    Motion *otherAddedMotion = nullptr;

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
        startTree = !startTree;
        TreeData &tree = startTree ? tStart_ : tGoal_;
        TreeData &otherTree = startTree ? tGoal_ : tStart_;
        PDF<Motion*> &pdf = startTree ? pdf_tStart_ : pdf_tGoal_;
        PDF<Motion*> &otherPdf = startTree ? pdf_tGoal_ : pdf_tStart_;
        Motion *target = startTree ? gmotion : smotion;

        addedMotion = nullptr;
        otherAddedMotion = nullptr;

        /* pick a state from the tree */
        imotion = pdf.sample(rng_.uniform01());
        imotion->selection_count++;
        pdf.update(imotion->element, weightFunction(imotion));

        /* sample candidate segment to explore from the tree */
        connect_flag = false;
        if (startTree)
            tmotion->phase_end = rng_.uniformInt(std::min(imotion->phase_end + min_inc, experience_->phase_end), std::min(imotion->phase_end + max_inc, experience_->phase_end));
        else
            tmotion->phase_end = rng_.uniformInt(std::max(0, (int)imotion->phase_end - (int)max_inc), std::max(0, (int)imotion->phase_end - (int)min_inc));

        /* switch to connect mode if the segment attempts to reach the other end */
        if ((startTree && (tmotion->phase_end == experience_->phase_end)) || (!startTree && (tmotion->phase_end == 0)))
        {
            connect_flag = true;
            tmotion->phase_end = target->phase_end;
            si_->copyState(tmotion->state, target->state); /* make sure we don't modify pointed start/goal */
        }

        /* attempt generating a valid segment to connect or explore */
        if (!getValidSegment(imotion, tmotion, connect_flag))
            continue;

        Motion *motion = new Motion(si_, tmotion->phase_span);
        si_->copyState(motion->state, tmotion->state);
        for (size_t i = 0; i < tmotion->phase_span; ++i)
            si_->copyState(motion->segment[i], tmotion->segment[i]);
        motion->phase_span = tmotion->phase_span;
        motion->phase_end = tmotion->phase_end;
        motion->parent = imotion;
        motion->element = pdf.add(motion, weightFunction(motion));
        tree->add(motion);

        /* remember which motion was just added and check if it reaches the other end */
        addedMotion = motion;
        if (connect_flag)
        {
            OMPL_INFORM("%s: Solution found by %s", getName().c_str(), startTree ? "startTree" : "goalTree");
            solved = true;
            break;
        }

        /* attempt connecting to last added motion from the nearest state in the other tree*/
        imotion = otherTree->nearest(addedMotion);
        imotion->selection_count++;
        otherPdf.update(imotion->element, weightFunction(imotion));

        /* attempt generating a valid segment to connect to the other tree */
        if (getValidSegment(imotion, tmotion, true))
        {
            Motion *other_motion = new Motion(si_, tmotion->phase_span);
            si_->copyState(other_motion->state, tmotion->segment[tmotion->phase_span - 1]);
            for (size_t i = 0; i < tmotion->phase_span; ++i)
                si_->copyState(other_motion->segment[i], tmotion->segment[i]);
            other_motion->phase_span = tmotion->phase_span;
            other_motion->phase_end = tmotion->phase_end;
            other_motion->parent = imotion;
            other_motion->element = otherPdf.add(other_motion, weightFunction(other_motion));
            otherTree->add(other_motion);

            /* remember the otherAddedMotion */
            OMPL_INFORM("%s: Solution found by connecting both trees", getName().c_str());
            otherAddedMotion = other_motion;
            solved = true;
            break;
        }

        /* update approximate solution */
        if (startTree)
        {
            double dist = 0.0;
            goal_s->isSatisfied(addedMotion->state, &dist);
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = addedMotion;
            }
        }
    }

    /* retrieve solution: approximate (only startTree), or exact from either tree or from connecting both*/
    bool approximate = false;
    Motion *startMotion = nullptr;
    Motion *goalMotion = nullptr;
    if (solved)
    {
        approxdif = 0;
        startMotion = startTree ? addedMotion : otherAddedMotion;
        goalMotion = startTree ? otherAddedMotion : addedMotion;
    }
    else
    {
        if (approxsol)
        {
            approximate = true;
            startMotion = approxsol;
        }
    }

    if (startMotion || goalMotion)
    {
        /* construct the solution path */
        Motion *solution = startMotion;
        std::vector<Motion *> mpath1;
        while (solution != nullptr)
        {
            mpath1.push_back(solution);
            solution = solution->parent;
        }

        solution = goalMotion;
        std::vector<Motion *> mpath2;
        while (solution != nullptr)
        {
            mpath2.push_back(solution);
            solution = solution->parent;
        }

        /* remove connection state to avoid duplicate state */
        if (!mpath1.empty() && !mpath2.empty())
            mpath1[0]->segment.erase(mpath1[0]->segment.end() - 1);

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath1.size() - 1; i >= 0; --i)
        {
            if (!mpath1[i]->parent)
                path->append(mpath1[i]->state);
            else
                for (size_t j = 1; j < mpath1[i]->segment.size(); ++j)
                    path->append(mpath1[i]->segment[j]);
        }

        for (auto &motion : mpath2)
        {
            if (!motion->parent)
                path->append(motion->state);
            else
                for (int j = motion->segment.size() - 1; j > 0; --j)
                    path->append(motion->segment[j]);
        }

        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    for (auto &state : tmotion->segment)
        si_->freeState(state);
    si_->freeState(tmotion->state);
    delete tmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());
    return solved ? base::PlannerStatus::EXACT_SOLUTION : (approximate ? base::PlannerStatus::APPROXIMATE_SOLUTION : base::PlannerStatus::TIMEOUT);
}

void ompl::geometric::ERTConnect::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1), ert::PlannerDataEdgeSegment(motion->segment));
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2), ert::PlannerDataEdgeSegment(motion->segment));
    }
}
