/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Rice University
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
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include <chrono>

ompl::geometric::ERT::ERT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "ERTintermediate" : "ERT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("goal_bias", this, &ERT::setGoalBias, &ERT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("experience_fraction_min", this, &ERT::setExperienceFractionMin, &ERT::getExperienceFractionMin, "0.:.05:1.");
    Planner::declareParam<double>("experience_fraction_max", this, &ERT::setExperienceFractionMax, &ERT::getExperienceFractionMax, "0.:.05:1.");
    Planner::declareParam<double>("experience_tubular_radius", this, &ERT::setExperienceTubularRadius, &ERT::getExperienceTubularRadius, "0.:.05:10000.");
    Planner::declareParam<bool>("experience_initial_update", this, &ERT::setExperienceInitialUpdate, &ERT::getExperienceInitialUpdate, "0,1");

    // NOTE: not used
    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::ERT::~ERT()
{
    freeMemory();
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

void ompl::geometric::ERT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
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

    // NOTE: this might need to move somewhere else, as MoveIt might call freeMemory() before planning
    for (auto &state : experience_)
        if (state != nullptr)
            si_->freeState(state);
    experience_.clear();
}

// NOTE: would checking the path by continuous sud-division lead to better performance?
// check: https://ompl.kavrakilab.org/DiscreteMotionValidator_8cpp_source.html#l00093
bool ompl::geometric::ERT::isSegmentValid(const std::vector<base::State*> &segment)
{
    // check the end-states (nodes) of all segments composing the path
    if (!si_->checkMotion(segment, segment.size() - 1))
        return false;

    // check the motions (segments) between end-states (nodes)
    for (unsigned int i = 0; i < segment.size() - 1; ++i)
        if (!si_->checkMotion(segment[i], segment[i + 1]))
            return false;
    return true;
}

// NOTE: check that states are not validated multiple times
bool ompl::geometric::ERT::isSegmentValid(const Motion *tmotion)
{
    // check the end-states (nodes) of all segments composing the path
    if (!si_->checkMotion(tmotion->segment, tmotion->phase_span))
        return false;

    // check the motions (segments) between end-states (nodes)
    for (unsigned int i = 0; i < tmotion->phase_span - 1; ++i)
        if (!si_->checkMotion(tmotion->segment[i], tmotion->segment[i + 1]))
            return false;
    return true;
}

// NOTE: check if making tmotion const, makes the content const
void ompl::geometric::ERT::getSegment(const Motion *imotion, Motion *tmotion, const bool connect_flag)
{
    // #################################
    // SETUP
    // #################################
    // NOTE: some of these could be pre-declared
    // although these are only copies of pointers
    const auto ss = si_->getStateSpace();
    const auto &locations = ss->getValueLocations();
    const unsigned int dimensionality = locations.size();

    // NOTE: some of these could be pre-declared
    std::vector<double> b(dimensionality), l(dimensionality);

    // #################################
    // NOTE: temporal checks
    // #################################
    if (!ss->satisfiesBounds(imotion->state)) {
        OMPL_ERROR("%s: imotion->state does not satisfies bounds!", getName().c_str());
    }
    // ##################


    /*  */
    tmotion->phase_span = std::abs(int(tmotion->demo_index) - int(imotion->demo_index)) + 1;
    if (connect_flag)
    {
        /* compute transform parameters to connect */
        for(size_t i = 0; i < dimensionality; ++i)
        {
            b[i] = *(ss->getValueAddressAtLocation(imotion->state, locations[i])) - *(ss->getValueAddressAtLocation(experience_[imotion->demo_index], locations[i]));
            l[i] = *(ss->getValueAddressAtLocation(tmotion->state, locations[i])) - (*(ss->getValueAddressAtLocation(experience_[tmotion->demo_index], locations[i])) + b[i]);
        }
    }
    else
    {
        // NOTE: could this noise be pre-computed?
        /* compute transform parameters to explore */
        double noise = tmotion->phase_span * experience_tubular_radius_ / experience_.size();
        for(size_t i = 0; i < dimensionality; ++i)
        {
            b[i] = *(ss->getValueAddressAtLocation(imotion->state, locations[i])) - *(ss->getValueAddressAtLocation(experience_[imotion->demo_index], locations[i]));
            l[i] = rng_.uniformReal(-noise, noise);
        }
    }

    /* transform segment */
    double t;
    for(size_t i = 0; i < tmotion->phase_span; ++i)
    {
        t = double(i) / double(tmotion->phase_span - 1);
        for(size_t j = 0; j < dimensionality; ++j)
            *(ss->getValueAddressAtLocation(tmotion->segment[i], locations[j])) = *(ss->getValueAddressAtLocation(experience_[imotion->demo_index + i], locations[j])) + t * l[j] + b[j];

        // NOTE: can this change the guarantees of continuity between states?
        // NOTE: can this create any discontinuity?
        ss->enforceBounds(tmotion->segment[i]);
    }
    si_->copyState(tmotion->state, tmotion->segment[tmotion->phase_span - 1]);

    // #################################
    // NOTE: temporal checks
    // #################################
    // check correctness of the resulting segment
    double tolerance = 1e-10;
    if (!(si_->distance(imotion->state, tmotion->segment[0]) < tolerance) || !(si_->distance(tmotion->state, tmotion->segment[tmotion->phase_span - 1]) < tolerance)) {
        OMPL_ERROR("%s: Generated segment does not match desired init and targ states!", getName().c_str());
        // std::cout << "init discrepancy: " << si_->distance(imotion->state, tmotion->segment.front()) << std::endl;
        // std::cout << "targ discrepancy: " << si_->distance(tmotion->state, tmotion->segment.back()) << std::endl;
        std::cout << "init discrepancy: " << si_->distance(imotion->state, tmotion->segment[0]) << std::endl;
        std::cout << "targ discrepancy: " << si_->distance(tmotion->state, tmotion->segment[tmotion->phase_span - 1]) << std::endl;
        // exit(0);
    }
    // #################################
}


ompl::base::PlannerStatus ompl::geometric::ERT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // #############################################
    // NOTE: TEMPORAL CHECKS
    // #############################################
    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // increase goal threshold to deal with numerical issues
    // si_->getStateSpace()->setLongestValidSegmentFraction(1e-6);
    goal_s->setThreshold(1e-5);
    // #############################################

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_, 0);
        si_->copyState(motion->state, st);
        motion->demo_index = 0;
        motion->element = pdf_.add(motion, weightFunction(motion));
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // #############################################
    // map experience onto current planning problem
    // #############################################
    // NOTE: should this be here or in setup()? Do we know everything we need in setup()?
    // NOTE: currently only one experience is created for a selected start and goal

    base::State *sstate = si_->allocState();
    base::State *gstate = si_->allocState();
    sstate = pdef_->getStartState(0);
    if ((goal_s != nullptr) && goal_s->canSample())
        goal_s->sampleGoal(gstate);
    else
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }








    if (experience_.empty())
    {
        OMPL_INFORM("%s: No experience provided. Setting straight experience", getName().c_str());

        // NOTE: this should be defined according to the max_segment_fraction!
        // NOTE: should this parameter be public?
        // populate experience as a straigh line between start and goal
        unsigned int count = 50;
        experience_.resize(count);
        si_->getMotionStates(sstate, gstate, experience_, count, true, true); // last argument true since experience_ has not been allocatd memory before

        // #############################################
        // NOTE: TEMPORAL CHECKS
        // #############################################
        double tolerance = 1e-10;
        if (!(si_->distance(sstate, experience_.front()) < tolerance) || !(si_->distance(gstate, experience_.back()) < tolerance)) {
            OMPL_ERROR("%s: start-goal of the experience does not match the set start-goal states", getName().c_str());
            return base::PlannerStatus::CRASH;
        }
        // #############################################
    }
    else
    {
        // TODO: does this deform the experience?
        // resample experience to given count
        //.resample


        /* update experience as its mapping onto the current planning problem */
        if (experienceInitialUpdate_)
        {
            Motion *imotion = new Motion(si_, 0); // init
            imotion->state = sstate;
            imotion->demo_index = 0;

            Motion *tmotion = new Motion(si_, 0); // targ
            tmotion->state = gstate;
            tmotion->demo_index = experience_.size() - 1;
            tmotion->segment.resize(experience_.size());
            tmotion->segment = experience_; // copy the pointers to update the experience_

            getSegment(imotion, tmotion, true);


            // #############################################
            // NOTE: TEMPORAL CHECKS
            // #############################################
            double tolerance = 1e-10;
            if (!(si_->distance(sstate, experience_.front()) < tolerance) || !(si_->distance(gstate, experience_.back()) < tolerance)) {
                OMPL_ERROR("%s: start-goal of the experience does not match the set start-goal states", getName().c_str());
                return base::PlannerStatus::CRASH;
            }
            // #############################################



            // si_->freeState(imotion->state);
            // delete imotion;
            // si_->freeState(tmotion->state);
            // delete tmotion;
        }
    }
    OMPL_INFORM("%s: Updated experience has %u states", getName().c_str(), experience_.size());

    // NOTE: need to free his memory
    // std::cout << "deleting sstate" << std::endl;
    // std::cout << "deleting gstate" << std::endl;

    //


    /* check if the mapped experience is a solution */
    if (isSegmentValid(experience_))
    {
        OMPL_INFORM("%s: Updated experience solves the current problem defition!", getName().c_str());

        auto path(std::make_shared<PathGeometric>(si_));
        for (auto &state : experience_)
            path->append(state);

        bool is_approximate = false;
        pdef_->addSolutionPath(path, is_approximate, si_->distance(gstate, experience_.back()), getName().c_str());
        return base::PlannerStatus(true, is_approximate);
    }

    // #############################################
    // NOTE: AT THIS POINT, WE START PLANNING
    // #############################################
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    // TODO: need to assert that segment_fraction_min < segment_fraction_max
    bool connect_flag;
    unsigned int max_ind_demo = experience_.size() - 1; // TODO: if the experience is a Motion*, this could be experience_->phase_span
    unsigned int min_inc = std::max((unsigned int)(segment_fraction_min_ * max_ind_demo), (unsigned int)1);
    unsigned int max_inc = std::max((unsigned int)(segment_fraction_max_ * max_ind_demo), (unsigned int)(min_inc+1));

    Motion *imotion; // init
    Motion *tmotion = new Motion(si_, experience_.size()); /* pre-allocate memory for candidate segments */

    while (!ptc)
    {
        /* pick a state from the tree */
        imotion = pdf_.sample(rng_.uniform01());
        imotion->selected_num++;
        pdf_.update(imotion->element, weightFunction(imotion));


        // #############################################
        // NOTE: temporal checks
        // #############################################
        // TODO: this does not happen anymore since we are not adding the goal to the PDF
        // TODO: it would be nice if the PDF already avoid picking goals
        if (imotion->demo_index == max_ind_demo) {
            double distx;
            bool sat = goal->isSatisfied(imotion->state, &distx);
            std::cout << "THIS SHOULD NOT HAPPEN " << pdf_.getWeight(imotion->element) << " " << sat << " " << distx << std::endl;
            exit(0);
        }
        // #############################################




        /* sample candidate segment to extend the tree */
        connect_flag = false;
        tmotion->demo_index = rng_.uniformInt(std::min(imotion->demo_index + min_inc, max_ind_demo), std::min(imotion->demo_index + max_inc, max_ind_demo));

        /* bias to goal with certain probability or when candidate segment has alpha_end = 1 */
        if (((goal_s != nullptr) && (rng_.uniform01() < goalBias_) && goal_s->canSample()) || ((max_ind_demo - tmotion->demo_index) <= min_inc)) {
            connect_flag = true;
            tmotion->demo_index = max_ind_demo;
            goal_s->sampleGoal(tmotion->state);
        }

        // NOTE: could this already return if the segment is valid?
        // NOTE: could this only pass the motions? If so, the segment could be stored in tmotion->segment
        /* get segment to connect or explore */
        getSegment(imotion, tmotion, connect_flag);

        // NOTE: temporal, need to think about a better way
        // tmotion->segment.resize(candidate_segment_phase_span_);
        // for (size_t i = 0; i < candidate_segment_phase_span_; ++i)
        //     tmotion->segment[i] = candidate_segment_[i];

        /* integrate segment to the tree */
        if (isSegmentValid(tmotion))
        {
            Motion *motion = new Motion(si_, tmotion->phase_span);
            si_->copyState(motion->state, tmotion->state);
            for (size_t i = 0; i < tmotion->phase_span; ++i)
                si_->copyState(motion->segment[i], tmotion->segment[i]);
            motion->phase_span = tmotion->phase_span;
            motion->demo_index = tmotion->demo_index;
            motion->parent = imotion;

            // NOTE: this should not be needed if all target states where to converge to the goal
            // assign weight to new node (0 for the states corresponding at the end of the demonstration, such that they are not selected)
            double weight = 0.0;
            if (!(tmotion->demo_index == max_ind_demo)) {weight = weightFunction(motion);}
            motion->element = pdf_.add(motion, weight);
            // motion->element = pdf_.add(motion, weightFunction(motion));
            nn_->add(motion);

            double dist = 0.0;
            if (goal->isSatisfied(motion->state, &dist))
            {
                approxdif = dist;
                solution = motion;
                // break; // NOTE: put this back, as well as the weightFunction above
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
        else
        {
            // NOTE: if the vector was resized all the time (i.e. only declared before the main loop), whould this be necessary?
            // NOTE: this should be done even when a solution is found! However, it deletes the one stored in the tree...
            // free memory of the invalid segment
            // maybe no need to free all the time if re-using the same variable?
            // for (auto &state : tmotion->segment)
            //     si_->freeState(state);
        }

    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
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
        solved = true;
    }

    // NOTE: need to free memory
    // NOTE: need to free the segment in the motion too?

    // si_->freeState(sstate);
    // si_->freeState(gstate);
    // if (imotion->state != nullptr)
    //     si_->freeState(imotion->state);
    // delete imotion;
    // if (tmotion->state != nullptr)
    //     si_->freeState(tmotion->state);
    // delete tmotion;

    for (auto &state : tmotion->segment)
        si_->freeState(state);
    si_->freeState(tmotion->state);
    delete tmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
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
           data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state), base::PlannerDataEdgeSegment(motion->segment));
   }
}
