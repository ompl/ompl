/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

/* Author: Jonathan Sobieski, Mark Moll */

#include "ompl/tools/config/SelfConfig.h"
#include "ompl/control/planners/pdst/PDST.h"

ompl::control::PDST::PDST(const SpaceInformationPtr &si)
    : base::Planner(si, "PDST"), siC_(si.get()), maxDistance_(0.0), bsp_(NULL),
    goalBias_(0.05), goalSampler_(NULL), iteration_(1), lastGoalMotion_(NULL)
{
    Planner::declareParam<double>("range", this, &PDST::setRange, &PDST::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &PDST::setGoalBias, &PDST::getGoalBias, "0.:.05:1.");
}

ompl::control::PDST::~PDST(void)
{
    freeMemory();
}

ompl::base::PlannerStatus ompl::control::PDST::solve(const base::PlannerTerminationCondition &ptc)
{
    // Make sure the planner is configured correctly.
    // ompl::base::Planner::checkValidity checks that there is at least one
    // start state and an ompl::base::Goal object specified and throws an
    // exception if this is not the case.
    checkValidity();

    if (!bsp_)
    {
        OMPL_ERROR("PDST was not set up.");
        return base::PlannerStatus::CRASH;
    }

    // depending on how the planning problem is set up, this may be necessary
    bsp_->bounds_ = projectionEvaluator_->getBounds();
    goalSampler_ = dynamic_cast<ompl::base::GoalSampleableRegion*>(pdef_->getGoal().get());

    // Ensure that we have a state sampler AND a control sampler
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    // Initialize to correct values depending on whether or not previous calls to solve
    // generated an approximate or exact solution. If solve is being called for the first
    // time then initializes hasSolution to false and isApproximate to true.
    double distanceToGoal, closestDistanceToGoal = std::numeric_limits<double>::infinity();
    bool hasSolution = lastGoalMotion_ != NULL;
    bool isApproximate = hasSolution ? pdef_->getGoal()->isSatisfied(lastGoalMotion_->state_, &closestDistanceToGoal) : true;
    unsigned ndim = projectionEvaluator_->getDimension();

    // If an exact solution has already been found, do not run for another iteration.
    if (hasSolution && !isApproximate)
        return ompl::base::PlannerStatus::EXACT_SOLUTION;

    // Initialize tree with start state(s)
    while (const base::State *st = pis_.nextStart())
    {
        Motion *startMotion = new Motion(si_->cloneState(st), projectionEvaluator_);
        bsp_->addMotion(startMotion);
        startMotion->heapElement_ = priorityQueue_.insert(startMotion);
    }

    if (priorityQueue_.empty())
    {
        OMPL_ERROR("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    base::State *scratch = si_->allocState();
    base::State *scratch2 = si_->allocState();

    while (!ptc)
    {
        // Get the top priority path.
        Motion *motionSelected = priorityQueue_.top()->data;
        motionSelected->updatePriority();
        priorityQueue_.update(motionSelected->heapElement_);

        Motion *newMotion = propagateFrom(motionSelected, hasSolution, distanceToGoal, scratch);
        if (newMotion == NULL)
            continue;

        // Check if the newMotion reached the goal.
        if (hasSolution)
        {
            closestDistanceToGoal = distanceToGoal;
            lastGoalMotion_ = newMotion;
            isApproximate = false;
            break;
        }
        else if (distanceToGoal < closestDistanceToGoal)
        {
            closestDistanceToGoal = distanceToGoal;
            lastGoalMotion_ = newMotion;
        }

        // subdivide cell that contained selected motion, put motions of that
        // cell in subcells and split motions so that they contained within
        // one subcell
        Cell *cellSelected = motionSelected->cell_;
        std::vector<Motion*> motions;
        cellSelected->subdivide(ndim);
        motions.swap(cellSelected->motions_);
        for (std::vector<Motion*>::iterator m = motions.begin() ; m != motions.end() ; ++m)
            insertSampleIntoBsp(*m, cellSelected, scratch, scratch2);
        iteration_++;
    }
    si_->freeState(scratch);
    si_->freeState(scratch2);


    if (lastGoalMotion_ != NULL)
        hasSolution = true;

    // If a solution path has been computed, save it in the problem definition object.
    if (hasSolution)
    {
        double dt = siC_->getPropagationStepSize();
        PathControl *path = new PathControl(si_);

        // Compute the path by going up the tree of motions.
        std::vector<Motion*> mpath;
        Motion *solution = lastGoalMotion_;
        while (solution->parent_ != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent_;
        }

        // Add the solution path in order from the start state to the goal.
        for (std::vector<Motion*>::reverse_iterator rIt = mpath.rbegin() ; rIt != mpath.rend() ; ++rIt)
        {
            if ((*rIt)->parent_ != NULL)
                path->append((*rIt)->state_, (*rIt)->control_, (*rIt)->controlDuration_ * dt);
            else
                path->append((*rIt)->state_);
        }
        pdef_->addSolutionPath(base::PathPtr(path), isApproximate, closestDistanceToGoal);
    }

    return base::PlannerStatus(hasSolution, isApproximate);
}

ompl::control::PDST::Motion* ompl::control::PDST::propagateFrom(Motion* motion, bool& goalReached, double& distanceToGoal, base::State* scratch)
{
    base::Goal *goal = pdef_->getGoal().get();

    if (motion->controlDuration_ > 1)
    {
        // sample a point along the trajectory given by motion
        unsigned int duration = rng_.uniformInt(1, motion->controlDuration_);
        if (duration != motion->controlDuration_)
        {
            // split motion into two motions
            motion = motion->split(siC_, duration, projectionEvaluator_);
            motion->heapElement_ = priorityQueue_.insert(motion);
        }
    }

    // generate a random nearby state
    if (goalSampler_ && rng_.uniform01() < goalBias_ && goalSampler_->canSample())
        goalSampler_->sampleGoal(scratch);
    else
        sampler_->sampleUniform(scratch);

    // generate a random control
    Control *randomControl = siC_->allocControl();
    // ignore returned duration. It's only the direction we care about; we
    // always propagate for (at most) siC_->getMaxControlDuration() steps
    controlSampler_->sampleTo(randomControl, motion->control_,
        motion->state_, scratch);

    // propagate using the random control and split when crossing cell bounds
    Motion *m = new Motion(si_->allocState(), motion, randomControl, 0, motion->projection_, iteration_, NULL),
        *prevm;
    base::State* prevState = si_->cloneState(motion->state_);
    Cell *prevCell = motion->cell_;
    base::EuclideanProjection prevProjection(projectionEvaluator_->getDimension());
    std::vector<Motion*> newMotions;
    unsigned index = 1, totalDuration = 0;

    while (true)
    {
        siC_->propagate(prevState, randomControl, 1, m->state_);
        if (!si_->isValid(m->state_))
        {
            std::swap(prevState, m->state_);
            goalReached = goal->isSatisfied(m->state_, &distanceToGoal);
            break;
        }
        projectionEvaluator_->project(m->state_, m->projection_);
        m->cell_ = bsp_->stab(m->projection_);
        if (m->cell_ != prevCell)
        {
            if (m->controlDuration_ > 0) // the first propagate call is allowed to cross a cell boundary
            {
                // start a new motion, save previous one
                prevm = m;
                m = new Motion(prevm->state_, prevm, siC_->cloneControl(randomControl),
                    1, prevm->projection_, iteration_, m->cell_);
                prevm->cell_= prevCell;
                prevm->state_ = si_->cloneState(prevState);
                prevm->projection_ = prevProjection;
                totalDuration += prevm->controlDuration_;
                newMotions.push_back(prevm);
            }
            prevCell = m->cell_;
        }
        else
            ++(m->controlDuration_);
        if ((goalReached = goal->isSatisfied(m->state_, &distanceToGoal))
            || index == siC_->getMaxControlDuration())
            break;
        std::swap(prevState, m->state_);
        prevProjection.swap(m->projection_);
        ++index;
    } // end while

    si_->freeState(prevState);
    // add last motion if it has non-zero duration
    if (m->controlDuration_ > 0)
    {
        totalDuration += m->controlDuration_;
        newMotions.push_back(m);
    }
    else
        freeMotion(m);

    // check if total motion duration is long enough
    if (totalDuration >= siC_->getMinControlDuration())
    {
        // if so, add motin segments to the priority queue and bsp.
        for (std::size_t i = 0; i < newMotions.size() ; ++i)
        {
            newMotions[i]->heapElement_ = priorityQueue_.insert(newMotions[i]);
            newMotions[i]->cell_->motions_.push_back(newMotions[i]);
        }
        return newMotions.back();
    }
    else
    {
        // otherwise, clean up
        for (std::size_t i = 0 ; i < newMotions.size() ; ++i)
            freeMotion(newMotions[i]);
        goalReached = false;
        distanceToGoal = std::numeric_limits<double>::infinity();
        return NULL;
    }
}

void ompl::control::PDST::insertSampleIntoBsp(Motion *motion, Cell *bsp, base::State* prevState, base::State* state)
{
    bsp->stab(motion->projection_)->addMotion(motion);
    priorityQueue_.update(motion->heapElement_);

    // If the motion is at most 1 step, then it cannot be split across cell
    // bounds. So we're done.
    if (motion->controlDuration_ <= 1)
        return;

    // motion can cross at most one cell boundary, since bsp has just been split
    // into two cells.
    si_->copyState(prevState, motion->parent_->state_);
    base::EuclideanProjection proj(projectionEvaluator_->getDimension()),
        prevProj(projectionEvaluator_->getDimension());
    Cell *cell = NULL, *prevCell = NULL;

    motion->cell_ = bsp->stab(motion->projection_);
    priorityQueue_.update(motion->heapElement_);
    for (unsigned int i = 0 ; i < motion->controlDuration_ ; ++i)
    {
        siC_->propagate(prevState, motion->control_, 1, state);
        projectionEvaluator_->project(state, proj);
        if (!prevCell)
        {
            prevCell = bsp->stab(proj);
            // if cell at the start and end are the same, then we don't split
            if (prevCell == motion->cell_)
                break;
        }
        else
        {
            cell = bsp->stab(proj);
            if (cell != prevCell)
            {
                motion = motion->split(siC_, i, si_->cloneState(prevState), prevProj, prevCell);
                motion->heapElement_ = priorityQueue_.insert(motion);
                break;
            }
        }
        std::swap(state, prevState);
        std::swap(cell, prevCell);
        proj.swap(prevProj);
    }
}

void ompl::control::PDST::clear(void)
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    iteration_ = 1;
    lastGoalMotion_ = NULL;
    freeMemory();
    bsp_ = new Cell(1.0, projectionEvaluator_->getBounds(), 0);
}

void ompl::control::PDST::freeMemory(void)
{
    // Iterate over the elements in the priority queue and clear it
    std::vector<Motion*> motions;
    motions.reserve(priorityQueue_.size());
    priorityQueue_.getContent(motions);
    for (std::vector<Motion*>::iterator it = motions.begin() ; it < motions.end() ; ++it)
        freeMotion(*it);
    priorityQueue_.clear(); // clears the Element objects in the priority queue
    delete bsp_;
    bsp_ = NULL;
}

void ompl::control::PDST::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);
    bsp_ = new Cell(1.0, projectionEvaluator_->getBounds(), 0);
    lastGoalMotion_ = NULL;
}

void ompl::control::PDST::getPlannerData(ompl::base::PlannerData &data) const
{
    base::Planner::getPlannerData(data);

    double propagationStepSize = siC_->getPropagationStepSize();
    std::vector<Motion*> motions;
    priorityQueue_.getContent(motions);

    // Add goal vertex
    if (lastGoalMotion_ != NULL)
        data.addGoalVertex(lastGoalMotion_->state_);

    for (std::vector<Motion*>::iterator it = motions.begin(); it < motions.end(); ++it)
    {
        Motion *child = *it;
        Motion *parent = child->parent_;

        if (parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(child->state_));
        else if (data.hasControls())
            data.addEdge(base::PlannerDataVertex(parent->state_),
                base::PlannerDataVertex(child->state_),
                PlannerDataEdgeControl(child->control_,
                    child->controlDuration_ * propagationStepSize));
        else
            data.addEdge(base::PlannerDataVertex(parent->state_),
                base::PlannerDataVertex(child->state_));
    }
}

void ompl::control::PDST::Cell::subdivide(unsigned int spaceDimension)
{
    double childVolume = 0.5 * volume_;
    unsigned int nextSplitDimension = (splitDimension_ + 1) % spaceDimension;
    splitValue_ = 0.5 * (bounds_.low[splitDimension_] + bounds_.high[splitDimension_]);

    left_ = new Cell(childVolume, bounds_, nextSplitDimension);
    left_->bounds_.high[splitDimension_] = splitValue_;
    left_->motions_.reserve(motions_.size());
    right_ = new Cell(childVolume, bounds_, nextSplitDimension);
    right_->bounds_.low[splitDimension_] = splitValue_;
    right_->motions_.reserve(motions_.size());
}
