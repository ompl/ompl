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
#include "ompl/geometric/planners/pdst/PDST.h"

ompl::geometric::PDST::PDST(const base::SpaceInformationPtr &si)
    : base::Planner(si, "PDST"), maxDistance_(0.0), bsp_(NULL),
    goalBias_(0.05), goalSampler_(NULL), iteration_(1), lastGoalMotion_(NULL)
{
    Planner::declareParam<double>("range", this, &PDST::setRange, &PDST::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &PDST::setGoalBias, &PDST::getGoalBias, "0.:.05:1.");
}

ompl::geometric::PDST::~PDST(void)
{
    freeMemory();
}

ompl::base::PlannerStatus ompl::geometric::PDST::solve(const base::PlannerTerminationCondition &ptc)
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
    base::Goal *goal = pdef_->getGoal().get();
    goalSampler_ = dynamic_cast<ompl::base::GoalSampleableRegion*>(goal);

    // Ensure that we have a state sampler
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // Initialize to correct values depending on whether or not previous calls to solve
    // generated an approximate or exact solution. If solve is being called for the first
    // time then initializes hasSolution to false and isApproximate to true.
    double distanceToGoal, closestDistanceToGoal = std::numeric_limits<double>::infinity();
    bool hasSolution = lastGoalMotion_ != NULL;
    bool isApproximate = hasSolution ? goalSampler_->isSatisfied(lastGoalMotion_->state_, &closestDistanceToGoal) : true;
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

    base::State* state = si_->allocState();
    base::EuclideanProjection projection(projectionEvaluator_->getDimension());
    while (!ptc)
    {
        // Get the top priority path.
        Motion *motionSelected = priorityQueue_.top()->data;
        motionSelected->updatePriority();
        priorityQueue_.update(motionSelected->heapElement_);

        Motion *newMotion = propagateFrom(motionSelected, state, projection);

        // Check if the newMotion reached the goal.
        if (goal->isSatisfied(newMotion->state_, &distanceToGoal))
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
            insertSampleIntoBsp(*m, cellSelected, state, projection);
        iteration_++;
    }
    si_->freeState(state);


    if (lastGoalMotion_ != NULL)
        hasSolution = true;

    // If a solution path has been computed, save it in the problem definition object.
    if (hasSolution)
    {
        PathGeometric *path = new PathGeometric(si_);

        // Compute the path by going up the tree of motions.
        std::vector<Motion*> mpath;
        Motion *solution = lastGoalMotion_;
        while (solution->parent_ != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent_;
        }

        // Add the solution path in order from the start state to the goal.
        for (std::vector<Motion*>::reverse_iterator rIt = mpath.rbegin(); rIt != mpath.rend(); ++rIt)
            path->append((*rIt)->state_);
        pdef_->addSolutionPath(base::PathPtr(path), isApproximate, closestDistanceToGoal);
    }

    return base::PlannerStatus(hasSolution, isApproximate);
}

ompl::geometric::PDST::Motion* ompl::geometric::PDST::propagateFrom(Motion* motion, base::State* state, base::EuclideanProjection& projection)
{
    if (motion->parent_)
    {
        // sample a point along the trajectory given by motion
        double length = rng_.uniform01();
        // split motion into two motions
        motion = motion->split(si_, length, projectionEvaluator_);
        motion->heapElement_ = priorityQueue_.insert(motion);
    }

    // generate a random state
    if (goalSampler_ && rng_.uniform01() < goalBias_ && goalSampler_->canSample())
        goalSampler_->sampleGoal(state);
    else
        sampler_->sampleUniform(state);

    std::pair<base::State*, double> lastValid = std::make_pair(state, 0.);
    si_->checkMotion(motion->state_, state, lastValid);
    projectionEvaluator_->project(state, projection);

    Motion* m = new Motion(si_->cloneState(state), motion, projection, iteration_, bsp_->stab(projection));
    int numSegments = si_->getStateSpace()->validSegmentCount(motion->state_, m->state_);

    m->heapElement_ = priorityQueue_.insert(m);
    m->cell_->motions_.push_back(m);

    if (numSegments > 1)
    {
        si_->getStateSpace()->interpolate(motion->state_, m->state_, 1.0 / (double)numSegments, state);
        projectionEvaluator_->project(state, projection);
        Cell* cell = bsp_->stab(projection);
        while (cell != motion->cell_ && numSegments > 1)
            cell = split(bsp_, cell, m, numSegments, state, projection);
    }
    return m;
}

void ompl::geometric::PDST::insertSampleIntoBsp(Motion *motion, Cell *bsp, base::State* state, base::EuclideanProjection& projection)
{
    bsp->stab(motion->projection_)->addMotion(motion);
    priorityQueue_.update(motion->heapElement_);

    // If the motion corresponds to a start state, then it cannot be split across cell
    // bounds. So we're done.
    if (!motion->parent_)
        return;

    // if cell of this motion and its parent are the same, we are done
    Cell* cell = motion->parent_->cell_;
    if (cell == bsp)
        cell = bsp->stab(motion->parent_->projection_);
    if (cell == motion->cell_)
        return;

    // motion can cross at most one cell boundary, since bsp has just been split
    // into two cells.
    int numSegments = si_->getStateSpace()->validSegmentCount(motion->parent_->state_, motion->state_);

    if (numSegments > 1)
    {
        base::EuclideanProjection projection(projectionEvaluator_->getDimension());
        si_->getStateSpace()->interpolate(motion->parent_->state_, motion->state_, 1.0 / (double)numSegments, state);
        projectionEvaluator_->project(state, projection);
        cell = bsp->stab(projection);
        if (cell != motion->cell_)
            split(bsp, cell, motion, numSegments, state, projection);
    }
}

ompl::geometric::PDST::Cell* ompl::geometric::PDST::split(const Cell* bsp, Cell* startCell, Motion* motion, int& numSegments, base::State* state, base::EuclideanProjection& projection)
{
    Cell* nextStartCell = motion->cell_;
    Cell* cell;
    int i = 1, j = numSegments, k = 1;
    // Find the largest k such that the interpolated state at k/numSegments is
    // still in the startCell. The variables i and j bracket the range that k
    // can be in. Initially, k=1. The loop will never be entered if i=1 and j=2.
    // It is assumed that numSegments>1 (this is checked for in the calling
    // contexts).
    while (j - i > 1)
    {
        k = (i + j) / 2;
        si_->getStateSpace()->interpolate(motion->parent_->state_, motion->state_, (double)k / (double)numSegments, state);
        projectionEvaluator_->project(state, projection);
        cell = bsp->stab(projection);
        if (cell == startCell)
            i = k;
        else
        {
            j = k;
            nextStartCell = cell;
        }
    }

    Motion* m = new Motion(si_->cloneState(state), motion->parent_, projection, iteration_, startCell);
    motion->parent_ = m;
    m->heapElement_ = priorityQueue_.insert(m);
    m->cell_->motions_.push_back(m);
    numSegments -= k;
    // return the cell of the interpolated state at (k+1)/numSegments.
    return nextStartCell;
}

void ompl::geometric::PDST::clear(void)
{
    Planner::clear();
    sampler_.reset();
    iteration_ = 1;
    lastGoalMotion_ = NULL;
    freeMemory();
    bsp_ = new Cell(1.0, projectionEvaluator_->getBounds(), 0);
}

void ompl::geometric::PDST::freeMemory(void)
{
    // Iterate over the elements in the priority queue and clear it
    std::vector<Motion*> motions;
    motions.reserve(priorityQueue_.size());
    priorityQueue_.getContent(motions);
    for (std::vector<Motion*>::iterator it = motions.begin(); it < motions.end(); ++it)
        freeMotion(*it);
    priorityQueue_.clear(); // clears the Element objects in the priority queue
    delete bsp_;
    bsp_ = NULL;
}

void ompl::geometric::PDST::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);
    if (bsp_)
        delete bsp_;
    bsp_ = new Cell(1.0, projectionEvaluator_->getBounds(), 0);
    lastGoalMotion_ = NULL;
}

void ompl::geometric::PDST::getPlannerData(ompl::base::PlannerData &data) const
{
    base::Planner::getPlannerData(data);

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
        else
            data.addEdge(base::PlannerDataVertex(parent->state_),
                base::PlannerDataVertex(child->state_));
    }
}

void ompl::geometric::PDST::Cell::subdivide(unsigned int spaceDimension)
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
