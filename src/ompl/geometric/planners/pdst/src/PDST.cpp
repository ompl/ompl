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

ompl::geometric::PDST::PDST(const base::SpaceInformationPtr &si) : base::Planner(si, "PDST")
{
    Planner::declareParam<double>("goal_bias", this, &PDST::setGoalBias, &PDST::getGoalBias, "0.:.05:1.");
}

ompl::geometric::PDST::~PDST()
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

    if (bsp_ == nullptr)
        throw Exception("PDST was not set up.");

    // depending on how the planning problem is set up, this may be necessary
    bsp_->bounds_ = projectionEvaluator_->getBounds();

    base::Goal *goal = pdef_->getGoal().get();
    goalSampler_ = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);

    // Ensure that we have a state sampler
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // Initialize to correct values depending on whether or not previous calls to solve
    // generated an approximate or exact solution. If solve is being called for the first
    // time then initializes hasSolution to false and isApproximate to true.
    double distanceToGoal, closestDistanceToGoal = std::numeric_limits<double>::infinity();
    bool hasSolution = lastGoalMotion_ != nullptr;
    bool isApproximate = !hasSolution || !goal->isSatisfied(lastGoalMotion_->endState_, &closestDistanceToGoal);
    unsigned ndim = projectionEvaluator_->getDimension();

    // If an exact solution has already been found, do not run for another iteration.
    if (hasSolution && !isApproximate)
        return ompl::base::PlannerStatus::EXACT_SOLUTION;

    // Initialize tree with start state(s)
    while (const base::State *st = pis_.nextStart())
    {
        auto *startMotion = new Motion(si_->cloneState(st));
        bsp_->addMotion(startMotion);
        startMotion->heapElement_ = priorityQueue_.insert(startMotion);
    }

    if (priorityQueue_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                priorityQueue_.size());

    base::State *tmpState1 = si_->allocState(), *tmpState2 = si_->allocState();
    Eigen::VectorXd tmpProj(ndim);
    while (!ptc)
    {
        // Get the top priority path.
        Motion *motionSelected = priorityQueue_.top()->data;
        motionSelected->updatePriority();
        priorityQueue_.update(motionSelected->heapElement_);

        Motion *newMotion = propagateFrom(motionSelected, tmpState1, tmpState2);
        addMotion(newMotion, bsp_, tmpState1, tmpProj);

        // Check if the newMotion reached the goal.
        hasSolution = goal->isSatisfied(newMotion->endState_, &distanceToGoal);
        if (hasSolution)
        {
            closestDistanceToGoal = distanceToGoal;
            lastGoalMotion_ = newMotion;
            isApproximate = false;
            break;
        }
        if (distanceToGoal < closestDistanceToGoal)
        {
            closestDistanceToGoal = distanceToGoal;
            lastGoalMotion_ = newMotion;
        }

        // subdivide cell that contained selected motion, put motions of that
        // cell in subcells and split motions so that they contained within
        // one subcell
        Cell *cellSelected = motionSelected->cell_;
        std::vector<Motion *> motions;
        cellSelected->subdivide(ndim);
        motions.swap(cellSelected->motions_);
        for (auto &motion : motions)
            addMotion(motion, cellSelected, tmpState1, tmpProj);
    }

    if (lastGoalMotion_ != nullptr)
        hasSolution = true;

    // If a solution path has been computed, save it in the problem definition object.
    if (hasSolution)
    {
        auto path(std::make_shared<PathGeometric>(si_));

        // Compute the path by going up the tree of motions.
        std::vector<base::State *> spath(1, lastGoalMotion_->endState_);
        Motion *m = lastGoalMotion_;
        while (m != nullptr)
        {
            m = m->ancestor();
            spath.push_back(m->startState_);
            m = m->parent_;
        }

        // Add the solution path in order from the start state to the goal.
        for (auto rIt = spath.rbegin(); rIt != spath.rend(); ++rIt)
            path->append((*rIt));
        pdef_->addSolutionPath(path, isApproximate, closestDistanceToGoal, getName());
    }

    si_->freeState(tmpState1);
    si_->freeState(tmpState2);

    OMPL_INFORM("%s: Created %u states and %u cells", getName().c_str(), priorityQueue_.size(), bsp_->size());

    return {hasSolution, isApproximate};
}

ompl::geometric::PDST::Motion *ompl::geometric::PDST::propagateFrom(Motion *motion, base::State *start,
                                                                    base::State *rnd)
{
    // sample a point along the trajectory given by motion
    si_->getStateSpace()->interpolate(motion->startState_, motion->endState_, rng_.uniform01(), start);
    // generate a random state
    if ((goalSampler_ != nullptr) && rng_.uniform01() < goalBias_ && goalSampler_->canSample())
        goalSampler_->sampleGoal(rnd);
    else
        sampler_->sampleUniform(rnd);
    // compute longest valid segment from start towards rnd
    std::pair<base::State *, double> lastValid = std::make_pair(rnd, 0.);
    si_->checkMotion(start, rnd, lastValid);
    return new Motion(si_->cloneState(start), si_->cloneState(rnd), ++iteration_, motion);
}

void ompl::geometric::PDST::addMotion(Motion *motion, Cell *bsp, base::State *state, Eigen::Ref<Eigen::VectorXd> proj)
{
    projectionEvaluator_->project(motion->endState_, proj);
    bsp->stab(proj)->addMotion(motion);
    updateHeapElement(motion);

    // If the motion corresponds to a start state, then it cannot be split across cell
    // bounds. So we're done.
    if (motion->parent_ == nullptr)
        return;

    // Otherwise, split into smaller segments s.t. endpoints project into same cell
    int numSegments = si_->getStateSpace()->validSegmentCount(motion->startState_, motion->endState_);
    Cell *startCell;
    projectionEvaluator_->project(motion->startState_, proj);
    startCell = bsp->stab(proj);

    while (startCell != motion->cell_ && numSegments > 1)
    {
        Cell *nextStartCell = motion->cell_, *cell;
        int i = 0, j = numSegments, k = 1;
        // Find the largest k such that the interpolated state at k/numSegments is
        // still in startCell. The variables i and j bracket the range that k
        // can be in.
        while (j - i > 1)
        {
            k = (i + j) / 2;
            si_->getStateSpace()->interpolate(motion->startState_, motion->endState_, (double)k / (double)numSegments,
                                              state);
            projectionEvaluator_->project(state, proj);
            cell = bsp->stab(proj);
            if (cell == startCell)
                i = k;
            else
            {
                j = k;
                nextStartCell = cell;
            }
        }

        auto *m = new Motion(motion->startState_, si_->cloneState(state), motion->priority_, motion->parent_);
        startCell->addMotion(m);
        m->heapElement_ = priorityQueue_.insert(m);
        m->isSplit_ = true;
        motion->startState_ = m->endState_;
        motion->parent_ = m;
        numSegments -= k;
        startCell = nextStartCell;
    }
}

void ompl::geometric::PDST::clear()
{
    Planner::clear();
    sampler_.reset();
    iteration_ = 1;
    lastGoalMotion_ = nullptr;
    freeMemory();
    if (projectionEvaluator_ && projectionEvaluator_->hasBounds())
        bsp_ = new Cell(1., projectionEvaluator_->getBounds(), 0);
}

void ompl::geometric::PDST::freeMemory()
{
    // Iterate over the elements in the priority queue and clear it
    std::vector<Motion *> motions;
    motions.reserve(priorityQueue_.size());
    priorityQueue_.getContent(motions);
    for (auto &motion : motions)
    {
        if (motion->startState_ != motion->endState_)
            si_->freeState(motion->startState_);
        if (!motion->isSplit_)
            si_->freeState(motion->endState_);
        delete motion;
    }
    priorityQueue_.clear();  // clears the Element objects in the priority queue
    delete bsp_;
    bsp_ = nullptr;
}

void ompl::geometric::PDST::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    if (!projectionEvaluator_->hasBounds())
        projectionEvaluator_->inferBounds();
    if (!projectionEvaluator_->hasBounds())
        throw Exception("PDST requires a projection evaluator that specifies bounds for the projected space");
    if (bsp_ != nullptr)
        delete bsp_;
    bsp_ = new Cell(1., projectionEvaluator_->getBounds(), 0);
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::PDST::getPlannerData(ompl::base::PlannerData &data) const
{
    base::Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    priorityQueue_.getContent(motions);

    // Add goal vertex
    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(lastGoalMotion_->endState_);

    for (const auto &cur : motions)
        if (!cur->isSplit_)
        {
            Motion *ancestor = cur->ancestor();
            if (cur->parent_ == nullptr)
                data.addStartVertex(base::PlannerDataVertex(cur->endState_));
            else
            {
                data.addEdge(base::PlannerDataVertex(ancestor->startState_), base::PlannerDataVertex(cur->endState_));
                // add edge connecting start state of ancestor's parent to start state of ancestor
                if (ancestor->parent_ != nullptr)
                    data.addEdge(base::PlannerDataVertex(ancestor->parent_->ancestor()->startState_),
                                 base::PlannerDataVertex(ancestor->startState_));
            }
        }
}

void ompl::geometric::PDST::Cell::subdivide(unsigned int spaceDimension)
{
    double childVolume = .5 * volume_;
    unsigned int nextSplitDimension = (splitDimension_ + 1) % spaceDimension;
    splitValue_ = .5 * (bounds_.low[splitDimension_] + bounds_.high[splitDimension_]);

    left_ = new Cell(childVolume, bounds_, nextSplitDimension);
    left_->bounds_.high[splitDimension_] = splitValue_;
    left_->motions_.reserve(motions_.size());
    right_ = new Cell(childVolume, bounds_, nextSplitDimension);
    right_->bounds_.low[splitDimension_] = splitValue_;
    right_->motions_.reserve(motions_.size());
}
