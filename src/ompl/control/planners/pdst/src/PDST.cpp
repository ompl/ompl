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

ompl::control::PDST::PDST(const SpaceInformationPtr &si) : base::Planner(si, "PDST"), siC_(si.get())
{
    Planner::declareParam<double>("goal_bias", this, &PDST::setGoalBias, &PDST::getGoalBias, "0.:.05:1.");
}

ompl::control::PDST::~PDST()
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

    // depending on how the planning problem is set up, this may be necessary
    bsp_->bounds_ = projectionEvaluator_->getBounds();
    base::Goal *goal = pdef_->getGoal().get();
    goalSampler_ = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);

    // Ensure that we have a state sampler AND a control sampler
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    // Initialize to correct values depending on whether or not previous calls to solve
    // generated an approximate or exact solution. If solve is being called for the first
    // time then initializes hasSolution to false and isApproximate to true.
    double distanceToGoal, closestDistanceToGoal = std::numeric_limits<double>::infinity();
    bool hasSolution = lastGoalMotion_ != nullptr;
    bool isApproximate = !hasSolution || !goal->isSatisfied(lastGoalMotion_->endState_, &closestDistanceToGoal);
    unsigned int ndim = projectionEvaluator_->getDimension();

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
    Eigen::VectorXd tmpProj1(ndim), tmpProj2(ndim);
    while (!ptc)
    {
        // Get the top priority path.
        Motion *motionSelected = priorityQueue_.top()->data;
        motionSelected->updatePriority();
        priorityQueue_.update(motionSelected->heapElement_);

        Motion *newMotion = propagateFrom(motionSelected, tmpState1, tmpState2);
        if (newMotion == nullptr)
            continue;

        addMotion(newMotion, bsp_, tmpState1, tmpState2, tmpProj1, tmpProj2);

        // Check if the newMotion reached the goal.
        hasSolution = goal->isSatisfied(newMotion->endState_, &distanceToGoal);
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
        std::vector<Motion *> motions;
        cellSelected->subdivide(ndim);
        motions.swap(cellSelected->motions_);
        for (auto &motion : motions)
            addMotion(motion, cellSelected, tmpState1, tmpState2, tmpProj1, tmpProj2);
    }

    if (lastGoalMotion_ != nullptr)
        hasSolution = true;

    // If a solution path has been computed, save it in the problem definition object.
    if (hasSolution)
    {
        Motion *m;
        std::vector<unsigned int> durations(
            1, findDurationAndAncestor(lastGoalMotion_, lastGoalMotion_->endState_, tmpState1, m));
        std::vector<Motion *> mpath(1, m);

        while (m->parent_)
        {
            durations.push_back(findDurationAndAncestor(m->parent_, m->startState_, tmpState1, m));
            mpath.push_back(m);
        }

        auto path(std::make_shared<PathControl>(si_));
        double dt = siC_->getPropagationStepSize();
        path->append(mpath.back()->endState_);
        for (int i = (int)mpath.size() - 2; i > 0; i--)
            path->append(mpath[i - 1]->startState_, mpath[i]->control_, durations[i] * dt);
        path->append(lastGoalMotion_->endState_, mpath[0]->control_, durations[0] * dt);
        pdef_->addSolutionPath(path, isApproximate, closestDistanceToGoal, getName());
    }

    si_->freeState(tmpState1);
    si_->freeState(tmpState2);

    OMPL_INFORM("%s: Created %u states and %u cells", getName().c_str(), priorityQueue_.size(), bsp_->size());

    return {hasSolution, isApproximate};
}

ompl::control::PDST::Motion *ompl::control::PDST::propagateFrom(Motion *motion, base::State *start, base::State *rnd)
{
    // sample a point along the trajectory given by motion
    unsigned int prevDuration = motion->controlDuration_;
    if (motion->controlDuration_ > 1)
        prevDuration = rng_.uniformInt(1, motion->controlDuration_);
    if (prevDuration == motion->controlDuration_)
        start = motion->endState_;
    else
        siC_->propagate(motion->startState_, motion->control_, prevDuration, start);
    // generate a random state
    if (goalSampler_ && rng_.uniform01() < goalBias_ && goalSampler_->canSample())
        goalSampler_->sampleGoal(rnd);
    else
        sampler_->sampleUniform(rnd);
    // generate a random control
    Control *control = siC_->allocControl();
    unsigned int duration = controlSampler_->sampleTo(control, motion->control_, start, rnd);
    // return new motion if duration is large enough
    if (duration < siC_->getMinControlDuration())
    {
        siC_->freeControl(control);
        return nullptr;
    }
    return new Motion(si_->cloneState(start), si_->cloneState(rnd), control, duration, ++iteration_, motion);
}

void ompl::control::PDST::addMotion(Motion *motion, Cell *bsp, base::State *prevState, base::State *state,
                                    Eigen::Ref<Eigen::VectorXd> prevProj, Eigen::Ref<Eigen::VectorXd> proj)
{
    // If the motion is at most 1 step, then it cannot be split across cell bounds.
    if (motion->controlDuration_ <= 1)
    {
        projectionEvaluator_->project(motion->endState_, proj);
        bsp->stab(proj)->addMotion(motion);
        updateHeapElement(motion);
        return;
    }

    Cell *cell = nullptr, *prevCell = nullptr;
    si_->copyState(prevState, motion->startState_);
    // propagate the motion, check for cell boundary crossings, and split as necessary
    for (unsigned int i = 0, duration = 0; i < motion->controlDuration_ - 1; ++i, ++duration)
    {
        siC_->propagate(prevState, motion->control_, 1, state);
        projectionEvaluator_->project(state, proj);
        cell = bsp->stab(proj);
        if (duration > 0 && cell != prevCell)
        {
            auto *newMotion = new Motion(motion->startState_, si_->cloneState(prevState), motion->control_, duration,
                                         motion->priority_, motion->parent_);
            newMotion->isSplit_ = true;
            prevCell->addMotion(newMotion);
            updateHeapElement(newMotion);
            motion->startState_ = newMotion->endState_;
            motion->controlDuration_ -= duration;
            motion->parent_ = newMotion;
            duration = 0;
        }
        std::swap(state, prevState);
        std::swap(cell, prevCell);
        proj.swap(prevProj);
    }
    prevCell->addMotion(motion);
    updateHeapElement(motion);
}

unsigned int ompl::control::PDST::findDurationAndAncestor(Motion *motion, base::State *state, base::State *scratch,
                                                          Motion *&ancestor) const
{
    const double eps = std::numeric_limits<float>::epsilon();
    unsigned int duration;
    ancestor = motion;
    if (state == motion->endState_ || motion->controlDuration_ == 0 || si_->distance(motion->endState_, state) < eps)
        duration = motion->controlDuration_;
    else if (motion->controlDuration_ > 0 && si_->distance(motion->startState_, state) < eps)
        duration = 0;
    else
    {
        duration = 1;
        si_->copyState(scratch, motion->startState_);
        for (; duration <= motion->controlDuration_; ++duration)
        {
            siC_->propagate(scratch, motion->control_, 1, scratch);
            if (si_->distance(scratch, state) < eps)
                break;
        }
    }
    if (duration <= motion->controlDuration_)
    {
        // ancestor points to the start of a split motion; duration is computed
        // relative to start state of ancestor motion
        while (ancestor->parent_ && ancestor->control_ == ancestor->parent_->control_)
        {
            ancestor = ancestor->parent_;
            duration += ancestor->controlDuration_;
        }
        return duration;
    }
    // motion is no longer the parent of the motion starting at
    // state because it has been split afterwards, so look for
    // the starting point in the parent of motion.
    return findDurationAndAncestor(motion->parent_, state, scratch, ancestor);
}

void ompl::control::PDST::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    iteration_ = 1;
    lastGoalMotion_ = nullptr;
    freeMemory();
    bsp_ = new Cell(1., projectionEvaluator_->getBounds(), 0);
}

void ompl::control::PDST::freeMemory()
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
        {
            si_->freeState(motion->endState_);
            if (motion->control_)
                siC_->freeControl(motion->control_);
        }
        delete motion;
    }
    priorityQueue_.clear();  // clears the Element objects in the priority queue
    delete bsp_;
    bsp_ = nullptr;
}

void ompl::control::PDST::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    if (!projectionEvaluator_->hasBounds())
        projectionEvaluator_->inferBounds();
    if (!projectionEvaluator_->hasBounds())
        throw Exception("PDST requires a projection evaluator that specifies bounds for the projected space");
    if (bsp_)
        delete bsp_;
    bsp_ = new Cell(1., projectionEvaluator_->getBounds(), 0);
    lastGoalMotion_ = nullptr;
}

void ompl::control::PDST::getPlannerData(ompl::base::PlannerData &data) const
{
    base::Planner::getPlannerData(data);

    double dt = siC_->getPropagationStepSize();
    base::State *scratch = si_->allocState();
    std::vector<Motion *> motions;
    motions.reserve(priorityQueue_.size());
    priorityQueue_.getContent(motions);

    // Add goal vertex
    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(lastGoalMotion_->endState_);

    for (auto &motion : motions)
        if (!motion->isSplit_)
        {
            // We only add one edge for each motion that has been split into smaller segments
            Motion *cur = motion, *ancestor;
            unsigned int duration = findDurationAndAncestor(cur, cur->endState_, scratch, ancestor);

            if (cur->parent_ == nullptr)
                data.addStartVertex(base::PlannerDataVertex(cur->endState_));
            else if (data.hasControls())
            {
                data.addEdge(base::PlannerDataVertex(ancestor->startState_), base::PlannerDataVertex(cur->endState_),
                             PlannerDataEdgeControl(cur->control_, duration * dt));
                if (ancestor->parent_)
                {
                    // Include an edge between start state of parent ancestor motion and
                    // the start state of the ancestor motion, which lies somewhere on
                    // the parent ancestor motion.
                    cur = ancestor;
                    duration = findDurationAndAncestor(cur->parent_, cur->startState_, scratch, ancestor);
                    data.addEdge(base::PlannerDataVertex(ancestor->startState_),
                                 base::PlannerDataVertex(cur->startState_),
                                 PlannerDataEdgeControl(ancestor->control_, duration * dt));
                }
            }
            else
            {
                data.addEdge(base::PlannerDataVertex(ancestor->startState_), base::PlannerDataVertex(cur->endState_));
                if (ancestor->parent_)
                {
                    // Include an edge between start state of parent ancestor motion and
                    // the start state of the ancestor motion, which lies somewhere on
                    // the parent ancestor motion.
                    cur = ancestor;
                    findDurationAndAncestor(cur->parent_, cur->startState_, scratch, ancestor);
                    data.addEdge(base::PlannerDataVertex(ancestor->startState_),
                                 base::PlannerDataVertex(cur->startState_));
                }
            }
        }

    si_->freeState(scratch);
}

void ompl::control::PDST::Cell::subdivide(unsigned int spaceDimension)
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
