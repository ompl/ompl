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

/* Author: Jonathan Sobieski */

#include "ompl/control/planners/pdst/PDST.h"



/* PDST member definitions *******************************************************************************/

#ifdef PDST_DEBUG
unsigned int ompl::control::PDST::Motion::SPLIT_COUNT;
unsigned int ompl::control::PDST::Motion::MOTION_COUNT;
#endif

ompl::base::PlannerStatus
ompl::control::PDST::solve(const base::PlannerTerminationCondition &ptc)
{
    // Make sure the planner is configured correctly.
    // ompl::base::Planner::checkValidity checks that there is at least one
    // start state and an ompl::base::Goal object specified and throws an
    // exception if this is not the case.
    checkValidity();

    // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
    base::Goal *goal = pdef_->getGoal().get();

    // Planner::checkValidity() should have validated that the goal is not null.
    assert(goal != NULL);

    goalSampler_ = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);

    // Initialize to correct values depending on whether or not previous calls to solve
    // generated an approximate or exact solution. If solve is being called for the first
    // time then initializes hasSolution to false and isApproximate to true.
    double closestDistanceToGoal = std::numeric_limits<double>::infinity();
    bool hasSolution = (closestMotionToGoal_ == NULL) ? false : true;
    bool isApproximate = hasSolution ?
    ((goal->isSatisfied(closestMotionToGoal_->getState(), &closestDistanceToGoal)) ? true : false) :
    true;

    // If an exact solution has already been found, do not run for another iteration.
    if (hasSolution && !isApproximate)
    {
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
    }

#ifndef NDEBUG
    int startStateCount = pdef_->getStartStateCount();

    // Planner::checkValidity() should have validated that there are some start states.
    assert(startStateCount > 0);
#endif

    // Store the start states in startMotions.
    while (const base::State *st = pis_.nextStart())
    {
    // Compute the projection of the starting state.
    ompl::base::EuclideanProjection *startProjection = new ompl::base::EuclideanProjection();
    projectionEvaluator_->project(st, *startProjection);

    // A starting Motion has the given start state, no parent, no control, 0 control duration,
    // priority of 1.0, and the computed projection.
    Motion *startMotion = new Motion(st, NULL, NULL, 0, 1.0, startProjection);

    // Add the starting motion to the initial cell and update the initial
    // path's cell pointer.
    bsp_->addMotion(startMotion);
    startMotion->setCell(bsp_);

    // Add the start path to the priority queue
    ompl::BinaryHeap<Motion *, MotionCompare>::Element *startHeapElement = priorityQueue_.insert(startMotion);
    startMotion->setHeapElement(startHeapElement);

    startMotions_.push_back(startMotion);
    }

#ifdef PDST_DEBUG
    // TODO: Remove this lastSelectedScore code since it was only used for debugging purposes.
    double lastSelectedScore = 1.0;
#endif

    for( /* iter_ initialized in PDST constructor */ ; ptc() == false; iter_++)
    {
    //std::cout << "Next iteration " << iter_ << " of solve" << std::endl;
    //std::cout << "priorityQueue_->size() == " << priorityQueue_.size() << std::endl;

    // Start planning here.
    // call routines from SpaceInformation (si_) as needed. i.e.,
    // si_->allocStateSampler() for sampling,
    // si_->checkMotion(state1, state2) for state validity, etc...
    // use the Goal pointer to evaluate whether a sampled state satisfies the goal requirements
    // use log macros for informative messaging, i.e., logInfo("Planner found a solution!");

    // Get the top priority path.
    Motion *motionSelected = priorityQueue_.top()->data;
    priorityQueue_.pop();

#ifdef PDST_DEBUG
    // TODO: Remove when done debugging
    //std::cout << "Attempting to propagate from motion:" << std::endl;
    //motionSelected->printMotion(siC_, std::cout);
    //std::cout << "motionSelected->computeScore() = " << motionSelected->computeScore() << std::endl;
    //std::cout << "motionSelected->getPriority() = " << motionSelected->getPriority() << std::endl;

    // TODO: Remove this code when done debugging
    assert(!((motionSelected->computeScore() < lastSelectedScore) && (motionSelected->getPriority() != iter_ - 1)));
    lastSelectedScore = motionSelected->computeScore();
#endif

#ifdef PDST_DEBUG
    //std::cout << "Projecting selected motion." << std::endl;
#endif

    ompl::base::EuclideanProjection euclideanProjection;

#ifdef PDST_DEBUG
    /*
    std::cout << "euclideanProjection.size() = " << euclideanProjection.size() << std::endl;
    std::cout << "euclideanProjection.max_size() = " << euclideanProjection.max_size() << std::endl;
    std::cout << "euclideanProjection.empty() = " << euclideanProjection.empty() << std::endl;
    */
#endif
    
    projectionEvaluator_->project(motionSelected->getState(), euclideanProjection);

#ifdef PDST_DEBUG
    /*
    std::cout << "Printing projection of selected motion." << std::endl;
    int size = euclideanProjection.size();
    std::cout << "projection size = " << size << std::endl;

    for (int i = 0; i < size; i++)
    {
        std::cout << "projection[i] = " << euclideanProjection[i] << std::endl;
    }
    */

    /*
    for (ompl::base::EuclideanProjection::iterator it = euclideanProjection.begin(); it < euclideanProjection.end(); ++it)
    {
        std::cout << "projection value = " << *it << std::endl;
    }
    */
#endif

    // Propagate a new path from the next path
    Motion *newMotion = propagateFrom(&motionSelected);
    if (newMotion == NULL)
    {
#ifdef PDST_DEBUG
        //std::cout << "WARNING: propagateFrom failed to propagate." << std::endl;
        motionSelected->incrementPropagationFailureCount(); // TODO: Remove this line when done debugging
#endif
        motionSelected->setPriority((motionSelected->getPriority() * 2) + 1);
        ompl::BinaryHeap<Motion *, MotionCompare>::Element *heapElement = priorityQueue_.insert(motionSelected);
        motionSelected->setHeapElement(heapElement);
        continue;
    }
#ifdef PDST_DEBUG
    else
    {
        // TODO: Remove when done debugging
        /*
        std::cout << "Information for newly generated motion:" << std::endl;
        std::cout << "Parent motion information:" << std::endl;
        motionSelected->printMotion(siC_, std::cout);
        std::cout << "New motion information:" << std::endl;
        newMotion->printMotion(siC_, std::cout);
        */
    }
#endif

    // Check if the newMotion reached the goal.
    double distanceToGoal;
    if (goal->isSatisfied(newMotion->getState(), &distanceToGoal))
    {
        closestDistanceToGoal = distanceToGoal;
        closestMotionToGoal_ = newMotion;
        hasSolution = true;
        isApproximate = false;

#ifdef PDST_DEBUG
        // If debugging print out the solution path
        /*
        std::cout << "PDST found a path!" << std::endl;

        std::cout << "\n\nPRINTING SOLUTION PATH:" << std::endl;

        int solutionStateCount = 1;
        for (Motion *solution = newMotion; true; solution = solution->getParent(), solutionStateCount++)
        {
        std::cout << "\nSolution State: " << solutionStateCount << std::endl;
        solution->printMotion(siC_, std::cout);

        if (solution->getMotionId() == 0)
        {
            break;
        }
        }

        std::cout << "END OF SOLUTION PATH" << std::endl;
        
        std::cout << "\n\nPROBLEM SOLVED!" << std::endl;
        std::cout << "Ran for " << iter_ << " iterations." << std::endl;
        std::cout << "priorityQueue.size() = " << priorityQueue_.size() << std::endl;
        std::cout << "propagateFailureCount_ = " << propagateFailureCount_ << std::endl;
        std::cout << "Motion::SPLIT_COUNT = " << Motion::getSplitCount() << std::endl;
        std::cout << "solutionStateCount = " << solutionStateCount << std::endl;
        std::cout << "Final State:" << std::endl;
        siC_->printState(newMotion->getState(), std::cout);

        newMotion->printMotion(siC_, std::cout);
        */
#endif

        // Go to control flow after the main for loop
        break;
    }
    else if (distanceToGoal < closestDistanceToGoal)
    {
        closestDistanceToGoal = distanceToGoal;
        closestMotionToGoal_ = newMotion;
    }
    

    // Penalize the path from which PDST just propagated.
    motionSelected->setPriority(2 * motionSelected->getPriority() + 1);

    // Initialize priority of new sample.
    newMotion->setPriority(iter_);

    // Insert newMotion into the appropriate cell and priority queue, splitting if necessary.
    insertSampleIntoBsp(newMotion);

    ompl::BinaryHeap<Motion *, MotionCompare>::Element *newMotionHeapElement = priorityQueue_.insert(newMotion);
    newMotion->setHeapElement(newMotionHeapElement);
    ompl::BinaryHeap<Motion *, MotionCompare>::Element *motionSelectedHeapElement = priorityQueue_.insert(motionSelected);
    motionSelected->setHeapElement(motionSelectedHeapElement);

    // Get the cell that contained motionSelected
    Cell *cellSelected = motionSelected->getCell();

    // Use the selected cell's list of paths as the reinsertion list
    std::list<Motion *> *reinsertionList = cellSelected->getMotions();

    // TODO: Remove when done debugging
    assert(reinsertionList != NULL);

    // Reinitialize the selected cell's list of paths to a new empty list and subdivide.
    cellSelected->reinitializePaths();
    cellSelected->subdivide(projectionDimension_);

#ifdef PDST_DEBUG
    //std::cout << "Finished subdivide. About to reinsert samples." << std::endl;
#endif

    // For each path in the reinsertion list, reinsert
    while (!reinsertionList->empty())
    {
#ifdef PDST_DEBUG
        //std::cout << "Running a reinsertion iteration" << std::endl;
#endif

        Motion *motionToReinsert = reinsertionList->front();

#ifdef PDST_DEBUG
        //std::cout << "Attempting to reinsert motion:" << std::endl;
        //motionToReinsert->printMotion(siC_, std::cout);

        // TODO: Remove this when done debugging
        double scoreBefore = motionToReinsert->computeScore();
#endif

        insertSampleIntoBsp(motionToReinsert, cellSelected);

#ifdef PDST_DEBUG
        double scoreAfter = motionToReinsert->computeScore();

        /*
        if (!(scoreBefore < scoreAfter))
        {
        std::cout << "ERROR: scoreBefore = " << scoreBefore << " >= scoreAfter = " << scoreAfter << std::endl;
        exit(-1);
        }
        */
        assert(scoreBefore == std::numeric_limits<double>::infinity() || scoreBefore < scoreAfter);
        assert(scoreAfter == 2.0 * scoreBefore);

        //std::cout << "About to pop motion from list" << std::endl;
#endif

        reinsertionList->pop_front();

#ifdef PDST_DEBUG
        // TODO: Remove when done debugging
        assert(motionToReinsert->getHeapElement() != NULL);
#endif

        priorityQueue_.update(motionToReinsert->getHeapElement());
    }

    delete reinsertionList;

#ifdef PDST_DEBUG
    //std::cout << "Completed reinsertionList while loop" << std::endl;
#endif
    }

#ifdef PDST_DEBUG
    std::cout << "Ran for " << iter_ << " iterations." << std::endl;
    std::cout << "priorityQueue.size() = " << priorityQueue_.size() << std::endl;
#endif

    std::cout << "Ran for " << iter_ << " iterations." << std::endl;
    std::cout << "priorityQueue.size() = " << priorityQueue_.size() << std::endl;

#ifdef PDST_DEBUG
    std::cout << "propagateFailureCount_ = " << propagateFailureCount_ << std::endl;
    std::cout << "Motion::SPLIT_COUNT = " << Motion::getSplitCount() << std::endl;
#endif

    if (closestMotionToGoal_ != NULL)
    {
    hasSolution = true;
    }

    // If a solution path has been computed, save it in the problem definition object.
    if (hasSolution)
    {
    assert(closestMotionToGoal_ != NULL);

#ifdef PDST_DEBUG    
    //std::cout << "hasSolution == true" << std::endl;
#endif
    
    // Get the time step used to convert discrete motion steps to time durations.
    double propagationStepSize = siC_->getPropagationStepSize();

    // TODO: Consider changing this code to look more like the code for the other
    // planners and use the overloaded operators rather than the const iterator
    // C++ interface to std::vector.

    // TODO: Tell Mark that the documentation for PathControl.append() is rather
    // poorly worded.

    PathControl *solutionPath = new PathControl(si_);

    // Compute the path by recursively going up the tree of motions.
    std::vector<Motion *> solutionMotions;
    for (Motion *solutionMotion = closestMotionToGoal_;
         solutionMotion != NULL;
         solutionMotion = solutionMotion->getParent())
    {
#ifdef PDST_DEBUG
        //std::cout << "Adding Motion:"  << std::endl;
        //siC_->printState(solutionMotion->getState());
#endif
        solutionMotions.push_back(solutionMotion);
    }

#ifdef PDST_DEBUG
    //std::cout << "After solutionMotions loop" << std::endl;
#endif

    // Add the solution path in order from the start state to the goal.
    for (std::vector<Motion *>::reverse_iterator rIt = solutionMotions.rbegin();
         rIt != solutionMotions.rend();
         ++rIt)
    {
        if ((*rIt)->getParent() != NULL)
        {
        // NOTE: I tested that (double = unsigned int * double) does not
        // result in cast errors
        solutionPath->append((*rIt)->getState(), (*rIt)->getControl(),
                    (*rIt)->getControlDuration() * propagationStepSize);
        }
        else
        {
        solutionPath->append((*rIt)->getState());
        }
    }

#ifdef PDST_DEBUG
    //std::cout << "After solutionPath loop" << std::endl;
#endif
    
    pdef_->addSolutionPath(ompl::base::PathPtr(solutionPath), isApproximate, closestDistanceToGoal);
    std::cout << "closestDistanceToGoal = " << closestDistanceToGoal << std::endl;
    }
    
    // Return a value from the PlannerStatus enumeration.
    // See ompl::base::PlannerStatus for the possible return values
    return ompl::base::PlannerStatus(hasSolution, isApproximate);
}

ompl::control::PDST::Motion *
ompl::control::PDST::propagateFrom(Motion **startMotion)
{
#ifdef PDST_DEBUG
    //std::cout << "Called propagateFrom" << std::endl;
#endif

    // assert input is valid
    assert(!(startMotion == NULL || (*startMotion) == NULL));
    
    // randomly generate a time from which to propagate
    unsigned int duration = (*startMotion)->getControlDuration();

    // If duration is 0 this is the root Motion of the tree and cannot be split
    // If duration is 1 then this Motion cannot be split.
    if (duration > 1)
    {
    unsigned int randomDuration = rng_.uniformInt(1, duration);

    // If the randomly sampled duration to extend from was not equal to
    // the length of this motion then split this motion and propagate from the split point..
    if (randomDuration != duration)
    {
#ifdef PDST_DEBUG
        //std::cout << "propgateFrom is splitting the start motion for propagation." << std::endl;

        /*
        std::cout << "Before Split:" << std::endl;
        (*startMotion)->printMotion(siC_);
        */
#endif

        // TODO: Validate that splitting a path in half every time you propagate from the middle of
        // the path does not affect the probabilistic completeness proof for the PDST motion planner

        // Split this motion so that this motion ends at the point from which to propagate.
        // TODO: Note: The return value from split is the second half of the startMotion which is
        // currently ignored and safely reinserted into the planner. It is probably better code
        // design if split doesn't manage adding the new motion to the priority queue or binary
        // space partition.
        Motion *firstPart =    (*startMotion)->split(randomDuration,
                              true,
                              (ompl::control::SpaceInformation *)si_.get(),
                              projectionEvaluator_);

        ompl::BinaryHeap<Motion *, MotionCompare>::Element *startMotionHeapElement = priorityQueue_.insert(*startMotion);
        (*startMotion)->setHeapElement(startMotionHeapElement);

#ifdef PDST_DEBUG
        /*
        std::cout << "Result of splitting:" << std::endl;
        std::cout << "startMotion:" << std::endl;
        (*startMotion)->printMotion(siC_);
        std::cout << "firstPart:" << std::endl;
        firstPart->printMotion(siC_);
        */
#endif

        *startMotion = firstPart;
    }
    }

    // randomly generate a nearby state
    ompl::base::State *randomNearbyState = si_->allocState();

    assert(randomNearbyState != NULL);
    assert(sampler_ != NULL);
    assert((*startMotion) != NULL);
    assert(randomNearbyState != NULL);
    assert((*startMotion)->getState() != NULL);

    if (goalSampler_ && rng_.uniform01() < goalBias_ && goalSampler_->canSample())
    {
    goalSampler_->sampleGoal(randomNearbyState);
    }
    else
    {
    bool b = sampler_->sampleNear(randomNearbyState, (*startMotion)->getState(), maxDistance_);
    if (!b)
    {
        OMPL_WARN("PDST unable to sample a random nearby state using "
            "ompl::base::ValidStateSampler.sampleNear(State *state, State *near, const double distance)");
        si_->freeState(randomNearbyState);
        return NULL;
    }
    }

    // randomly generate a control
    Control *randomControl = controlSpace_->allocControl();
    duration = controlSampler_->sampleTo(randomControl, (*startMotion)->getControl(),
                     (*startMotion)->getState(), randomNearbyState);


    // Attempt to propagate the system using the random control
    duration = siC_->propagateWhileValid((*startMotion)->getState(), randomControl, duration, randomNearbyState);

#ifdef PDST_DEBUG
    //std::cout << "propagateWhileValid duration = " << duration << std::endl;

    //std::cout << "siC_->getMinControlDuration = " << siC_->getMinControlDuration() << std::endl;
#endif

    if (duration >= siC_->getMinControlDuration())
    {
    ompl::base::EuclideanProjection *newMotionProjection = new ompl::base::EuclideanProjection();
    projectionEvaluator_->project(randomNearbyState, *newMotionProjection);

    // Create the new motion. The appropriate priority will need to be set in the calling function
    Motion *newMotion = new Motion(randomNearbyState,
                       (*startMotion),
                       randomControl,
                       duration,
                       0,
                       newMotionProjection);

    // return the newly propagated Motion
    return newMotion;
    }
    else
    {
    // It is reasonable that in some motion planning problems the propagation
    // step might fail sometimes.

#ifdef PDST_DEBUG
    // TODO: Remove the propagateFailureCount_ variable when done debugging.
    // TODO: Ask Mark if I should leave this in the planner interface. I want to because it is
    // extremely helpful for debugging, purposes. However it really isn't necessary as long as
    // you have the problem set up correctly.
    propagateFailureCount_++;
#endif

    // If the propagation step succeeds then randomNearbyState and randomControl are
    // used to hold the new Motion's information - otherwise the memory should be reclaimed.
    si_->freeState(randomNearbyState);
    controlSpace_->freeControl(randomControl);

    return NULL;
    }
}

// Insert the sample motion into the correct cell and adjust its priority in the
// priority queue.
void
ompl::control::PDST::insertSampleIntoBsp(Motion *motion, Cell *bsp)
{
    if (bsp == NULL)
    {
    bsp = bsp_;
    }

    assert(motion != NULL);
    
    int intermediateStateCount = motion->getControlDuration();
    if (intermediateStateCount <= 1)
    {
    // If the motion is only 1 step, then no need to worry about splitting it
    // across cell boundaries. Simply add it to the BSP.
    Cell *cell = bsp->stab(motion->getProjection());
    assert(cell != NULL);
    motion->setCell(cell);
    cell->addMotion(motion);
    return;
    }

    // The motion to insert into the BSP is more than one step long and may
    // need to be split across cell boundaries. Compute the intermediate states
    // and check for crossings of cell boundaries, splitting as necessary.

    std::vector<base::State *> intermediateStates;
    
#ifdef PDST_DEBUG
    //std::cout << "Before siC_->propagate" << std::endl;
#endif
    siC_->propagate(motion->getParent()->getState(), motion->getControl(),
            intermediateStateCount, intermediateStates, true);
#ifdef PDST_DEBUG
    //std::cout << "After siC_->propagate" << std::endl;
#endif

    std::vector<ompl::base::EuclideanProjection *> intermediateProjections;
    intermediateProjections.reserve(intermediateStateCount);
    for (int stateIdx = 0; stateIdx < intermediateStateCount; stateIdx++)
    {
    ompl::base::EuclideanProjection *proj = new ompl::base::EuclideanProjection();
    projectionEvaluator_->project(intermediateStates[stateIdx], *proj);
    intermediateProjections.push_back(proj);
    }

    // Iterate over all states checking for border crossings and split the Motion as necessary.
    Cell *startingCell = bsp->stab(intermediateProjections[0]);
    int lastSplitIdx = 0;
    for (int durationIdx = 1; durationIdx < intermediateStateCount; durationIdx++)
    {
    Cell *intermediateCell = bsp->stab(intermediateProjections[durationIdx]);
    if (intermediateCell != startingCell)
    {
        // Split the cell. This sets motion->parent_ to be the newly split path.
        // motion->split initializes the new Motion with the correct cell.
        motion->split(durationIdx - lastSplitIdx, false, (ompl::control::SpaceInformation *)si_.get(),
              projectionEvaluator_, intermediateStates[durationIdx - 1],
              intermediateProjections[durationIdx - 1], startingCell);

        // Add the new Motion to the priority queue.
        ompl::BinaryHeap<Motion *, MotionCompare>::Element *parentHeapElement = priorityQueue_.insert(motion->getParent());
        motion->getParent()->setHeapElement(parentHeapElement);

        // Update startingCell and lastSplitIndex for next iteration of loop
        startingCell = intermediateCell;
        lastSplitIdx = durationIdx;
    }
    else
    {
        delete intermediateProjections[durationIdx - 1];
        siC_->freeState(intermediateStates[durationIdx - 1]);
    }
    }

    if (lastSplitIdx != intermediateStateCount - 1)
    {
    delete intermediateProjections[intermediateStateCount - 1];
    siC_->freeState(intermediateStates[intermediateStateCount - 1]);
    }

    // At the end of the loop the motion is the remaining part that is all contained within a Cell.
    motion->setCell(startingCell);
    startingCell->addMotion(motion);
}

// Clear all internal datastructures.
// Planner settings are not affected.
// Subsequent calls to solve() will ignore all previous work.
// After clear runs the planner should be able to run in an attempt to solve
// the exact same motion planning problem that it was configured to solve before
// however any progress that it made in previous attempts, if any previous attempts occured
// to finding a solution should be discarded.
void
ompl::control::PDST::clear(void)
{
#ifdef PDST_DEBUG
    // TODO: Remove when done debugging
    std::cout << "Called clear" << std::endl;
#endif

    // TODO: Write a good implementation of clear
    // This might be done already, just need to quickly rethink through it
    Planner::clear();
    // clear the data structures here
    // TODO: Ask Mark if clear should reset a pointer wrapper. Does that make it NULL?
    // TODO: Ask Mark if clear should reset everything to the point that all setup must
    // be called again, or should the planner be able to just automatically call solve and work?
    //sampler_.reset();
    //controlSampler_.reset();
    startMotions_.clear();
    iter_ = 1;
    closestMotionToGoal_ = NULL;
#ifdef PDST_DEBUG
    propagateFailureCount_ = 0;
#endif

    freeMemory();
    if (setup_)
    {
    // Rerun setup to ensure that the planner is ready to run again.
    setup();
    }
}

void
ompl::control::PDST::freeMemory(void)
{
    // Iterate over the elements in the priority queue and clear it
    std::vector<Motion *> motions;
    motions.resize(priorityQueue_.size());
    priorityQueue_.getContent(motions);
    for (std::vector<Motion *>::iterator it = motions.begin(); it < motions.end(); ++it)
    {
    delete *it;
    }
    priorityQueue_.clear(); // clears the Element objects in the priority queue
    
    // delete the bsp
    delete bsp_;
    bsp_ = NULL;
}

void
ompl::control::PDST::checkValidity(void)
{
    Planner::checkValidity();

    if (projectionDimension_ <= 0)
    {
    //("PDST projection dimension was not set. You must call setProjectionDimension before using PDST.");
    throw "PDST projection dimension was not set. You must call setProjectionDimension before using PDST.";
    }

    if (projectionSpaceBoundaries_ == NULL)
    {
    //("PDST projection space boundaries must be set. You must call setProjectionSpaceBoundaries before using PDST.");
    throw "PDST projection space boundaries must be set. You must call setProjectionSpaceBoundaries before using PDST.";
    }

    if (projectionSpaceBoundaries_->size() != projectionDimension_ * 2)
    {
#ifdef PDST_DEBUG
    std::cout << "PDST projectionSpaceBoundaries_ size does not match projection dimension size." << std::endl;
    std::cout << "projectionSpaceBoundaries_->size() = " << projectionSpaceBoundaries_->size() << std::endl;
    std::cout << "projectionDimension_ = " << projectionDimension_ << std::endl;
    for (unsigned int i = 0; i < projectionSpaceBoundaries_->size(); i++)
    {
        std::cout << i << " = " << projectionSpaceBoundaries_->at(i) << std::endl;
    }
#endif
    //("PDST projectionSpaceBoundaries_ size does not match projection dimension size.");
    throw "PDST projectionSpaceBoundaries_ size does not match projection dimension size.";
    }

    for (unsigned int dimIdx = 0; dimIdx < projectionDimension_; dimIdx++)
    {
    if ((*projectionSpaceBoundaries_)[2 * dimIdx] >= (*projectionSpaceBoundaries_)[2 * dimIdx + 1])
    {
        //("PDST projection space boundaries have lower bound greater than or equal to upper bound of dimension");
        throw "PDST projection space boundaries have lower bound greater than or equal to upper bound of dimension";
    }
    }
}

// optional, if additional setup/configuration is needed, the setup() method can be implemented
void
ompl::control::PDST::setup(void)
{
#ifdef PDST_DEBUG
    // TODO: Remove when done debugging
    std::cout << "Called setup" << std::endl;
#endif

    Planner::setup();
#ifdef PDST_DEBUG
    Motion::setup();
#endif

    // TODO: EST has some code using the tools::SelfConfig class.
    // TODO: Ask Mark if self config should matter. It seems like it really doesn't.
    // Look into whether this is necessary to run here.
    // perhaps attempt some auto-configuration
    /*
      SelfConfig sc(si_, getName());
      sc.configure...
    */

    // Set the value of projectionDimension_.
    // TODO: Determine if this really needs to be done this way.
    // TODO: If I leave this line of code then the planner getters and setters for
    // projectionDimension_ should be ignored and this should become a stack variable which
    // is set at the beginnig of calls to solve rather than a planner variable.
    projectionDimension_ = projectionEvaluator_->getDimension();
#ifdef PDST_DEBUG
    std::cout << "projectionDimension_ = " << projectionDimension_ << std::endl;
#endif

    // Initialize the Binary Space Partition.
    std::vector<double> *cellBoundaries = new std::vector<double>(*projectionSpaceBoundaries_);
    bsp_ = new Cell(1.0, cellBoundaries, 0);

    closestMotionToGoal_ = NULL;
}

void
ompl::control::PDST::getPlannerData(ompl::base::PlannerData &data) const
{
    ompl::base::Planner::getPlannerData(data);

    double propagationStepSize = siC_->getPropagationStepSize();

    std::vector<Motion *> motions;
    priorityQueue_.getContent(motions);

    // Add Goal vertex
    if (closestMotionToGoal_ != NULL)
    {
    data.addGoalVertex(closestMotionToGoal_->getState());
    }

    for (std::vector<Motion *>::iterator it = motions.begin(); it < motions.end(); ++it)
    {
    Motion *child = *it;
    Motion *parent = child->getParent();

    if (parent == NULL)
    {
        data.addStartVertex(ompl::base::PlannerDataVertex(child->getState()));
    }
    else if (data.hasControls())
    {
        data.addEdge(
        ompl::base::PlannerDataVertex(parent->getState()),
        ompl::base::PlannerDataVertex(child->getState()),
        ompl::control::PlannerDataEdgeControl(
            child->getControl(),
            child->getControlDuration() * propagationStepSize));
    }
    else
    {
        data.addEdge(ompl::base::PlannerDataVertex(parent->getState()),
             ompl::base::PlannerDataVertex(child->getState()));
    }
    }
}


// TODO: Ask Mark if the fact that operator != is not virtual in PlannerDataVertex and PlannerDataEdge is a problem
// TODO: Ask Mark if I should be concerned about the efficiency of getPlannerData - It looks like getPlannerData uses
// some sort of hash table structure to ensure that duplicate states are not added as vertices and duplicate edges are
// not addes as well. This seems like it could make getPlannerData be extremely time consuming.
// TODO: Note in documentation that the projection evaluator needs to correctly return getDimension or need to fix planner code
// Maybe I can change the projection dimension to not be a planner field and just be returned by the projectionEvaluator_ or make it a local variable in the call to solve
