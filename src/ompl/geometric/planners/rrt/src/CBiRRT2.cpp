/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Ryan Luna */

#include "ompl/geometric/planners/rrt/CBiRRT2.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::geometric::CBiRRT2::CBiRRT2(const base::SpaceInformationPtr &si) : base::Planner(si, "CBiRRT2")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    maxDistance_ = 0.0;
    connectionPoint_ = std::make_pair<base::State*, base::State*>(NULL, NULL);

    const base::ConstrainedSpaceInformationPtr& csi = boost::dynamic_pointer_cast<base::ConstrainedSpaceInformation>(si);
    if (!csi)
        OMPL_ERROR("%s: Failed to cast SpaceInformation to ConstrainedSpaceInformation", getName().c_str());
    else
        ci_ = csi->getConstraintInformation();

    Planner::declareParam<double>("range", this, &CBiRRT2::setRange, &CBiRRT2::getRange, "0.:1.:10000.");
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::geometric::CBiRRT2::~CBiRRT2(void)
{
    freeMemory();
}

void ompl::geometric::CBiRRT2::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State*, base::State*>(NULL, NULL);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

void ompl::geometric::CBiRRT2::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    tStart_->setDistanceFunction(boost::bind(&CBiRRT2::distanceFunction, this, _1, _2));
    tGoal_->setDistanceFunction(boost::bind(&CBiRRT2::distanceFunction, this, _1, _2));
}

void ompl::geometric::CBiRRT2::freeMemory(void)
{
    std::vector<Motion*> motions;
    if (tStart_)
    {
        tStart_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

// Grow the given tree from nmotion toward rmotion
ompl::geometric::CBiRRT2::GrowState ompl::geometric::CBiRRT2::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                       const Motion *rmotion, const Motion *nmotion)
{
    base::State *dstate = rmotion->state;

    // assume we can reach rmotion from nmotion
    bool reach = true;

    // If rmotion is further than maxDistance_ away, interpolate toward rmotion
    // until maxDistance_ is reached.
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);

        // Project new state onto constraint manifold and check its validity
        if (!ci_->project(tgi.xstate) || !si_->isValid(tgi.xstate))
            return TRAPPED;

        dstate = tgi.xstate;
        reach = false; // rmotion is too far away from nmotion
    }

    std::vector<base::State*> extension;

    ///////////////////////////////////////
    // A somewhat major assumption here:
    // Checking for the extension in the goal tree *should* be performed backward, from the
    // sampled state toward nmotion, since this is the direction of motion in the
    // solution path.  If the extension fails backward, however, then we have not actually
    // connected to the goal tree.  To avoid throwing away lots of work, it is assumed that
    // the motions (distances, costs, etc.) are symmetric.
    ///////////////////////////////////////
    reach &= constrainedExtend(nmotion->state, dstate, extension);

    if (extension.size() > 0)
    {
        const Motion* parent = nmotion;
        // Create a new motion for the sequence of states in the extension
        for(size_t i = 0; i < extension.size(); ++i)
        {
            Motion *motion = new Motion(extension[i]);
            motion->parent = parent;
            motion->root = parent->root;
            tree->add(motion);

            tgi.xmotion = motion;
            parent = motion;
        }

        return reach ? REACHED : ADVANCED;
    }

    return TRAPPED;
}

bool ompl::geometric::CBiRRT2::constrainedExtend(const base::State* a, const base::State* b,
                                                 std::vector<base::State*>& result) const
{
    // A semi-faithful implementation from the paper
    // Instead of using vector operations, this implementation
    // will accomplish a similar extension using interpolation in
    // the state space

    // Assuming a and b are both valid and on constraint manifold
    base::StateSpacePtr ss = si_->getStateSpace();
    const base::State* previous = a;

    // number of discrete steps between a and b in the state space
    int n = ss->validSegmentCount(a, b);

    if (n == 0) // don't divide by zero
        return true;

    double dist = ss->distance(a, b);
    double delta = dist / n; // This is the step size that we will take during extension

    while (true)
    {
        // The distance to travel is less than our step size.  Just declare victory
        if (dist < (delta + std::numeric_limits<double>::epsilon()))
        {
            result.push_back(si_->cloneState(b));
            return true;
        }

        // Compute the parametrization for interpolation
        double t = delta / dist;

        base::State* scratchState = ss->allocState();
        ss->interpolate(previous, b, t, scratchState);

        // Project new state onto constraint manifold.  Make sure the new state is valid
        // and that it has not deviated too far from where we started
        if (!ci_->project(scratchState) ||
            !si_->isValid(scratchState) ||
            ss->distance(previous, scratchState) > 2.0*delta)
        {
            ss->freeState(scratchState);
            break;
        }

        // Check for divergence.  Divergence is declared if we are no closer to b
        // than before projection
        double newDist = ss->distance(scratchState, b);
        if (newDist >= dist)
        {
            // Since we already collision checked this state, we might as well keep it
            result.push_back(scratchState);
            break;
        }
        dist = newDist;

        // No divergence; getting closer.  Store the new state
        result.push_back(scratchState);
        previous = scratchState;
    }
    return false;
}

ompl::base::PlannerStatus ompl::geometric::CBiRRT2::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    if (!ci_)
    {
        OMPL_ERROR("%s: ConstraintInformation is invalid!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Get a start state and root the start tree with it
    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting with %d states", getName().c_str(), (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion   *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool solved         = false;

    TreeData tree = tStart_;
    TreeData otherTree = tGoal_;
    tgi.start = true;  // begin planning with the start tree

    while (ptc == false)
    {
        // Make sure that the goal tree has one state in it
        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
            {
                if (ci_->isSatisfied(st))
                {
                    Motion* motion = new Motion(si_);
                    si_->copyState(motion->state, st);
                    motion->root = motion->state;
                    tGoal_->add(motion);
                }
                else
                    OMPL_WARN("%s: Sampled a goal state that does NOT satisfy path constraints!", getName().c_str());
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        // Sample a valid, constrained state
        if (!ci_->sample(rstate) || !si_->isValid(rstate))
            continue;

        // Try to connect the state we sampled to tree
        // If we did not add any state to the tree, try again
        if (growTree(tree, tgi, rmotion, tree->nearest(rmotion)) == TRAPPED)
            continue;

        // Remember the state added to the tree
        const Motion* startMotion = tgi.xmotion;
        Motion *addedMotion = tgi.xmotion;

        // Now growing the other tree
        tgi.start = !tgi.start;

        // Grow the other tree toward the state we just added in the initial tree
        GrowState growOther;
        do
        {
            growOther = growTree(otherTree, tgi, startMotion, otherTree->nearest(tgi.xmotion));
        } while (growOther == ADVANCED && !ptc);

        // Update the distance between trees
        const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
        if (newDist < distanceBetweenTrees_)
        {
            distanceBetweenTrees_ = newDist;
            OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
        }

        // Did not connect to initial tree
        if (growOther == TRAPPED || ptc)
        {
            // Swap the trees for the next iteration
            std::swap(tree, otherTree);
            continue;
        }

        // If we are here, we connected the trees!  If the trees are connected
        // in a valid way (start and goal pair is valid), we are done
        // First, make sure the motion named startMotion is in the start tree,
        // else swap with goalMotion
        const Motion* goalMotion = tgi.xmotion;

        if (tgi.start) // at this point, this means start was attempted after goal tree this iteration
            std::swap(startMotion, goalMotion);

        if (goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
        {
            // There is now a duplicate state in the start and goal trees
            // (the last state that was added).  startMotion and goalMotion
            // should be the same state.  To avoid the duplicate state, we
            // start the first part of the path from one of the parents, since
            // one of them MUST exist
            if (startMotion->parent)
                startMotion = startMotion->parent;
            else
                goalMotion = goalMotion->parent;

            connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

            // Constructing the solution path
            // Get the portion of the path in the start tree
            const Motion *solution = startMotion;
            std::vector<const Motion*> mpath1;
            while (solution != NULL)
            {
                mpath1.push_back(solution);
                solution = solution->parent;
            }

            // Get the portion of the path in the goal tree
            solution = goalMotion;
            std::vector<const Motion*> mpath2;
            while (solution != NULL)
            {
                mpath2.push_back(solution);
                solution = solution->parent;
            }

            // Concatenating the start and goal paths
            PathGeometric *path = new PathGeometric(si_);
            path->getStates().reserve(mpath1.size() + mpath2.size());
            for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
                path->append(mpath1[i]->state);
            for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
                path->append(mpath2[i]->state);

            shortcutPath(path, ptc);
            pdef_->addSolutionPath(base::PathPtr(path), false, 0.0);
            solved = true;
            break;
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

bool ompl::geometric::CBiRRT2::shortcutPath(PathGeometric *path, const base::PlannerTerminationCondition &ptc)
{
    std::vector<base::State*>& states = path->getStates();

    unsigned int count = 0;
    unsigned int maxCount = 10;

    while(!ptc && count < maxCount && states.size() > 2)
    {
        int i = rng_.uniformInt(0, states.size()-2);
        int j;
        do
        {
            j = rng_.uniformInt(0, states.size()-1);
        } while (abs(i-j) < 2); // make sure the difference between i and j is at least two

        if (i > j)
            std::swap(i, j);

        base::State* a = states[i];
        base::State* b = states[j];
        std::vector<base::State*> shortcut;

        bool foundShortcut = false;
        if (constrainedExtend(a, b, shortcut) && shortcut.size() > 1)
        {
            // see if shortcut is actually shorter
            double shortcutDist = 0.0;
            for(size_t k = 1; k < shortcut.size(); ++k)
                shortcutDist += si_->distance(shortcut[k-1], shortcut[k]);

            double pathDist = 0.0;
            for(int k = i+1; k < j; ++k)
                pathDist += si_->distance(states[k-1], states[k]);

            if (shortcutDist < pathDist)
            {
                // Delete states between [i+1, j]
                for(int k = i+1; k < j+1; ++k)
                    si_->freeState(states[k]);
                states.erase(states.begin() + i+1, states.begin() + j+1);

                // Inserting new shortcut
                states.insert(states.begin() + i+1, shortcut.begin(), shortcut.end());
                foundShortcut = true;
            }
        }

        if (!foundShortcut)
        {
            for(size_t k = 0; k < shortcut.size(); ++k)
                si_->freeState(shortcut[k]);
            count++;
        }
        else
            count = 0;
    }

    return true;
}

void ompl::geometric::CBiRRT2::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (tStart_)
        tStart_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state, 1),
                         base::PlannerDataVertex(motions[i]->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addGoalVertex(base::PlannerDataVertex(motions[i]->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motions[i]->state, 2),
                         base::PlannerDataVertex(motions[i]->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = boost::lexical_cast<std::string>(distanceBetweenTrees_);
}
