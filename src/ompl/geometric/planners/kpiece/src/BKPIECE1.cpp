/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Rice University, Inc.
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

/* Author: Ioan Sucan */

#include "ompl/geometric/planners/kpiece/BKPIECE1.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <cassert>

void ompl::geometric::BKPIECE1::setup(void)
{
    Planner::setup();
    checkProjectionEvaluator(this, projectionEvaluator_);
    checkMotionLength(this, maxDistance_);

    if (minValidPathFraction_ < std::numeric_limits<double>::epsilon() || minValidPathFraction_ > 1.0)
        throw Exception("The minimum valid path fraction must be in the range (0,1]");
    if (selectBorderFraction_ < std::numeric_limits<double>::epsilon() || selectBorderFraction_ > 1.0)
        throw Exception("The fraction of time spent selecting border cells must be in the range (0,1]");

    tStart_.grid.setDimension(projectionEvaluator_->getDimension());
    tGoal_.grid.setDimension(projectionEvaluator_->getDimension());
}

bool ompl::geometric::BKPIECE1::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        msg_.error("Unknown type of goal (or goal undefined)");
        return false;
    }

    while (const base::State *st = pis_.nextStart())
    {
        Motion* motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = st;
        addMotion(tStart_, motion);
    }

    if (tStart_.size == 0)
    {
        msg_.error("Motion planning start tree could not be initialized!");
        return false;
    }

    if (!goal->canSample())
    {
        msg_.error("Insufficient states in sampleable goal region");
        return false;
    }

    if (!sampler_)
        sampler_ = si_->allocManifoldStateSampler();

    msg_.inform("Starting with %d states", (int)(tStart_.size + tGoal_.size));

    std::vector<Motion*> solution;
    base::State *xstate = si_->allocState();
    bool      startTree = true;
    Grid::Coord coordC;

    while (ptc() == false)
    {
        TreeData &tree      = startTree ? tStart_ : tGoal_;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;
        tree.iteration++;

        // if we have not sampled too many goals already
        if (tGoal_.size == 0 || pis_.getSampledGoalsCount() < tGoal_.size / 2)
        {
            const base::State *st = tGoal_.size == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
            {
                Motion* motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                addMotion(tGoal_, motion);
            }
            if (tGoal_.size == 0)
            {
                msg_.error("Unable to sample any valid states for goal tree");
                break;
            }
        }

        Grid::Cell *ecell = NULL;
        Motion* existing  = NULL;
        selectMotion(tree, existing, ecell);
        assert(existing);
        sampler_->sampleUniformNear(xstate, existing->state, maxDistance_);

        std::pair<base::State*, double> fail(xstate, 0.0);
        bool keep = si_->checkMotion(existing->state, xstate, fail);
        if (!keep && fail.second > minValidPathFraction_)
            keep = true;

        if (keep)
        {
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, xstate);
            motion->root = existing->root;
            motion->parent = existing;

            addMotion(tree, motion);

            projectionEvaluator_->computeCoordinates(xstate, coordC);
            Grid::Cell* cellC = otherTree.grid.getCell(coordC);

            if (cellC && !cellC->data->motions.empty())
            {
                Motion* connectOther = cellC->data->motions[rng_.uniformInt(0, cellC->data->motions.size() - 1)];

                if (goal->isStartGoalPairValid(startTree ? connectOther->root : motion->root, startTree ? motion->root : connectOther->root) &&
                    si_->checkMotion(motion->state, connectOther->state))
                {
                    /* extract the motions and put them in solution vector */

                    std::vector<Motion*> mpath1;
                    while (motion != NULL)
                    {
                        mpath1.push_back(motion);
                        motion = motion->parent;
                    }

                    std::vector<Motion*> mpath2;
                    while (connectOther != NULL)
                    {
                        mpath2.push_back(connectOther);
                        connectOther = connectOther->parent;
                    }

                    if (startTree)
                        mpath1.swap(mpath2);

                    std::vector<Motion*> solution;
                    for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
                        solution.push_back(mpath1[i]);
                    solution.insert(solution.end(), mpath2.begin(), mpath2.end());

                    PathGeometric *path = new PathGeometric(si_);
                    for (unsigned int i = 0 ; i < solution.size() ; ++i)
                        path->states.push_back(si_->cloneState(solution[i]->state));

                    goal->setDifference(0.0);
                    goal->setSolutionPath(base::PathPtr(path));
                    break;
                }
            }

            ecell->data->score *= goodScoreFactor_;
        }
        else
            ecell->data->score *= badScoreFactor_;
        tree.grid.update(ecell);
    }

    si_->freeState(xstate);

    msg_.inform("Created %u (%u start + %u goal) states in %u cells (%u start (%u on boundary) + %u goal (%u on boundary))",
                tStart_.size + tGoal_.size, tStart_.size, tGoal_.size,
                tStart_.grid.size() + tGoal_.grid.size(), tStart_.grid.size(), tStart_.grid.countExternal(),
                tGoal_.grid.size(), tGoal_.grid.countExternal());

    return goal->isAchieved();
}

void ompl::geometric::BKPIECE1::selectMotion(TreeData &tree, Motion* &smotion, Grid::Cell* &scell)
{
    scell = rng_.uniform01() < std::max(selectBorderFraction_, tree.grid.fracExternal()) ?
        tree.grid.topExternal() : tree.grid.topInternal();

    // We are running on finite precision, so our update scheme will end up
    // with 0 values for the score. This is where we fix the problem
    if (scell->data->score < std::numeric_limits<double>::epsilon())
    {
        std::vector<CellData*> content;
        content.reserve(tree.grid.size());
        tree.grid.getContent(content);
        for (std::vector<CellData*>::iterator it = content.begin() ; it != content.end() ; ++it)
            (*it)->score += 1.0 + log((double)((*it)->iteration));
        tree.grid.updateAll();
    }

    assert(scell && !scell->data->motions.empty());

    scell->data->selections++;
    smotion = scell->data->motions[rng_.halfNormalInt(0, scell->data->motions.size() - 1)];
}

void ompl::geometric::BKPIECE1::addMotion(TreeData &tree, Motion *motion)
{
    Grid::Coord coord;
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid::Cell *cell = tree.grid.getCell(coord);
    if (cell)
    {
        cell->data->motions.push_back(motion);
        cell->data->coverage += 1.0;
        tree.grid.update(cell);
    }
    else
    {
        cell = tree.grid.createCell(coord);
        cell->data = new CellData();
        cell->data->motions.push_back(motion);
        cell->data->coverage = 1.0;
        cell->data->iteration = tree.iteration;
        cell->data->selections = 1;
        cell->data->score = 1.0 + log((double)(tree.iteration));
        tree.grid.add(cell);
    }
    tree.size++;
}

void ompl::geometric::BKPIECE1::freeMemory(void)
{
    freeGridMotions(tStart_.grid);
    freeGridMotions(tGoal_.grid);
}

void ompl::geometric::BKPIECE1::freeGridMotions(Grid &grid)
{
    for (Grid::iterator it = grid.begin(); it != grid.end() ; ++it)
        freeCellData(it->second->data);
}

void ompl::geometric::BKPIECE1::freeCellData(CellData *cdata)
{
    for (unsigned int i = 0 ; i < cdata->motions.size() ; ++i)
        freeMotion(cdata->motions[i]);
    delete cdata;
}

void ompl::geometric::BKPIECE1::freeMotion(Motion *motion)
{
    if (motion->state)
        si_->freeState(motion->state);
    delete motion;
}

void ompl::geometric::BKPIECE1::clear(void)
{
    Planner::clear();

    sampler_.reset();

    freeMemory();

    tStart_.grid.clear();
    tStart_.size = 0;
    tStart_.iteration = 1;

    tGoal_.grid.clear();
    tGoal_.size = 0;
    tGoal_.iteration = 1;
}

void ompl::geometric::BKPIECE1::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<CellData*> cdata;
    tStart_.grid.getContent(cdata);
    for (unsigned int i = 0 ; i < cdata.size() ; ++i)
        for (unsigned int j = 0 ; j < cdata[i]->motions.size() ; ++j)
        {
            data.recordEdge(cdata[i]->motions[j]->parent ? cdata[i]->motions[j]->parent->state : NULL, cdata[i]->motions[j]->state);
            data.tagState(cdata[i]->motions[j]->state, 1);
        }


    cdata.clear();
    tGoal_.grid.getContent(cdata);
    for (unsigned int i = 0 ; i < cdata.size() ; ++i)
        for (unsigned int j = 0 ; j < cdata[i]->motions.size() ; ++j)
        {
            data.recordEdge(cdata[i]->motions[j]->parent ? cdata[i]->motions[j]->parent->state : NULL, cdata[i]->motions[j]->state);
            data.tagState(cdata[i]->motions[j]->state, 2);
        }
}
