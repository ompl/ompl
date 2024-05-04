/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include "ompl/geometric/planners/sbl/SBL.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::geometric::SBL::SBL(const base::SpaceInformationPtr &si) : base::Planner(si, "SBL")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;

    Planner::declareParam<double>("range", this, &SBL::setRange, &SBL::getRange, "0.:1.:10000.");
}

ompl::geometric::SBL::~SBL()
{
    freeMemory();
}

void ompl::geometric::SBL::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);

    tStart_.grid.setDimension(projectionEvaluator_->getDimension());
    tGoal_.grid.setDimension(projectionEvaluator_->getDimension());
}

void ompl::geometric::SBL::freeGridMotions(Grid<MotionInfo> &grid)
{
    for (const auto &it : grid)
    {
        for (unsigned int i = 0; i < it.second->data.size(); ++i)
        {
            if (it.second->data[i]->state)
                si_->freeState(it.second->data[i]->state);
            delete it.second->data[i];
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::SBL::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->valid = true;
        motion->root = motion->state;
        addMotion(tStart_, motion);
    }

    if (tStart_.size == 0)
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
        sampler_ = si_->allocValidStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_.size + tGoal_.size));

    std::vector<Motion *> solution;
    base::State *xstate = si_->allocState();

    bool startTree = true;
    bool solved = false;

    while (ptc == false)
    {
        TreeData &tree = startTree ? tStart_ : tGoal_;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        // if we have not sampled too many goals already
        if (tGoal_.size == 0 || pis_.getSampledGoalsCount() < tGoal_.size / 2)
        {
            const base::State *st = tGoal_.size == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                motion->valid = true;
                addMotion(tGoal_, motion);
            }
            if (tGoal_.size == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        Motion *existing = selectMotion(tree);
        assert(existing);
        if (!sampler_->sampleNear(xstate, existing->state, maxDistance_))
            continue;

        /* create a motion */
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, xstate);
        motion->parent = existing;
        motion->root = existing->root;
        existing->children.push_back(motion);

        addMotion(tree, motion);

        if (checkSolution(!startTree, tree, otherTree, motion, solution))
        {
            auto path(std::make_shared<PathGeometric>(si_));
            for (auto &i : solution)
                path->append(i->state);

            pdef_->addSolutionPath(path, false, 0.0, getName());
            solved = true;
            break;
        }
    }

    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u (%u start + %u goal) states in %u cells (%u start + %u goal)", getName().c_str(),
                tStart_.size + tGoal_.size, tStart_.size, tGoal_.size, tStart_.grid.size() + tGoal_.grid.size(),
                tStart_.grid.size(), tGoal_.grid.size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

bool ompl::geometric::SBL::checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion *motion,
                                         std::vector<Motion *> &solution)
{
    Grid<MotionInfo>::Coord coord(projectionEvaluator_->getDimension());
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid<MotionInfo>::Cell *cell = otherTree.grid.getCell(coord);

    if (cell && !cell->data.empty())
    {
        Motion *connectOther = cell->data[rng_.uniformInt(0, cell->data.size() - 1)];

        if (pdef_->getGoal()->isStartGoalPairValid(start ? motion->root : connectOther->root,
                                                   start ? connectOther->root : motion->root))
        {
            auto *connect = new Motion(si_);

            si_->copyState(connect->state, connectOther->state);
            connect->parent = motion;
            connect->root = motion->root;
            motion->children.push_back(connect);
            addMotion(tree, connect);

            if (isPathValid(tree, connect) && isPathValid(otherTree, connectOther))
            {
                if (start)
                    connectionPoint_ = std::make_pair(motion->state, connectOther->state);
                else
                    connectionPoint_ = std::make_pair(connectOther->state, motion->state);

                /* extract the motions and put them in solution vector */

                std::vector<Motion *> mpath1;
                while (motion != nullptr)
                {
                    mpath1.push_back(motion);
                    motion = motion->parent;
                }

                std::vector<Motion *> mpath2;
                while (connectOther != nullptr)
                {
                    mpath2.push_back(connectOther);
                    connectOther = connectOther->parent;
                }

                if (!start)
                    mpath1.swap(mpath2);

                for (int i = mpath1.size() - 1; i >= 0; --i)
                    solution.push_back(mpath1[i]);
                solution.insert(solution.end(), mpath2.begin(), mpath2.end());

                return true;
            }
        }
    }
    return false;
}

bool ompl::geometric::SBL::isPathValid(TreeData &tree, Motion *motion)
{
    std::vector<Motion *> mpath;

    /* construct the solution path */
    while (motion != nullptr)
    {
        mpath.push_back(motion);
        motion = motion->parent;
    }

    /* check the path */
    for (int i = mpath.size() - 1; i >= 0; --i)
        if (!mpath[i]->valid)
        {
            if (si_->checkMotion(mpath[i]->parent->state, mpath[i]->state))
                mpath[i]->valid = true;
            else
            {
                removeMotion(tree, mpath[i]);
                return false;
            }
        }
    return true;
}

ompl::geometric::SBL::Motion *ompl::geometric::SBL::selectMotion(TreeData &tree)
{
    GridCell *cell = tree.pdf.sample(rng_.uniform01());
    return cell && !cell->data.empty() ? cell->data[rng_.uniformInt(0, cell->data.size() - 1)] : nullptr;
}

void ompl::geometric::SBL::removeMotion(TreeData &tree, Motion *motion)
{
    /* remove from grid */

    Grid<MotionInfo>::Coord coord(projectionEvaluator_->getDimension());
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid<MotionInfo>::Cell *cell = tree.grid.getCell(coord);
    if (cell)
    {
        for (unsigned int i = 0; i < cell->data.size(); ++i)
        {
            if (cell->data[i] == motion)
            {
                cell->data.erase(cell->data.begin() + i);
                tree.size--;
                break;
            }
        }
        if (cell->data.empty())
        {
            tree.pdf.remove(cell->data.elem_);
            tree.grid.remove(cell);
            tree.grid.destroyCell(cell);
        }
        else
        {
            tree.pdf.update(cell->data.elem_, 1.0 / cell->data.size());
        }
    }

    /* remove self from parent list */

    if (motion->parent)
    {
        for (unsigned int i = 0; i < motion->parent->children.size(); ++i)
        {
            if (motion->parent->children[i] == motion)
            {
                motion->parent->children.erase(motion->parent->children.begin() + i);
                break;
            }
        }
    }

    /* remove children */
    for (auto &i : motion->children)
    {
        i->parent = nullptr;
        removeMotion(tree, i);
    }

    if (motion->state)
        si_->freeState(motion->state);
    delete motion;
}

void ompl::geometric::SBL::addMotion(TreeData &tree, Motion *motion)
{
    Grid<MotionInfo>::Coord coord(projectionEvaluator_->getDimension());
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid<MotionInfo>::Cell *cell = tree.grid.getCell(coord);
    if (cell)
    {
        cell->data.push_back(motion);
        tree.pdf.update(cell->data.elem_, 1.0 / cell->data.size());
    }
    else
    {
        cell = tree.grid.createCell(coord);
        cell->data.push_back(motion);
        tree.grid.add(cell);
        cell->data.elem_ = tree.pdf.add(cell, 1.0);
    }
    tree.size++;
}

void ompl::geometric::SBL::clear()
{
    Planner::clear();

    sampler_.reset();

    freeMemory();

    tStart_.grid.clear();
    tStart_.size = 0;
    tStart_.pdf.clear();

    tGoal_.grid.clear();
    tGoal_.size = 0;
    tGoal_.pdf.clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
}

void ompl::geometric::SBL::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<MotionInfo> motionInfo;
    tStart_.grid.getContent(motionInfo);

    for (auto &m : motionInfo)
        for (auto &motion : m.motions_)
            if (motion->parent == nullptr)
                data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
            else
                data.addEdge(base::PlannerDataVertex(motion->parent->state, 1),
                             base::PlannerDataVertex(motion->state, 1));

    motionInfo.clear();
    tGoal_.grid.getContent(motionInfo);
    for (auto &m : motionInfo)
        for (auto &motion : m.motions_)
            if (motion->parent == nullptr)
                data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
            else
                // The edges in the goal tree are reversed so that they are in the same direction as start tree
                data.addEdge(base::PlannerDataVertex(motion->state, 2),
                             base::PlannerDataVertex(motion->parent->state, 2));

    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}
