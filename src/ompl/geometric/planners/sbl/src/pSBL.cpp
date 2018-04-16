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

#include "ompl/geometric/planners/sbl/pSBL.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::geometric::pSBL::pSBL(const base::SpaceInformationPtr &si) : base::Planner(si, "pSBL"), samplerArray_(si)
{
    specs_.recognizedGoal = base::GOAL_STATE;
    specs_.multithreaded = true;
    setThreadCount(2);

    Planner::declareParam<double>("range", this, &pSBL::setRange, &pSBL::getRange, "0.:1.:10000.");
    Planner::declareParam<unsigned int>("thread_count", this, &pSBL::setThreadCount, &pSBL::getThreadCount, "1:64");
}

ompl::geometric::pSBL::~pSBL()
{
    freeMemory();
}

void ompl::geometric::pSBL::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);

    tStart_.grid.setDimension(projectionEvaluator_->getDimension());
    tGoal_.grid.setDimension(projectionEvaluator_->getDimension());
}

void ompl::geometric::pSBL::clear()
{
    Planner::clear();

    samplerArray_.clear();

    freeMemory();

    tStart_.grid.clear();
    tStart_.size = 0;
    tStart_.pdf.clear();

    tGoal_.grid.clear();
    tGoal_.size = 0;
    tGoal_.pdf.clear();

    removeList_.motions.clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
}

void ompl::geometric::pSBL::freeGridMotions(Grid<MotionInfo> &grid)
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

void ompl::geometric::pSBL::threadSolve(unsigned int tid, const base::PlannerTerminationCondition &ptc,
                                        SolutionInfo *sol)
{
    RNG rng;

    std::vector<Motion *> solution;
    base::State *xstate = si_->allocState();
    bool startTree = rng.uniformBool();

    while (!sol->found && ptc == false)
    {
        bool retry = true;
        while (retry && !sol->found && ptc == false)
        {
            removeList_.lock.lock();
            if (!removeList_.motions.empty())
            {
                if (loopLock_.try_lock())
                {
                    retry = false;
                    std::map<Motion *, bool> seen;
                    for (auto &motion : removeList_.motions)
                        if (seen.find(motion.motion) == seen.end())
                            removeMotion(*motion.tree, motion.motion, seen);
                    removeList_.motions.clear();
                    loopLock_.unlock();
                }
            }
            else
                retry = false;
            removeList_.lock.unlock();
        }

        if (sol->found || ptc)
            break;

        loopLockCounter_.lock();
        if (loopCounter_ == 0)
            loopLock_.lock();
        loopCounter_++;
        loopLockCounter_.unlock();

        TreeData &tree = startTree ? tStart_ : tGoal_;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        Motion *existing = selectMotion(rng, tree);
        if (!samplerArray_[tid]->sampleNear(xstate, existing->state, maxDistance_))
            continue;

        /* create a motion */
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, xstate);
        motion->parent = existing;
        motion->root = existing->root;

        existing->lock.lock();
        existing->children.push_back(motion);
        existing->lock.unlock();

        addMotion(tree, motion);

        if (checkSolution(rng, !startTree, tree, otherTree, motion, solution))
        {
            sol->lock.lock();
            if (!sol->found)
            {
                sol->found = true;
                auto path(std::make_shared<PathGeometric>(si_));
                for (auto &i : solution)
                    path->append(i->state);
                pdef_->addSolutionPath(path, false, 0.0, getName());
            }
            sol->lock.unlock();
        }

        loopLockCounter_.lock();
        loopCounter_--;
        if (loopCounter_ == 0)
            loopLock_.unlock();
        loopLockCounter_.unlock();
    }

    si_->freeState(xstate);
}

ompl::base::PlannerStatus ompl::geometric::pSBL::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    auto *goal = dynamic_cast<base::GoalState *>(pdef_->getGoal().get());

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

    if (tGoal_.size == 0)
    {
        if (si_->satisfiesBounds(goal->getState()) && si_->isValid(goal->getState()))
        {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, goal->getState());
            motion->valid = true;
            motion->root = motion->state;
            addMotion(tGoal_, motion);
        }
        else
            OMPL_ERROR("%s: Goal state is invalid!", getName().c_str());
    }

    if (tStart_.size == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    if (tGoal_.size == 0)
    {
        OMPL_ERROR("%s: Motion planning goal tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    samplerArray_.resize(threadCount_);

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_.size + tGoal_.size));

    SolutionInfo sol;
    sol.found = false;
    loopCounter_ = 0;

    std::vector<std::thread *> th(threadCount_);
    for (unsigned int i = 0; i < threadCount_; ++i)
        th[i] = new std::thread([this, i, &ptc, &sol] { threadSolve(i, ptc, &sol); });
    for (unsigned int i = 0; i < threadCount_; ++i)
    {
        th[i]->join();
        delete th[i];
    }

    OMPL_INFORM("%s: Created %u (%u start + %u goal) states in %u cells (%u start + %u goal)", getName().c_str(),
                tStart_.size + tGoal_.size, tStart_.size, tGoal_.size, tStart_.grid.size() + tGoal_.grid.size(),
                tStart_.grid.size(), tGoal_.grid.size());

    return sol.found ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

bool ompl::geometric::pSBL::checkSolution(RNG &rng, bool start, TreeData &tree, TreeData &otherTree, Motion *motion,
                                          std::vector<Motion *> &solution)
{
    Grid<MotionInfo>::Coord coord(projectionEvaluator_->getDimension());
    projectionEvaluator_->computeCoordinates(motion->state, coord);

    otherTree.lock.lock();
    Grid<MotionInfo>::Cell *cell = otherTree.grid.getCell(coord);

    if (cell && !cell->data.empty())
    {
        Motion *connectOther = cell->data[rng.uniformInt(0, cell->data.size() - 1)];
        otherTree.lock.unlock();

        if (pdef_->getGoal()->isStartGoalPairValid(start ? motion->root : connectOther->root,
                                                   start ? connectOther->root : motion->root))
        {
            auto *connect = new Motion(si_);

            si_->copyState(connect->state, connectOther->state);
            connect->parent = motion;
            connect->root = motion->root;

            motion->lock.lock();
            motion->children.push_back(connect);
            motion->lock.unlock();

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
    else
        otherTree.lock.unlock();

    return false;
}

bool ompl::geometric::pSBL::isPathValid(TreeData &tree, Motion *motion)
{
    std::vector<Motion *> mpath;

    /* construct the solution path */
    while (motion != nullptr)
    {
        mpath.push_back(motion);
        motion = motion->parent;
    }

    bool result = true;

    /* check the path */
    for (int i = mpath.size() - 1; result && i >= 0; --i)
    {
        mpath[i]->lock.lock();
        if (!mpath[i]->valid)
        {
            if (si_->checkMotion(mpath[i]->parent->state, mpath[i]->state))
                mpath[i]->valid = true;
            else
            {
                // remember we need to remove this motion
                PendingRemoveMotion prm;
                prm.tree = &tree;
                prm.motion = mpath[i];
                removeList_.lock.lock();
                removeList_.motions.push_back(prm);
                removeList_.lock.unlock();
                result = false;
            }
        }
        mpath[i]->lock.unlock();
    }

    return result;
}

ompl::geometric::pSBL::Motion *ompl::geometric::pSBL::selectMotion(RNG &rng, TreeData &tree)
{
    tree.lock.lock();
    GridCell *cell = tree.pdf.sample(rng.uniform01());
    Motion *result = cell && !cell->data.empty() ? cell->data[rng.uniformInt(0, cell->data.size() - 1)] : nullptr;
    tree.lock.unlock();
    return result;
}

void ompl::geometric::pSBL::removeMotion(TreeData &tree, Motion *motion, std::map<Motion *, bool> &seen)
{
    /* remove from grid */
    seen[motion] = true;

    Grid<MotionInfo>::Coord coord(projectionEvaluator_->getDimension());
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid<MotionInfo>::Cell *cell = tree.grid.getCell(coord);
    if (cell)
    {
        for (unsigned int i = 0; i < cell->data.size(); ++i)
            if (cell->data[i] == motion)
            {
                cell->data.erase(cell->data.begin() + i);
                tree.size--;
                break;
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
            if (motion->parent->children[i] == motion)
            {
                motion->parent->children.erase(motion->parent->children.begin() + i);
                break;
            }
    }

    /* remove children */
    for (auto &i : motion->children)
    {
        i->parent = nullptr;
        removeMotion(tree, i, seen);
    }

    if (motion->state)
        si_->freeState(motion->state);
    delete motion;
}

void ompl::geometric::pSBL::addMotion(TreeData &tree, Motion *motion)
{
    Grid<MotionInfo>::Coord coord(projectionEvaluator_->getDimension());
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    tree.lock.lock();
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
    tree.lock.unlock();
}

void ompl::geometric::pSBL::getPlannerData(base::PlannerData &data) const
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

void ompl::geometric::pSBL::setThreadCount(unsigned int nthreads)
{
    assert(nthreads > 0);
    threadCount_ = nthreads;
}
