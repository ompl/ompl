/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
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

#include "ompl/control/planners/kpiece/KPIECE1.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/Exception.h"
#include <limits>
#include <cassert>

ompl::control::KPIECE1::KPIECE1(const SpaceInformationPtr &si) : base::Planner(si, "KPIECE1")
{
    specs_.approximateSolutions = true;

    siC_ = si.get();
    tree_.grid.onCellUpdate(computeImportance, nullptr);

    Planner::declareParam<double>("goal_bias", this, &KPIECE1::setGoalBias, &KPIECE1::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("border_fraction", this, &KPIECE1::setBorderFraction, &KPIECE1::getBorderFraction,
                                  "0.:0.05:1.");
    Planner::declareParam<unsigned int>("max_close_samples", this, &KPIECE1::setMaxCloseSamplesCount,
                                        &KPIECE1::getMaxCloseSamplesCount);
    Planner::declareParam<double>("bad_score_factor", this, &KPIECE1::setBadCellScoreFactor,
                                  &KPIECE1::getBadCellScoreFactor);
    Planner::declareParam<double>("good_score_factor", this, &KPIECE1::setGoodCellScoreFactor,
                                  &KPIECE1::getGoodCellScoreFactor);
}

ompl::control::KPIECE1::~KPIECE1()
{
    freeMemory();
}

void ompl::control::KPIECE1::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);

    if (badScoreFactor_ < std::numeric_limits<double>::epsilon() || badScoreFactor_ > 1.0)
        throw Exception("Bad cell score factor must be in the range (0,1]");
    if (goodScoreFactor_ < std::numeric_limits<double>::epsilon() || goodScoreFactor_ > 1.0)
        throw Exception("Good cell score factor must be in the range (0,1]");
    if (selectBorderFraction_ < std::numeric_limits<double>::epsilon() || selectBorderFraction_ > 1.0)
        throw Exception("The fraction of time spent selecting border cells must be in the range (0,1]");

    tree_.grid.setDimension(projectionEvaluator_->getDimension());
}

void ompl::control::KPIECE1::clear()
{
    Planner::clear();
    controlSampler_.reset();
    freeMemory();
    tree_.grid.clear();
    tree_.size = 0;
    tree_.iteration = 1;
    lastGoalMotion_ = nullptr;
}

void ompl::control::KPIECE1::freeMemory()
{
    freeGridMotions(tree_.grid);
}

void ompl::control::KPIECE1::freeGridMotions(Grid &grid)
{
    for (const auto &it : grid)
        freeCellData(it.second->data);
}

void ompl::control::KPIECE1::freeCellData(CellData *cdata)
{
    for (auto &motion : cdata->motions)
        freeMotion(motion);
    delete cdata;
}

void ompl::control::KPIECE1::freeMotion(Motion *motion)
{
    if (motion->state)
        si_->freeState(motion->state);
    if (motion->control)
        siC_->freeControl(motion->control);
    delete motion;
}

bool ompl::control::KPIECE1::CloseSamples::consider(Grid::Cell *cell, Motion *motion, double distance)
{
    if (samples.empty())
    {
        CloseSample cs(cell, motion, distance);
        samples.insert(cs);
        return true;
    }
    // if the sample we're considering is closer to the goal than the worst sample in the
    // set of close samples, we include it
    if (samples.rbegin()->distance > distance)
    {
        // if the inclusion would go above the maximum allowed size,
        // remove the last element
        if (samples.size() >= maxSize)
            samples.erase(--samples.end());
        CloseSample cs(cell, motion, distance);
        samples.insert(cs);
        return true;
    }

    return false;
}

/// @cond IGNORE
// this is the factor by which distances are inflated when considered for addition to closest samples
static const double CLOSE_MOTION_DISTANCE_INFLATION_FACTOR = 1.1;
/// @endcond

bool ompl::control::KPIECE1::CloseSamples::selectMotion(Motion *&smotion, Grid::Cell *&scell)
{
    if (samples.size() > 0)
    {
        scell = samples.begin()->cell;
        smotion = samples.begin()->motion;
        // average the highest & lowest distances and multiply by CLOSE_MOTION_DISTANCE_INFLATION_FACTOR
        // (make the distance appear artificially longer)
        double d =
            (samples.begin()->distance + samples.rbegin()->distance) * (CLOSE_MOTION_DISTANCE_INFLATION_FACTOR / 2.0);
        samples.erase(samples.begin());
        consider(scell, smotion, d);
        return true;
    }
    return false;
}

unsigned int ompl::control::KPIECE1::findNextMotion(const std::vector<Grid::Coord> &coords, unsigned int index,
                                                    unsigned int count)
{
    for (unsigned int i = index + 1; i < count; ++i)
        if (coords[i] != coords[index])
            return i - 1;

    return count - 1;
}

ompl::base::PlannerStatus ompl::control::KPIECE1::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        addMotion(motion, 1.0);
    }

    if (tree_.grid.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!controlSampler_)
        controlSampler_ = siC_->allocControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), tree_.size);

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    Control *rctrl = siC_->allocControl();

    std::vector<base::State *> states(siC_->getMaxControlDuration() + 1);
    std::vector<Grid::Coord> coords(states.size(), Grid::Coord(projectionEvaluator_->getDimension()));
    std::vector<Grid::Cell *> cells(coords.size());

    for (auto &state : states)
        state = si_->allocState();

    // samples that were found to be the best, so far
    CloseSamples closeSamples(nCloseSamples_);

    while (ptc == false)
    {
        tree_.iteration++;

        /* Decide on a state to expand from */
        Motion *existing = nullptr;
        Grid::Cell *ecell = nullptr;

        if (closeSamples.canSample() && rng_.uniform01() < goalBias_)
        {
            if (!closeSamples.selectMotion(existing, ecell))
                selectMotion(existing, ecell);
        }
        else
            selectMotion(existing, ecell);
        assert(existing);

        /* sample a random control */
        controlSampler_->sampleNext(rctrl, existing->control, existing->state);

        /* propagate */
        unsigned int cd =
            controlSampler_->sampleStepCount(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
        cd = siC_->propagateWhileValid(existing->state, rctrl, cd, states, false);

        /* if we have enough steps */
        if (cd >= siC_->getMinControlDuration())
        {
            std::size_t avgCov_two_thirds = (2 * tree_.size) / (3 * tree_.grid.size());
            bool interestingMotion = false;

            // split the motion into smaller ones, so we do not cross cell boundaries
            for (unsigned int i = 0; i < cd; ++i)
            {
                projectionEvaluator_->computeCoordinates(states[i], coords[i]);
                cells[i] = tree_.grid.getCell(coords[i]);
                if (!cells[i])
                    interestingMotion = true;
                else
                {
                    if (!interestingMotion && cells[i]->data->motions.size() <= avgCov_two_thirds)
                        interestingMotion = true;
                }
            }

            if (interestingMotion || rng_.uniform01() < 0.05)
            {
                unsigned int index = 0;
                while (index < cd)
                {
                    unsigned int nextIndex = findNextMotion(coords, index, cd);
                    auto *motion = new Motion(siC_);
                    si_->copyState(motion->state, states[nextIndex]);
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = nextIndex - index + 1;
                    motion->parent = existing;

                    double dist = 0.0;
                    bool solv = goal->isSatisfied(motion->state, &dist);
                    Grid::Cell *toCell = addMotion(motion, dist);

                    if (solv)
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }

                    closeSamples.consider(toCell, motion, dist);

                    // new parent will be the newly created motion
                    existing = motion;
                    index = nextIndex + 1;
                }

                if (solution)
                    break;
            }

            // update cell score
            ecell->data->score *= goodScoreFactor_;
        }
        else
            ecell->data->score *= badScoreFactor_;

        tree_.grid.update(ecell);
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
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);

        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    siC_->freeControl(rctrl);
    for (auto &state : states)
        si_->freeState(state);

    OMPL_INFORM("%s: Created %u states in %u cells (%u internal + %u external)", getName().c_str(), tree_.size,
                tree_.grid.size(), tree_.grid.countInternal(), tree_.grid.countExternal());

    return {solved, approximate};
}

bool ompl::control::KPIECE1::selectMotion(Motion *&smotion, Grid::Cell *&scell)
{
    scell = rng_.uniform01() < std::max(selectBorderFraction_, tree_.grid.fracExternal()) ? tree_.grid.topExternal() :
                                                                                            tree_.grid.topInternal();

    // We are running on finite precision, so our update scheme will end up
    // with 0 values for the score. This is where we fix the problem
    if (scell->data->score < std::numeric_limits<double>::epsilon())
    {
        OMPL_DEBUG("%s: Numerical precision limit reached. Resetting costs.", getName().c_str());
        std::vector<CellData *> content;
        content.reserve(tree_.grid.size());
        tree_.grid.getContent(content);
        for (auto &it : content)
            it->score += 1.0 + log((double)(it->iteration));
        tree_.grid.updateAll();
    }

    if (scell && !scell->data->motions.empty())
    {
        scell->data->selections++;
        smotion = scell->data->motions[rng_.halfNormalInt(0, scell->data->motions.size() - 1)];
        return true;
    }
    else
        return false;
}

/// @cond IGNORE
// this is the offset added to estimated distances to the goal, so we avoid division by 0
static const double DISTANCE_TO_GOAL_OFFSET = 1e-3;
/// @endcond

ompl::control::KPIECE1::Grid::Cell *ompl::control::KPIECE1::addMotion(Motion *motion, double dist)
{
    Grid::Coord coord(projectionEvaluator_->getDimension());
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid::Cell *cell = tree_.grid.getCell(coord);
    if (cell)
    {
        cell->data->motions.push_back(motion);
        cell->data->coverage += motion->steps;
        tree_.grid.update(cell);
    }
    else
    {
        cell = tree_.grid.createCell(coord);
        cell->data = new CellData();
        cell->data->motions.push_back(motion);
        cell->data->coverage = motion->steps;
        cell->data->iteration = tree_.iteration;
        cell->data->selections = 1;
        cell->data->score = (1.0 + log((double)(tree_.iteration))) / (DISTANCE_TO_GOAL_OFFSET + dist);
        tree_.grid.add(cell);
    }
    tree_.size++;
    return cell;
}

void ompl::control::KPIECE1::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    Grid::CellArray cells;
    tree_.grid.getCells(cells);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &cell : cells)
    {
        for (const auto &m : cell->data->motions)
        {
            if (m->parent)
            {
                if (data.hasControls())
                    data.addEdge(base::PlannerDataVertex(m->parent->state),
                                 base::PlannerDataVertex(m->state, cell->border ? 2 : 1),
                                 control::PlannerDataEdgeControl(m->control, m->steps * delta));
                else
                    data.addEdge(base::PlannerDataVertex(m->parent->state),
                                 base::PlannerDataVertex(m->state, cell->border ? 2 : 1));
            }
            else
                data.addStartVertex(base::PlannerDataVertex(m->state, cell->border ? 2 : 1));

            // A state created as a parent first may have an improper tag variable
            data.tagState(m->state, cell->border ? 2 : 1);
        }
    }
}
