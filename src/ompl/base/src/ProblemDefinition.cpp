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

#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/PathControl.h"
#include "ompl/tools/config/MagicConstants.h"
#include <sstream>
#include <algorithm>

#include <boost/thread/mutex.hpp>

/// @cond IGNORE
namespace ompl
{
    namespace base
    {

        class ProblemDefinition::PlannerSolutionSet
        {
        public:

            PlannerSolutionSet()
            {
            }

            void add(const PlannerSolution &s)
            {
                boost::mutex::scoped_lock slock(lock_);
                int index = solutions_.size();
                solutions_.push_back(s);
                solutions_.back().index_ = index;
                std::sort(solutions_.begin(), solutions_.end());
            }

            void clear()
            {
                boost::mutex::scoped_lock slock(lock_);
                solutions_.clear();
            }

            std::vector<PlannerSolution> getSolutions()
            {
                boost::mutex::scoped_lock slock(lock_);
                std::vector<PlannerSolution> copy = solutions_;
                return copy;
            }

            bool isApproximate()
            {
                boost::mutex::scoped_lock slock(lock_);
                bool result = false;
                if (!solutions_.empty())
                    result = solutions_[0].approximate_;
                return result;
            }

            bool isOptimized()
            {
                boost::mutex::scoped_lock slock(lock_);
                bool result = false;
                if (!solutions_.empty())
                    result = solutions_[0].optimized_;
                return result;
            }

            double getDifference()
            {
                boost::mutex::scoped_lock slock(lock_);
                double diff = -1.0;
                if (!solutions_.empty())
                    diff = solutions_[0].difference_;
                return diff;
            }

            PathPtr getTopSolution()
            {
                boost::mutex::scoped_lock slock(lock_);
                PathPtr copy;
                if (!solutions_.empty())
                    copy = solutions_[0].path_;
                return copy;
            }

            bool getTopSolution(PlannerSolution& solution)
            {
                boost::mutex::scoped_lock slock(lock_);

                if (!solutions_.empty())
                {
                    solution = solutions_[0];
                    return true;
                }
                else
                {
                    return false;
                }
            }

            std::size_t getSolutionCount()
            {
                boost::mutex::scoped_lock slock(lock_);
                std::size_t result = solutions_.size();
                return result;
            }

        private:

            std::vector<PlannerSolution> solutions_;
            boost::mutex                 lock_;
        };
    }
}
/// @endcond

ompl::base::ProblemDefinition::ProblemDefinition(const SpaceInformationPtr &si) : si_(si), solutions_(new PlannerSolutionSet())
{
}

void ompl::base::ProblemDefinition::setStartAndGoalStates(const State *start, const State *goal, const double threshold)
{
    clearStartStates();
    addStartState(start);
    setGoalState(goal, threshold);
}

void ompl::base::ProblemDefinition::setGoalState(const State *goal, const double threshold)
{
    clearGoal();
    GoalState *gs = new GoalState(si_);
    gs->setState(goal);
    gs->setThreshold(threshold);
    setGoal(GoalPtr(gs));
}

bool ompl::base::ProblemDefinition::hasStartState(const State *state, unsigned int *startIndex)
{
    for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
        if (si_->equalStates(state, startStates_[i]))
        {
            if (startIndex)
                *startIndex = i;
            return true;
        }
    return false;
}

bool ompl::base::ProblemDefinition::fixInvalidInputState(State *state, double dist, bool start, unsigned int attempts)
{
    bool result = false;

    bool b = si_->satisfiesBounds(state);
    bool v = false;
    if (b)
    {
        v = si_->isValid(state);
        if (!v)
            OMPL_DEBUG("%s state is not valid", start ? "Start" : "Goal");
    }
    else
        OMPL_DEBUG("%s state is not within space bounds", start ? "Start" : "Goal");

    if (!b || !v)
    {
        std::stringstream ss;
        si_->printState(state, ss);
        ss << " within distance " << dist;
        OMPL_DEBUG("Attempting to fix %s state %s", start ? "start" : "goal", ss.str().c_str());

        State *temp = si_->allocState();
        if (si_->searchValidNearby(temp, state, dist, attempts))
        {
            si_->copyState(state, temp);
            result = true;
        }
        else
            OMPL_WARN("Unable to fix %s state", start ? "start" : "goal");
        si_->freeState(temp);
    }

    return result;
}

bool ompl::base::ProblemDefinition::fixInvalidInputStates(double distStart, double distGoal, unsigned int attempts)
{
    bool result = true;

    // fix start states
    for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
        if (!fixInvalidInputState(startStates_[i], distStart, true, attempts))
            result = false;

    // fix goal state
    GoalState *goal = dynamic_cast<GoalState*>(goal_.get());
    if (goal)
    {
        if (!fixInvalidInputState(const_cast<State*>(goal->getState()), distGoal, false, attempts))
            result = false;
    }

    // fix goal state
    GoalStates *goals = dynamic_cast<GoalStates*>(goal_.get());
    if (goals)
    {
        for (unsigned int i = 0; i < goals->getStateCount(); ++i)
            if (!fixInvalidInputState(const_cast<State*>(goals->getState(i)), distGoal, false, attempts))
                result = false;
    }

    return result;
}

void ompl::base::ProblemDefinition::getInputStates(std::vector<const State*> &states) const
{
    states.clear();
    for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
        states.push_back(startStates_[i]);

    GoalState *goal = dynamic_cast<GoalState*>(goal_.get());
    if (goal)
        states.push_back(goal->getState());

    GoalStates *goals = dynamic_cast<GoalStates*>(goal_.get());
    if (goals)
        for (unsigned int i = 0; i < goals->getStateCount(); ++i)
            states.push_back (goals->getState(i));
}

ompl::base::PathPtr ompl::base::ProblemDefinition::isStraightLinePathValid() const
{
    PathPtr path;
    if (control::SpaceInformationPtr sic = boost::dynamic_pointer_cast<control::SpaceInformation, SpaceInformation>(si_))
    {
        unsigned int startIndex;
        if (isTrivial(&startIndex, NULL))
        {
            control::PathControl *pc = new control::PathControl(sic);
            pc->append(startStates_[startIndex]);
            control::Control *null = sic->allocControl();
            sic->nullControl(null);
            pc->append(startStates_[startIndex], null, 0.0);
            sic->freeControl(null);
            path.reset(pc);
        }
        else
        {
            control::Control *nc = sic->allocControl();
            State *result1 = sic->allocState();
            State *result2 = sic->allocState();
            sic->nullControl(nc);

            for (unsigned int k = 0 ; k < startStates_.size() && !path ; ++k)
            {
                const State *start = startStates_[k];
                if (start && si_->isValid(start) && si_->satisfiesBounds(start))
                {
                    sic->copyState(result1, start);
                    for (unsigned int i = 0 ; i < sic->getMaxControlDuration() && !path ; ++i)
                        if (sic->propagateWhileValid(result1, nc, 1, result2))
                        {
                            if (goal_->isSatisfied(result2))
                            {
                                control::PathControl *pc = new control::PathControl(sic);
                                pc->append(start);
                                pc->append(result2, nc, (i + 1) * sic->getPropagationStepSize());
                                path.reset(pc);
                                break;
                            }
                            std::swap(result1, result2);
                        }
                }
            }
            sic->freeState(result1);
            sic->freeState(result2);
            sic->freeControl(nc);
        }
    }
    else
    {
        std::vector<const State*> states;
        GoalState *goal = dynamic_cast<GoalState*>(goal_.get());
        if (goal)
            if (si_->isValid(goal->getState()) && si_->satisfiesBounds(goal->getState()))
                states.push_back(goal->getState());
        GoalStates *goals = dynamic_cast<GoalStates*>(goal_.get());
        if (goals)
            for (unsigned int i = 0; i < goals->getStateCount(); ++i)
                if (si_->isValid(goals->getState(i)) && si_->satisfiesBounds(goals->getState(i)))
                    states.push_back(goals->getState(i));

        if (states.empty())
        {
            unsigned int startIndex;
            if (isTrivial(&startIndex))
            {
                geometric::PathGeometric *pg = new geometric::PathGeometric(si_, startStates_[startIndex], startStates_[startIndex]);
                path.reset(pg);
            }
        }
        else
        {
            for (unsigned int i = 0 ; i < startStates_.size() && !path ; ++i)
            {
                const State *start = startStates_[i];
                if (start && si_->isValid(start) && si_->satisfiesBounds(start))
                {
                    for (unsigned int j = 0 ; j < states.size() && !path ; ++j)
                        if (si_->checkMotion(start, states[j]))
                        {
                            geometric::PathGeometric *pg = new geometric::PathGeometric(si_, start, states[j]);
                            path.reset(pg);
                            break;
                        }
                }
            }
        }
    }

    return path;
}

bool ompl::base::ProblemDefinition::isTrivial(unsigned int *startIndex, double *distance) const
{
    if (!goal_)
    {
        OMPL_ERROR("Goal undefined");
        return false;
    }

    for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
    {
        const State *start = startStates_[i];
        if (start && si_->isValid(start) && si_->satisfiesBounds(start))
        {
            double dist;
            if (goal_->isSatisfied(start, &dist))
            {
                if (startIndex)
                    *startIndex = i;
                if (distance)
                    *distance = dist;
                return true;
            }
        }
        else
        {
            OMPL_ERROR("Initial state is in collision!");
        }
    }

    return false;
}

bool ompl::base::ProblemDefinition::hasSolution() const
{
    return solutions_->getSolutionCount() > 0;
}

std::size_t ompl::base::ProblemDefinition::getSolutionCount() const
{
    return solutions_->getSolutionCount();
}

ompl::base::PathPtr ompl::base::ProblemDefinition::getSolutionPath() const
{
    return solutions_->getTopSolution();
}

bool ompl::base::ProblemDefinition::getSolution(PlannerSolution& solution) const
{
    return solutions_->getTopSolution(solution);
}

void ompl::base::ProblemDefinition::addSolutionPath(const PathPtr &path, bool approximate, double difference, const std::string& plannerName) const
{
    if (approximate)
        OMPL_INFORM("ProblemDefinition: Adding approximate solution from planner %s", plannerName.c_str());
    solutions_->add(PlannerSolution(path, approximate, difference, plannerName));
}

void ompl::base::ProblemDefinition::addSolutionPath(const PlannerSolution &sol) const
{
    if (sol.approximate_)
        OMPL_INFORM("ProblemDefinition: Adding approximate solution from planner %s", sol.plannerName_.c_str());
    solutions_->add(sol);
}

bool ompl::base::ProblemDefinition::hasApproximateSolution() const
{
    return solutions_->isApproximate();
}

bool ompl::base::ProblemDefinition::hasOptimizedSolution() const
{
    return solutions_->isOptimized();
}

double ompl::base::ProblemDefinition::getSolutionDifference() const
{
    return solutions_->getDifference();
}

std::vector<ompl::base::PlannerSolution> ompl::base::ProblemDefinition::getSolutions() const
{
    return solutions_->getSolutions();
}

void ompl::base::ProblemDefinition::clearSolutionPaths() const
{
    solutions_->clear();
}

void ompl::base::ProblemDefinition::print(std::ostream &out) const
{
    out << "Start states:" << std::endl;
    for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
        si_->printState(startStates_[i], out);
    if (goal_)
        goal_->print(out);
    else
        out << "Goal = NULL" << std::endl;
    if (optimizationObjective_)
        out << "Average state cost: " << optimizationObjective_->averageStateCost(magic::TEST_STATE_COUNT) << std::endl;
    out << "There are " << solutions_->getSolutionCount() << " solutions" << std::endl;
}

bool ompl::base::ProblemDefinition::hasSolutionNonExistenceProof() const
{
    return nonExistenceProof_.get();
}

void ompl::base::ProblemDefinition::clearSolutionNonExistenceProof()
{
    nonExistenceProof_.reset();
}

const ompl::base::SolutionNonExistenceProofPtr& ompl::base::ProblemDefinition::getSolutionNonExistenceProof() const
{
    return nonExistenceProof_;
}

void ompl::base::ProblemDefinition::setSolutionNonExistenceProof(const ompl::base::SolutionNonExistenceProofPtr& nonExistenceProof)
{
    nonExistenceProof_ = nonExistenceProof;
}
