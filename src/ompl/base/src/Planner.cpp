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

#include "ompl/base/Planner.h"
#include "ompl/util/Exception.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include <sstream>
#include <thread>
#include <utility>

ompl::base::Planner::Planner(SpaceInformationPtr si, std::string name)
  : si_(std::move(si)), pis_(this), name_(std::move(name)), setup_(false)
{
    if (!si_)
        throw Exception(name_, "Invalid space information instance for planner");
}

const ompl::base::PlannerSpecs &ompl::base::Planner::getSpecs() const
{
    return specs_;
}

const std::string &ompl::base::Planner::getName() const
{
    return name_;
}

void ompl::base::Planner::setName(const std::string &name)
{
    name_ = name;
}

const ompl::base::SpaceInformationPtr &ompl::base::Planner::getSpaceInformation() const
{
    return si_;
}

const ompl::base::ProblemDefinitionPtr &ompl::base::Planner::getProblemDefinition() const
{
    return pdef_;
}

ompl::base::ProblemDefinitionPtr &ompl::base::Planner::getProblemDefinition()
{
    return pdef_;
}

void ompl::base::Planner::setProblemDefinition(const ProblemDefinitionPtr &pdef)
{
    pdef_ = pdef;
    pis_.update();
}

const ompl::base::PlannerInputStates &ompl::base::Planner::getPlannerInputStates() const
{
    return pis_;
}

void ompl::base::Planner::setup()
{
    if (!si_->isSetup())
    {
        OMPL_INFORM("%s: Space information setup was not yet called. Calling now.", getName().c_str());
        si_->setup();
    }

    if (setup_)
        OMPL_WARN("%s: Planner setup called multiple times", getName().c_str());
    else
        setup_ = true;
}

void ompl::base::Planner::checkValidity()
{
    if (!isSetup())
        setup();
    pis_.checkValidity();
}

bool ompl::base::Planner::isSetup() const
{
    return setup_;
}

void ompl::base::Planner::clear()
{
    pis_.clear();
    pis_.update();
}

void ompl::base::Planner::clearQuery()
{
    clear();
}

void ompl::base::Planner::getPlannerData(PlannerData &data) const
{
    for (const auto &plannerProgressProperty : plannerProgressProperties_)
        data.properties[plannerProgressProperty.first] = plannerProgressProperty.second();
}

ompl::base::PlannerStatus ompl::base::Planner::solve(const PlannerTerminationConditionFn &ptc, double checkInterval)
{
    return solve(PlannerTerminationCondition(ptc, checkInterval));
}

ompl::base::PlannerStatus ompl::base::Planner::solve(double solveTime)
{
    if (solveTime < 1.0)
        return solve(timedPlannerTerminationCondition(solveTime));
    return solve(timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)));
}

void ompl::base::Planner::printProperties(std::ostream &out) const
{
    out << "Planner " + getName() + " specs:" << std::endl;
    out << "Multithreaded:                 " << (getSpecs().multithreaded ? "Yes" : "No") << std::endl;
    out << "Reports approximate solutions: " << (getSpecs().approximateSolutions ? "Yes" : "No") << std::endl;
    out << "Can optimize solutions:        " << (getSpecs().optimizingPaths ? "Yes" : "No") << std::endl;
    out << "Aware of the following parameters:";
    std::vector<std::string> params;
    params_.getParamNames(params);
    for (auto &param : params)
        out << " " << param;
    out << std::endl;
}

void ompl::base::Planner::printSettings(std::ostream &out) const
{
    out << "Declared parameters for planner " << getName() << ":" << std::endl;
    params_.print(out);
}

void ompl::base::PlannerInputStates::clear()
{
    if (tempState_ != nullptr)
    {
        si_->freeState(tempState_);
        tempState_ = nullptr;
    }
    addedStartStates_ = 0;
    sampledGoalsCount_ = 0;
    pdef_ = nullptr;
    si_ = nullptr;
}

void ompl::base::PlannerInputStates::restart()
{
    addedStartStates_ = 0;
    sampledGoalsCount_ = 0;
}

bool ompl::base::PlannerInputStates::update()
{
    if (planner_ == nullptr)
        throw Exception("No planner set for PlannerInputStates");
    return use(planner_->getProblemDefinition());
}

void ompl::base::PlannerInputStates::checkValidity() const
{
    std::string error;

    if (pdef_ == nullptr)
        error = "Problem definition not specified";
    else
    {
        if (pdef_->getStartStateCount() <= 0)
            error = "No start states specified";
        else if (!pdef_->getGoal())
            error = "No goal specified";
    }

    if (!error.empty())
    {
        if (planner_ != nullptr)
            throw Exception(planner_->getName(), error);
        else
            throw Exception(error);
    }
}

bool ompl::base::PlannerInputStates::use(const ProblemDefinitionPtr &pdef)
{
    if (pdef)
        return use(pdef.get());

    clear();
    return true;
}

bool ompl::base::PlannerInputStates::use(const ProblemDefinition *pdef)
{
    if (pdef_ != pdef)
    {
        clear();
        pdef_ = pdef;
        si_ = pdef->getSpaceInformation().get();
        return true;
    }
    return false;
}

const ompl::base::State *ompl::base::PlannerInputStates::nextStart()
{
    if (pdef_ == nullptr || si_ == nullptr)
    {
        std::string error = "Missing space information or problem definition";
        if (planner_ != nullptr)
            throw Exception(planner_->getName(), error);
        else
            throw Exception(error);
    }

    while (addedStartStates_ < pdef_->getStartStateCount())
    {
        const base::State *st = pdef_->getStartState(addedStartStates_);
        addedStartStates_++;
        bool bounds = si_->satisfiesBounds(st);
        bool valid = bounds ? si_->isValid(st) : false;
        if (bounds && valid)
            return st;

        OMPL_WARN("%s: Skipping invalid start state (invalid %s)",
                  planner_ ? planner_->getName().c_str() : "PlannerInputStates", bounds ? "state" : "bounds");
        std::stringstream ss;
        si_->printState(st, ss);
        OMPL_DEBUG("%s: Discarded start state %s", planner_ ? planner_->getName().c_str() : "PlannerInputStates",
                   ss.str().c_str());
    }
    return nullptr;
}

const ompl::base::State *ompl::base::PlannerInputStates::nextGoal()
{
    // This initialization is safe since we are in a non-const function anyway.
    static PlannerTerminationCondition ptc = plannerAlwaysTerminatingCondition();
    return nextGoal(ptc);
}

const ompl::base::State *ompl::base::PlannerInputStates::nextGoal(const PlannerTerminationCondition &ptc)
{
    if (pdef_ == nullptr || si_ == nullptr)
    {
        std::string error = "Missing space information or problem definition";
        if (planner_ != nullptr)
            throw Exception(planner_->getName(), error);
        else
            throw Exception(error);
    }

    if (pdef_->getGoal() != nullptr)
    {
        const GoalSampleableRegion *goal =
            pdef_->getGoal()->hasType(GOAL_SAMPLEABLE_REGION) ? pdef_->getGoal()->as<GoalSampleableRegion>() : nullptr;

        if (goal != nullptr)
        {
            time::point start_wait;
            bool first = true;
            bool attempt = true;
            while (attempt)
            {
                attempt = false;

                if (sampledGoalsCount_ < goal->maxSampleCount() && goal->canSample())
                {
                    if (tempState_ == nullptr)
                        tempState_ = si_->allocState();
                    do
                    {
                        goal->sampleGoal(tempState_);
                        sampledGoalsCount_++;
                        bool bounds = si_->satisfiesBounds(tempState_);
                        bool valid = bounds ? si_->isValid(tempState_) : false;
                        if (bounds && valid)
                        {
                            if (!first)  // if we waited, show how long
                            {
                                OMPL_DEBUG("%s: Waited %lf seconds for the first goal sample.",
                                           planner_ ? planner_->getName().c_str() : "PlannerInputStates",
                                           time::seconds(time::now() - start_wait));
                            }
                            return tempState_;
                        }

                        OMPL_WARN("%s: Skipping invalid goal state (invalid %s)",
                                  planner_ ? planner_->getName().c_str() : "PlannerInputStates",
                                  bounds ? "state" : "bounds");
                        std::stringstream ss;
                        si_->printState(tempState_, ss);
                        OMPL_DEBUG("%s: Discarded goal state:\n%s",
                                   planner_ ? planner_->getName().c_str() : "PlannerInputStates", ss.str().c_str());
                    } while (!ptc && sampledGoalsCount_ < goal->maxSampleCount() && goal->canSample());
                }
                if (goal->couldSample() && !ptc)
                {
                    if (first)
                    {
                        first = false;
                        start_wait = time::now();
                        OMPL_DEBUG("%s: Waiting for goal region samples ...",
                                   planner_ ? planner_->getName().c_str() : "PlannerInputStates");
                    }
                    std::this_thread::sleep_for(time::seconds(0.01));
                    attempt = !ptc;
                }
            }
        }
    }

    return nullptr;
}

bool ompl::base::PlannerInputStates::haveMoreStartStates() const
{
    if (pdef_ != nullptr)
        return addedStartStates_ < pdef_->getStartStateCount();
    return false;
}

bool ompl::base::PlannerInputStates::haveMoreGoalStates() const
{
    if ((pdef_ != nullptr) && pdef_->getGoal())
        if (pdef_->getGoal()->hasType(GOAL_SAMPLEABLE_REGION))
            return sampledGoalsCount_ < pdef_->getGoal()->as<GoalSampleableRegion>()->maxSampleCount();
    return false;
}
