/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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

/* Authors: John Schulman, Bryce Willey */

#include <chrono>
#include <thread>

#include "ompl/geometric/planners/trajopt/TrajOpt.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalLazySamples.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/objectives/ConvexifiableOptimization.h"
#include "ompl/base/PlannerTerminationCondition.h"

#include "ompl/trajopt/typedefs.h"
#include "ompl/trajopt/expr_ops.h"
#include "ompl/trajopt/solver_interface.h"
#include "ompl/trajopt/optimizers.h"
#include "ompl/trajopt/utils.h"

#include "ompl/util/Console.h"

ompl::geometric::TrajOpt::TrajOpt(const ompl::base::SpaceInformationPtr &si)
  : base::Planner(si, "TrajOpt") {
    // Make tmp file for the path at each iteration.
    fd = fopen("/tmp/tmpfile.txt", "w");
    auto fileLog = new ompl::msg::OutputHandlerFile("/tmp/IHateThis.log");
    ompl::msg::useOutputHandler(fileLog);
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEV2);
    OMPL_WARN("Number of Waypoints: %d", nSteps_);
}

// TODO: write
ompl::geometric::TrajOpt::~TrajOpt() {
    fclose(fd);
}

void ompl::geometric::TrajOpt::clear() {
    Planner::clear();
    pis_.restart();
    sqpOptimizer = nullptr;
    problem_ = std::make_shared<OmplOptProb>(nSteps_, si_);
}

void ompl::geometric::TrajOpt::setup()
{
    Planner::setup();
    problem_ = std::make_shared<OmplOptProb>(nSteps_, si_);
}

ompl::base::PlannerStatus ompl::geometric::TrajOpt::constructOptProblem()
{
    // TODO: assuming only one start state. Should look into handling multiple
    // smartly, possibly to better handle multiple intiations.
    if (!pis_.haveMoreStartStates())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    auto *goalRegion = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goalRegion == nullptr)
    {
        OMPL_ERROR("%S: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    if (!goalRegion->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }
    if (!goalRegion->canSample()) {
        //TODO
        OMPL_ERROR("TODO?");
    }

    pis_.update();
    const ompl::base::State *start = pis_.nextStart();
    const ompl::base::State *goal = pis_.nextGoal(ompl::base::timedPlannerTerminationCondition(1.0));
    if (goal == nullptr) {
        goal = pis_.nextGoal(); // try again
        if (goal == nullptr) {
            OMPL_ERROR("Goal is null? %p", goal);
        }
    }
    OMPL_INFORM("Goal is %p", goal);

    int dof = si_->getStateDimension();

    // TODO: get a method to take a state and a time stamp to turn it into a constraint.
    ompl::base::StateSpacePtr ss = si_->getStateSpace();
    std::vector<double> startVec(dof);
    std::vector<double> endVec(dof);
    ss->copyToReals(startVec, start);
    ss->copyToReals(endVec, goal);
    std::cerr << "-{ start: [";
    for (int i = 0; i < startVec.size(); i++) {
        std::cerr << startVec[i] << ", ";
    }
    std::cerr << "], goal: [";
    for (int i = 0; i < endVec.size(); i++) {
        std::cerr << endVec[i] << ", ";
    }
    std::cerr << "]}" << std::endl;

    for (int i = 0; i < dof; i++) {
        problem_->addLinearConstraint(sco::exprSub(
                sco::AffExpr(problem_->traj_vars_(0, i)), startVec[i]), sco::EQ);
        problem_->addLinearConstraint(sco::exprSub(
                sco::AffExpr(problem_->traj_vars_(nSteps_ - 1, i)), endVec[i]), sco::EQ);
    }

    if (!problem_->InitTrajIsSet()) {
        trajopt::TrajArray ta(nSteps_, dof);
        for (size_t i = 0; i < nSteps_; i++) {
            for (int j = 0; j < dof; j++) {
                ta(i, j) = startVec[j] + (endVec[j] - startVec[j]) * i / (nSteps_ - 1);
            }
        }
        problem_->SetInitTraj(ta);
    }

    // Grab the problem definition to get the Optmization objectives.
    static_cast<ompl::base::ConvexifiableOptimization *>(
            pdef_->getOptimizationObjective().get()
    )->addToProblem(problem_);

    // Finally, initialize the SQP/Model with all of the variables and costs/constraints.
    sqpOptimizer = new sco::BasicTrustRegionSQP(problem_);

    sqpOptimizer->maxIter_ = maxIter_;
    sqpOptimizer->minApproxImproveFrac_ = minApproxImproveFrac_;
    sqpOptimizer->improve_ratio_threshold_ = 0.2;
    sqpOptimizer->merit_error_coeff_ = initPenaltyCoef_;
    sqpOptimizer->initialize(trajopt::trajToDblVec(problem_->GetInitTraj()));
    sqpOptimizer->addCallback([this](sco::OptProb *prob, std::vector<double>& x) {
        plotCallback(x);
    });

    return base::PlannerStatus::EXACT_SOLUTION;
}

ompl::base::PlannerStatus ompl::geometric::TrajOpt::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    // If it has been solved already, just run the optimizer again?
    // TODO: do something smart like trying another random path, or changing opt params.
    if (!pdef_->hasSolution() || !sqpOptimizer)
    {
        // Restart the input state iterator so we can get the start state again.
        pis_.restart();
        problem_ = std::make_shared<OmplOptProb>(nSteps_, si_);
        auto constructStatus = constructOptProblem();
        if (constructStatus != base::PlannerStatus::EXACT_SOLUTION)
        {
            // Unable to even construct the optimization problem.
            return constructStatus;
        }
    }
    return optimize(ptc);
}

ompl::base::PlannerStatus ompl::geometric::TrajOpt::optimize(const ompl::base::PlannerTerminationCondition &ptc)
{
    sqpOptimizer->optimize();
    sco::OptResults &results = sqpOptimizer->results();
    switch(results.status) {
        case sco::OPT_CONVERGED: {
            plotCallback(results.x);
            trajopt::TrajArray ta = trajopt::getTraj(results.x, problem_->GetVars());
            ompl::base::PlannerSolution solution(trajFromTraj2Ompl(ta));
            ompl::geometric::PathGeometric *path = solution.path_->as<PathGeometric>();
            for (size_t i = 0; i < path->getStates().size() - 1; i++) 
            {
                if (!si_->checkMotion(path->getState(i), path->getState(i + 1)))
                {
                    OMPL_WARN("TrajOpt optimized all the way, but final solution still collides");
                    //return ompl::base::PlannerStatus(ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION);
                }
            }     
            solution.setOptimized(pdef_->getOptimizationObjective(),
                                  ompl::base::Cost(results.total_cost), true);
            pdef_->addSolutionPath(solution);
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION);
            break;
        }

        case sco::OPT_SCO_ITERATION_LIMIT:
        case sco::OPT_PENALTY_ITERATION_LIMIT:
            /*if (nSteps_ < 200)
            {
                OMPL_WARN("Wasn't able to find a path with %d waypoint. Trying again with double.", nSteps_);
                nSteps_ = nSteps_ * 2;
                auto lastPath = trajFromTraj2Ompl(trajopt::getTraj(results.x, problem_->GetVars()))->as<PathGeometric>();
                setInitialTrajectory(*lastPath);
                constructOptProblem();
                return optimize(ptc);
            }
            else*/
            {
                OMPL_WARN("Maxed out at nSteps_ == %d", nSteps_);
                //return ompl::base::PlannerStatus(ompl::base::PlannerStatus::StatusType::TIMEOUT);
                
                // Not really, but need to see wrong plans to figure out what's wrong.
                ompl::base::PlannerSolution solution(trajFromTraj2Ompl(trajopt::getTraj(results.x, problem_->GetVars())));
                solution.setOptimized(pdef_->getOptimizationObjective(),
                                      ompl::base::Cost(results.total_cost), true);
                pdef_->addSolutionPath(solution);
                return ompl::base::PlannerStatus(ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION);
            }
            break;

        case sco::OPT_FAILED:
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::StatusType::ABORT);
            break;

        case sco::INVALID:
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::StatusType::CRASH);
            break;
    }
    return ompl::base::PlannerStatus(ompl::base::PlannerStatus::StatusType::UNKNOWN);
}

void ompl::geometric::TrajOpt::plotCallback(std::vector<double>& x) {
    int dof = si_->getStateDimension();
    int steps = x.size() / dof;
    for (int i = 0; i < steps; i++) {
        fprintf(fd, "%f %f\n", x[i * dof + 0], x[i * dof + 1]);
    }
    fprintf(fd, "\n");
}
void ompl::geometric::TrajOpt::setInitialTrajectory(ompl::geometric::PathGeometric inPath) {
    int dof = si_->getStateDimension();
    size_t states = inPath.getStateCount();
    if (states > nSteps_) {
        OMPL_ERROR("Input path has %d states, but trajopt is only working with %d waypoints!", states, nSteps_);
        return;
    }
    if (states < nSteps_) {
        inPath.interpolate(nSteps_);
    }
    auto ss = si_->getStateSpace();

    trajopt::TrajArray ta(nSteps_, dof);
    for (size_t i = 0; i < nSteps_; i++) {
        const ompl::base::State *state = inPath.getState(i);
        std::vector<double> startVec(dof);
        ss->copyToReals(startVec, state);
        for (int j = 0; j < dof; j++) {
            ta(i, j) = startVec[j];
        }
    }

    problem_->SetInitTraj(ta);
}

ompl::base::PathPtr ompl::geometric::TrajOpt::trajFromTraj2Ompl(trajopt::TrajArray traj) {
    auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
    int dof = si_->getStateDimension();
    ompl::base::StateSpacePtr ss = si_->getStateSpace();
    // t is timestep.
    for (int t = 0; t < traj.rows(); t++) {
        std::vector<double> stateVec(dof);
        for (int i = 0; i < dof; i++) {
            stateVec[i] = traj(t, i);
        }
        ompl::base::State *s = si_->allocState();
        ss->copyFromReals(s, stateVec);
        path->append(s);
    }
    return path;
}
