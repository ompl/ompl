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

#include "ompl/geometric/planners/trajopt/TrajOpt.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/objectives/ConvexifiableOptimization.h"
#include "ompl/trajopt/typedefs.hpp"
#include "ompl/trajopt/expr_ops.hpp"
#include "ompl/trajopt/solver_interface.hpp"
#include "ompl/trajopt/optimizers.hpp"
#include "ompl/trajopt/utils.hpp"

ompl::geometric::TrajOpt::TrajOpt(const ompl::base::SpaceInformationPtr &si)
  : base::Planner(si, "TrajOpt") {}

// TODO: write
ompl::geometric::TrajOpt::~TrajOpt() {}
void ompl::geometric::TrajOpt::clear() {}

void ompl::geometric::TrajOpt::setup()
{
    Planner::setup();
    problem_ = std::make_shared<OmplOptProb>(nSteps_, si_);

    // TODO: assuming only one start state. Should look into handling multiple
    // smartly, possibly to better handle multiple intiations.
    if (pdef_->getStartStateCount() <= 0) {
        throw Exception("Need at least 1 start state.");
    }

    // TODO: get a method to take a state and a time stamp to turn it into a constraint.
    ompl::base::SE2StateSpace::StateType *start = pdef_->getStartState(0)
            ->as<ompl::base::SE2StateSpace::StateType>();
    ompl::base::SE2StateSpace::StateType *goal = pdef_->getGoal()->as<ompl::base::GoalState>()
            ->getState()->as<ompl::base::SE2StateSpace::StateType>();
    // There's no way to index into SE2, so just grab the x, then the y, then the yaw.
    problem_->addLinearConstraint(sco::exprSub(
            sco::AffExpr(problem_->traj_vars_(0, 0)), start->getX()), sco::EQ);
    problem_->addLinearConstraint(sco::exprSub(
            sco::AffExpr(problem_->traj_vars_(0, 1)), start->getY()), sco::EQ);
    problem_->addLinearConstraint(sco::exprSub(
            sco::AffExpr(problem_->traj_vars_(0, 2)), start->getYaw()), sco::EQ);
    problem_->addLinearConstraint(sco::exprSub(
            sco::AffExpr(problem_->traj_vars_(nSteps_- 1, 0)), goal->getX()), sco::EQ);
    problem_->addLinearConstraint(sco::exprSub(
            sco::AffExpr(problem_->traj_vars_(nSteps_ - 1, 1)), goal->getY()), sco::EQ);
    problem_->addLinearConstraint(sco::exprSub(
            sco::AffExpr(problem_->traj_vars_(nSteps_ - 1, 2)), goal->getYaw()), sco::EQ);

    // TODO: for now, make the initial trajectory a linear interpolation.
    trajopt::TrajArray ta(nSteps_, 3); // TODO replace with vector length.
    for (int i = 0; i < nSteps_; i++) {
        ta(i, 0) = start->getX() + (goal->getX() - start->getX()) * i / (nSteps_ - 1);
        ta(i, 1) = start->getY() + (goal->getY() - start->getY()) * i / (nSteps_ - 1);
        ta(i, 2) = start->getYaw() + (goal->getYaw() - start->getYaw()) * i / (nSteps_ - 1);
    }
    problem_->SetInitTraj(ta);

    // Grab the problem definition (from parent Planner class) to get all of the
    //   Optmization objectives.
    static_cast<ompl::base::ConvexifiableOptimization *>(
            pdef_->getOptimizationObjective().get())->addToProblem(problem_);

    // Finally, initialize the SQP/Model with all of the variables and costs/constraints.
    // TODO: ?
    sqpOptimizer = new sco::BasicTrustRegionSQP(problem_);

    sqpOptimizer->maxIter_ = maxIter_;
    sqpOptimizer->minApproxImproveFrac_ = minApproxImproveFrac_;
    sqpOptimizer->improve_ratio_threshold_ = 0.2;
    sqpOptimizer->merit_error_coeff_ = initPenaltyCoef_;
    sqpOptimizer->initialize(trajopt::trajToDblVec(problem_->GetInitTraj()));
}

ompl::base::PlannerStatus ompl::geometric::TrajOpt::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    sqpOptimizer->optimize();
    sco::OptResults &results = sqpOptimizer->results();
    switch(results.status) {
        case sco::OPT_CONVERGED: {
            // TODO: check that the path actually is collision free.
            // If not, return APPROXIMATE_SOlUTION.
            trajopt::TrajArray ta = trajopt::getTraj(results.x, problem_->GetVars());
            ompl::base::PlannerSolution solution(trajFromTraj2Ompl(ta));

            solution.setOptimized(pdef_->getOptimizationObjective(),
                    ompl::base::Cost(results.total_cost), true);
            pdef_->addSolutionPath(solution);
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION);
            break;
        }

        case sco::OPT_SCO_ITERATION_LIMIT:
        case sco::OPT_PENALTY_ITERATION_LIMIT:
            return ompl::base::PlannerStatus(ompl::base::PlannerStatus::StatusType::TIMEOUT);
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

ompl::base::PathPtr ompl::geometric::TrajOpt::trajFromTraj2Ompl(trajopt::TrajArray traj) {
    auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
    // t = timestep.
    for (int t = 0; t < traj.rows(); t++) {
        ompl::base::State *s = si_->allocState();
        ompl::base::SE2StateSpace::StateType *se2 =
                s->as<ompl::base::SE2StateSpace::StateType>();
        // TODO: unhardcode.
        se2->setX(traj(t, 0));
        se2->setY(traj(t, 1));
        se2->setYaw(traj(t, 2));
        /*for (int d = 0; d < traj.cols(); d++) {
            (*rvs)[d] = traj(t, d);
        }*/
        path->append(s);
    }
    return path;
}
