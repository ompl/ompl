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

/* Author: Vrushabh Zinage */


#include "ompl/geometric/planners/rrt/RIRRTStar.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>
#include <algorithm>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/GeometricEquations.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {
        RIRRTStar::Motion::Motion(const ob::SpaceInformationPtr &si)
            : state(si->allocState()), parent(nullptr), cost(ob::Cost(0.0)), incCost(ob::Cost(0.0))
        {
        }

        RIRRTStar::RIRRTStar(const ob::SpaceInformationPtr &si, int dimension, double alpha, const Eigen::MatrixXd &W)
            : ob::Planner(si, "RIRRTStar"), d_(dimension), alpha_(alpha), W_(W)
        {
            specs_.approximateSolutions = false;
            specs_.optimizingPaths = true;

            goalBias_ = 0.05;
            maxDistance_ = 0.0;

            Planner::declareParam<double>("range", this, &RIRRTStar::setRange, &RIRRTStar::getRange, "0.:1.:10000.");
            Planner::declareParam<double>("goal_bias", this, &RIRRTStar::setGoalBias, &RIRRTStar::getGoalBias, "0.:.05:1.");

            collisionCheck_ = (d_ == 2);
        }

        RIRRTStar::~RIRRTStar()
        {
            freeMemory();
        }

        void RIRRTStar::setup()
        {
            ob::Planner::setup();

            if (!nn_)
                nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
            nn_->setDistanceFunction([this](const Motion *a, const Motion *b) {
                return si_->distance(a->state, b->state);
            });

            if (!sampler_)
                sampler_ = si_->allocValidStateSampler();

            if (!opt_)
            {
                OMPL_INFORM("%s: No optimization objective specified. Using default objective.", getName().c_str());
                opt_ = std::make_shared<ob::PathLengthOptimizationObjective>(si_);
            }

            if (maxDistance_ < std::numeric_limits<double>::epsilon())
                maxDistance_ = si_->getMaximumExtent() * 0.2;

            validityChecker_ = si_->getStateValidityChecker();
            motionValidator_ = si_->getMotionValidator();

            bestCost_ = opt_->infiniteCost();
        }

        void RIRRTStar::clear()
        {
            ob::Planner::clear();
            sampler_.reset();
            freeMemory();
            if (nn_)
                nn_->clear();
            bestCost_ = opt_->infiniteCost();
        }

        void RIRRTStar::freeMemory()
        {
            std::vector<Motion *> motions;
            if (nn_)
                nn_->list(motions);

            for (auto &motion : motions)
            {
                if (motion->state)
                    si_->freeState(motion->state);
                delete motion;
            }
        }

        void RIRRTStar::addMotion(Motion *motion)
        {
            nn_->add(motion);
        }

        ob::PlannerStatus RIRRTStar::solve(const ob::PlannerTerminationCondition &ptc)
        {
            checkValidity();

            ob::GoalSampleableRegion *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

            while (const ob::State *st = pis_.nextStart())
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->cost = opt_->identityCost();
                motion->incCost = opt_->identityCost();
                nn_->add(motion);
            }

            if (nn_->size() == 0)
            {
                OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
                return ob::PlannerStatus::INVALID_START;
            }

            OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

            Motion *solution = nullptr;
            auto *rmotion = new Motion(si_);
            ob::State *rstate = rmotion->state;

            while (!ptc)
            {
                /* Sample random state (with goal biasing) */
                if (goal->canSample() && rng_.uniform01() < goalBias_)
                {
                    goal->sampleGoal(rstate);
                }
                else
                {
                    if (!sampler_->sample(rstate))
                        continue;
                }

                /* Find nearest state in the tree */
                Motion *nmotion = nn_->nearest(rmotion);

                /* Extend towards the sampled state */
                ob::State *dstate = rstate;

                double d = si_->distance(nmotion->state, rstate);
                if (d > maxDistance_)
                {
                    dstate = si_->allocState();
                    si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, dstate);
                }

                if (checkMotion(nmotion->state, dstate))
                {
                    /* Create a new motion */
                    auto *newMotion = new Motion(si_);
                    si_->copyState(newMotion->state, dstate);
                    newMotion->parent = nmotion;
                    newMotion->incCost = ob::Cost(computeCost(nmotion->state, newMotion->state));
                    newMotion->cost = opt_->combineCosts(nmotion->cost, newMotion->incCost);

                    /* Rewire the tree */
                    std::vector<Motion *> nbh;
                    nn_->nearestK(newMotion, 10, nbh); // Adjust k-nearest neighbors as needed

                    for (auto &neighbor : nbh)
                    {
                        if (neighbor == nmotion)
                            continue;

                        if (checkMotion(neighbor->state, newMotion->state))
                        {
                            double incCost = computeCost(neighbor->state, newMotion->state);
                            ob::Cost cost = opt_->combineCosts(neighbor->cost, ob::Cost(incCost));

                            if (opt_->isCostBetterThan(cost, newMotion->cost))
                            {
                                newMotion->incCost = ob::Cost(incCost);
                                newMotion->cost = cost;
                                newMotion->parent = neighbor;
                            }
                        }
                    }

                    addMotion(newMotion);

                    /* Check if the new motion reaches the goal */
                    if (goal->isSatisfied(newMotion->state))
                    {
                        if (opt_->isCostBetterThan(newMotion->cost, bestCost_))
                        {
                            bestCost_ = newMotion->cost;
                            solution = newMotion;
                        }
                    }
                }

                if (dstate != rstate)
                    si_->freeState(dstate);
            }

            bool solved = false;
            if (solution != nullptr)
            {
                /* Construct the solution path */
                std::vector<Motion *> mpath;
                while (solution != nullptr)
                {
                    mpath.push_back(solution);
                    solution = solution->parent;
                }

                /* Set the solution path */
                auto path(std::make_shared<og::PathGeometric>(si_));
                for (int i = mpath.size() - 1; i >= 0; --i)
                    path->append(mpath[i]->state);

                /* Use PlannerSolution to add the solution path */
                ob::PlannerSolution psol(path);
                psol.setPlannerName(getName());
                psol.setOptimized(opt_, bestCost_, true);
                pdef_->addSolutionPath(psol);

                solved = true;
            }

            delete rmotion;

            OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

            return solved ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
        }

        void RIRRTStar::extractState(const ob::State *state, Eigen::VectorXd &x, Eigen::MatrixXd &P) const
        {
            const auto *rv_state = state->as<ob::RealVectorStateSpace::StateType>();

            x.resize(d_);
            for (int i = 0; i < d_; ++i)
                x(i) = rv_state->values[i];

            size_t idx = d_;
            std::vector<double> P_vectorized;
            P_vectorized.reserve(d_ * (d_ + 1) / 2);
            for (int i = 0; i < d_; ++i)
            {
                for (int j = i; j < d_; ++j)
                {
                    P_vectorized.push_back(rv_state->values[idx++]);
                }
            }

            P = Eigen::MatrixXd::Zero(d_, d_);
            idx = 0;
            for (int i = 0; i < d_; ++i)
            {
                for (int j = i; j < d_; ++j)
                {
                    P(i, j) = P_vectorized[idx];
                    if (i != j)
                        P(j, i) = P_vectorized[idx];
                    ++idx;
                }
            }
        }

        double RIRRTStar::computeCost(const ob::State *s1, const ob::State *s2) const
        {
            Eigen::VectorXd x_k, x_k1;
            Eigen::MatrixXd P_k, P_k1;

            extractState(s1, x_k, P_k);
            extractState(s2, x_k1, P_k1);

            double D_travel = (x_k1 - x_k).norm();

            Eigen::MatrixXd P_hat = P_k + D_travel * W_;

            // Compute D_info using the analytical solution
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_k1.inverse() * P_hat);
            Eigen::VectorXd sigma = es.eigenvalues();

            for (int i = 0; i < sigma.size(); ++i)
            {
                if (sigma(i) <= 0)
                    sigma(i) = 1e-6;
            }

            Eigen::VectorXd S_star = sigma.unaryExpr([](double val) { return std::min(1.0, val); });

            double log_det_P_hat = std::log(P_hat.determinant());
            double log_det_Q_star = std::log(P_k1.determinant()) + S_star.array().log().sum();

            double D_info = 0.5 * (log_det_P_hat - log_det_Q_star);

            double D_total = D_travel + alpha_ * D_info;

            return D_total;
        }

        bool RIRRTStar::isStateValid(const ob::State *state) const
        {
            if (!collisionCheck_)
                return true;
            else
                return validityChecker_->isValid(state);
        }

        bool RIRRTStar::checkMotion(const ob::State *s1, const ob::State *s2) const
        {
            if (!collisionCheck_)
                return true;
            else
                return motionValidator_->checkMotion(s1, s2);
        }

    } // namespace geometric
} // namespace ompl

