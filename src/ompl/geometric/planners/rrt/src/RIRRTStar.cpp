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
#include <boost/math/distributions/chi_squared.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h> // Include PathLengthOptimizationObjective
#include <ompl/base/spaces/RealVectorStateSpace.h> // Include RealVectorStateSpace
#include <ompl/util/GeometricEquations.h> // Include GeometricEquations for unitNBallMeasure

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {
        RIRRTStar::Motion::Motion(const base::SpaceInformationPtr &si)
            : state(si->allocState()), parent(nullptr), cost(base::Cost(0.0)), incCost(base::Cost(0.0))
        {
        }

        RIRRTStar::RIRRTStar(const base::SpaceInformationPtr &si, int dimension, double alpha, const Eigen::MatrixXd &W)
            : base::Planner(si, "RIRRTStar"), d_(dimension), alpha_(alpha), W_(W)
        {
            specs_.approximateSolutions = false;
            specs_.optimizingPaths = true;

            goalBias_ = 0.05;
            maxDistance_ = 0.0;

            Planner::declareParam<double>("range", this, &RIRRTStar::setRange, &RIRRTStar::getRange, "0.:1.:10000.");
            Planner::declareParam<double>("goal_bias", this, &RIRRTStar::setGoalBias, &RIRRTStar::getGoalBias, "0.:.05:1.");

            // Determine if collision checking is needed
            collisionCheck_ = (d_ == 2);
        }

        RIRRTStar::~RIRRTStar()
        {
            freeMemory();
        }

        void RIRRTStar::setup()
        {
            base::Planner::setup();

            if (!nn_)
                nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
            nn_->setDistanceFunction([this](const Motion *a, const Motion *b) {
                return computeCost(a->state, b->state);
            });

            if (!sampler_)
                sampler_ = si_->allocValidStateSampler();

            if (!opt_)
            {
                OMPL_INFORM("%s: No optimization objective specified. Using default objective.", getName().c_str());
                opt_ = std::make_shared<ob::PathLengthOptimizationObjective>(si_);
            }

            // Remove the call to opt_->setup(); if it's not available in your OMPL version

            if (maxDistance_ < std::numeric_limits<double>::epsilon())
            {
                maxDistance_ = si_->getMaximumExtent() * 0.2;
            }

            // Set up the state validity checker
            validityChecker_ = si_->getStateValidityChecker();

            // Set up the motion validator
            motionValidator_ = si_->getMotionValidator();

            bestCost_ = opt_->infiniteCost();
        }

        void RIRRTStar::clear()
        {
            Planner::clear();
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

        void RIRRTStar::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
        {
            Planner::setProblemDefinition(pdef);
            opt_ = pdef_->getOptimizationObjective();
        }

        double RIRRTStar::calculateRadius() const
        {
            double volume = si_->getStateSpace()->getMeasure();
            double unitBallVolume = ompl::unitNBallMeasure(si_->getStateDimension()); // Corrected namespace
            double r = std::pow((2 * (1 + 1.0 / si_->getStateDimension())) *
                                (volume / unitBallVolume), 1.0 / si_->getStateDimension());
            return std::min(r * std::pow(std::log((double)(nn_->size() + 1)) / (double)(nn_->size() + 1), 1.0 / si_->getStateDimension()), maxDistance_);
        }

        void RIRRTStar::addMotion(Motion *motion)
        {
            nn_->add(motion);
        }

        base::PlannerStatus RIRRTStar::solve(const base::PlannerTerminationCondition &ptc)
        {
            checkValidity();

            base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

            while (const base::State *st = pis_.nextStart())
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
                return base::PlannerStatus::INVALID_START;
            }

            OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

            Motion *solution = nullptr;
            auto *rmotion = new Motion(si_);
            base::State *rstate = rmotion->state;

            while (!ptc)
            {
                // Sample a random state
                if (goal->canSample() && rng_.uniform01() < goalBias_)
                {
                    goal->sampleGoal(rstate);
                }
                else
                {
                    if (!sampler_->sample(rstate))
                    {
                        continue;
                    }
                }

                // Find nearest neighbor
                Motion *nmotion = nn_->nearest(rmotion);

                // Extend towards the sampled state
                double d = computeCost(nmotion->state, rstate);
                if (d > maxDistance_)
                {
                    // Interpolate between nmotion->state and rstate
                    si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, rstate);
                }

                // Check motion validity
                if (checkMotion(nmotion->state, rstate))
                {
                    // Create a new motion
                    auto *newMotion = new Motion(si_);
                    si_->copyState(newMotion->state, rstate);

                    // Find nearby neighbors
                    double radius = calculateRadius();
                    std::vector<Motion *> neighbors;
                    nn_->nearestR(newMotion, radius, neighbors);

                    // Initialize cost to arrive to newMotion via nmotion
                    newMotion->incCost = base::Cost(computeCost(nmotion->state, newMotion->state));
                    newMotion->cost = opt_->combineCosts(nmotion->cost, newMotion->incCost);
                    newMotion->parent = nmotion;

                    // Choose the best parent
                    for (auto &neighbor : neighbors)
                    {
                        if (neighbor == nmotion)
                            continue;

                        if (checkMotion(neighbor->state, newMotion->state))
                        {
                            double incCost = computeCost(neighbor->state, newMotion->state);
                            base::Cost cost = opt_->combineCosts(neighbor->cost, base::Cost(incCost));

                            if (opt_->isCostBetterThan(cost, newMotion->cost))
                            {
                                newMotion->incCost = base::Cost(incCost);
                                newMotion->cost = cost;
                                newMotion->parent = neighbor;
                            }
                        }
                    }

                    // Add the new motion to the tree
                    newMotion->parent->children.push_back(newMotion);
                    addMotion(newMotion);

                    // Rewire the tree
                    for (auto &neighbor : neighbors)
                    {
                        if (neighbor == newMotion->parent)
                            continue;

                        if (checkMotion(newMotion->state, neighbor->state))
                        {
                            double incCost = computeCost(newMotion->state, neighbor->state);
                            base::Cost cost = opt_->combineCosts(newMotion->cost, base::Cost(incCost));

                            if (opt_->isCostBetterThan(cost, neighbor->cost))
                            {
                                // Update parent
                                neighbor->parent->children.erase(std::remove(neighbor->parent->children.begin(),
                                                                             neighbor->parent->children.end(),
                                                                             neighbor),
                                                                 neighbor->parent->children.end());

                                neighbor->parent = newMotion;
                                neighbor->cost = cost;
                                neighbor->incCost = base::Cost(incCost);
                                newMotion->children.push_back(neighbor);
                            }
                        }
                    }

                    // Check if the new motion reaches the goal
                    if (goal->isSatisfied(newMotion->state))
                    {
                        if (opt_->isCostBetterThan(newMotion->cost, bestCost_))
                        {
                            bestCost_ = newMotion->cost;
                            solution = newMotion;
                        }
                    }
                }
            }

            bool solved = false;
            if (solution != nullptr)
            {
                // Construct the solution path
                std::vector<Motion *> mpath;
                while (solution != nullptr)
                {
                    mpath.push_back(solution);
                    solution = solution->parent;
                }

                // Set the solution path
                auto path(std::make_shared<geometric::PathGeometric>(si_));
                for (int i = mpath.size() - 1; i >= 0; --i)
                    path->append(mpath[i]->state);

                // Use PlannerSolution to add the solution path
                base::PlannerSolution psol(path);
                psol.setPlannerName(getName());
                psol.setOptimized(opt_, bestCost_, true);
                pdef_->addSolutionPath(psol);

                solved = true;
            }

            delete rmotion;

            OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

            return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
        }

        // Helper function to extract x and P from a state
        void RIRRTStar::extractState(const base::State *state, Eigen::VectorXd &x, Eigen::MatrixXd &P) const
        {
            const auto *rv_state = state->as<ob::RealVectorStateSpace::StateType>();
            // Extract x
            x.resize(d_);
            for (int i = 0; i < d_; ++i)
            {
                x(i) = rv_state->values[i];
            }
            // Extract P_vectorized
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
            // Reconstruct P from P_vectorized
            P = Eigen::MatrixXd::Zero(d_, d_);
            idx = 0;
            for (int i = 0; i < d_; ++i)
            {
                for (int j = i; j < d_; ++j)
                {
                    P(i, j) = P_vectorized[idx];
                    if (i != j)
                    {
                        P(j, i) = P_vectorized[idx]; // Since P is symmetric
                    }
                    ++idx;
                }
            }
        }

        // Helper function to compute the cost between two states
        double RIRRTStar::computeCost(const base::State *s1, const base::State *s2) const
        {
            // Extract x_k, x_{k+1}, P_k, P_{k+1}
            Eigen::VectorXd x_k, x_k1;
            Eigen::MatrixXd P_k, P_k1;

            extractState(s1, x_k, P_k);
            extractState(s2, x_k1, P_k1);

            // Compute D_travel = || x_{k+1} - x_k ||
            double D_travel = (x_k1 - x_k).norm();

            // Compute \hat{P}_{k+1} = P_k + || x_{k+1} - x_k || * W_
            Eigen::MatrixXd P_hat = P_k + D_travel * W_;

            // Compute D_info using the analytical solution
            Eigen::MatrixXd P_k1_inv = P_k1.inverse();
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_k1_inv * P_hat);
            Eigen::VectorXd sigma = es.eigenvalues();

            // Ensure eigenvalues are positive
            for (int i = 0; i < sigma.size(); ++i)
            {
                if (sigma(i) <= 0)
                {
                    sigma(i) = 1e-6; // Small positive value
                }
            }

            // Compute S^* = diag(min{1, sigma_i})
            Eigen::VectorXd S_star = sigma.unaryExpr([](double val) { return std::min(1.0, val); });

            // Compute log-det of P_hat and Q^*_{k+1}
            double log_det_P_hat = std::log((P_hat).determinant());

            // Compute log-det of Q^*_{k+1}
            double log_det_Q_star = std::log(P_k1.determinant()) + S_star.array().log().sum();

            double D_info = 0.5 * (log_det_P_hat - log_det_Q_star);

            // Total cost D = D_travel + alpha_ * D_info
            double D_total = D_travel + alpha_ * D_info;

            return D_total;
        }

        // State validity checking
        bool RIRRTStar::isStateValid(const base::State *state) const
        {
            if (!collisionCheck_)
            {
                // No collision checking for dimensions other than 2
                return true;
            }
            else
            {
                // Perform collision checking
                return validityChecker_->isValid(state);
            }
        }

        // Motion validity checking
        bool RIRRTStar::checkMotion(const base::State *s1, const base::State *s2) const
        {
            if (!collisionCheck_)
            {
                // No collision checking for dimensions other than 2
                return true;
            }
            else
            {
                return motionValidator_->checkMotion(s1, s2);
            }
        }

    } // namespace geometric
} // namespace ompl

