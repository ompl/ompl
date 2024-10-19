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

// RIRRTStar.h

#ifndef OMPL_GEOMETRIC_PLANNERS_RIRRTSTAR_H
#define OMPL_GEOMETRIC_PLANNERS_RIRRTSTAR_H

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <Eigen/Dense>
#include <vector>
#include <mutex>

// Namespace shortcuts
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {
        /** \brief Rationally Inattentive RRT* (RIRRTStar) planner */
        class RIRRTStar : public ob::Planner
        {
        public:
            /** \brief Constructor */
            RIRRTStar(const ob::SpaceInformationPtr &si, int dimension, double alpha, const Eigen::MatrixXd &W);

            ~RIRRTStar() override;

            /** \brief Set the goal bias (probability of sampling the goal) */
            void setGoalBias(double goalBias) { goalBias_ = goalBias; }

            /** \brief Get the goal bias */
            double getGoalBias() const { return goalBias_; }

            /** \brief Set the range distance for extending the tree */
            void setRange(double range) { maxDistance_ = range; }

            /** \brief Get the range distance */
            double getRange() const { return maxDistance_; }

            /** \brief Clear the planner's data structures */
            void clear() override;

            /** \brief Solve the planning problem */
            ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

            /** \brief Setup the planner */
            void setup() override;

        protected:
            /** \brief Represents a node in the tree */
            class Motion
            {
            public:
                /** \brief Constructor */
                Motion(const ob::SpaceInformationPtr &si);

                /** \brief The state contained by the motion */
                ob::State *state;

                /** \brief The parent motion */
                Motion *parent;

                /** \brief The set of child motions */
                std::vector<Motion *> children;

                /** \brief The cost to reach this motion */
                ob::Cost cost;

                /** \brief The incremental cost from the parent to this motion */
                ob::Cost incCost;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Add a motion to the tree */
            void addMotion(Motion *motion);

            /** \brief State sampler */
            ob::ValidStateSamplerPtr sampler_;

            /** \brief Nearest neighbor data structure */
            std::shared_ptr<ompl::NearestNeighbors<Motion *>> nn_;

            /** \brief The optimization objective */
            ob::OptimizationObjectivePtr opt_;

            /** \brief Goal bias */
            double goalBias_;

            /** \brief Maximum distance between states */
            double maxDistance_;

            /** \brief Number of dimensions */
            int d_;

            /** \brief Alpha parameter for the cost function */
            double alpha_;

            /** \brief W matrix for the cost function */
            Eigen::MatrixXd W_;

            /** \brief The best cost found so far */
            ob::Cost bestCost_;

            /** \brief Random number generator */
            ompl::RNG rng_;

            /** \brief Mutex for thread safety */
            std::mutex treeMutex_;

            /** \brief Collision checking flag */
            bool collisionCheck_;

            /** \brief State validity checker */
            ob::StateValidityCheckerPtr validityChecker_;

            /** \brief Motion validator */
            ob::MotionValidatorPtr motionValidator_;

            /** \brief Helper functions */
            void extractState(const ob::State *state, Eigen::VectorXd &x, Eigen::MatrixXd &P) const;
            double computeCost(const ob::State *s1, const ob::State *s2) const;
            bool isStateValid(const ob::State *state) const;
            bool checkMotion(const ob::State *s1, const ob::State *s2) const;
        };
    } // namespace geometric
} // namespace ompl

#endif // OMPL_GEOMETRIC_PLANNERS_RIRRTSTAR_H

