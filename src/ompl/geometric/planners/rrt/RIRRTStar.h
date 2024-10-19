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

#ifndef OMPL_GEOMETRIC_PLANNERS_RIRRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RIRRTSTAR_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <ompl/base/spaces/RealVectorStateSpace.h> // Include RealVectorStateSpace

namespace ompl
{
    namespace geometric
    {
        /** \brief Risk-Informed RRT* (RIRRTStar) planner */
        class RIRRTStar : public base::Planner
        {
        public:
            /** \brief Constructor */
            RIRRTStar(const base::SpaceInformationPtr &si, int dimension, double alpha, const Eigen::MatrixXd &W);

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
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Setup the planner */
            void setup() override;

            /** \brief Set the problem definition */
            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

        protected:
            /** \brief Represents a node in the tree */
            class Motion
            {
            public:
                /** \brief Constructor */
                Motion(const base::SpaceInformationPtr &si);

                /** \brief The state contained by the motion */
                base::State *state;

                /** \brief The parent motion */
                Motion *parent;

                /** \brief The set of child motions */
                std::vector<Motion *> children;

                /** \brief The cost to reach this motion */
                base::Cost cost;

                /** \brief The cost of the parent to this motion */
                base::Cost incCost;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Add a motion to the tree */
            void addMotion(Motion *motion);

            /** \brief Calculate the neighborhood radius */
            double calculateRadius() const;

            /** \brief State sampler */
            base::ValidStateSamplerPtr sampler_;

            /** \brief Nearest neighbor data structure */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The optimization objective */
            base::OptimizationObjectivePtr opt_;

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
            base::Cost bestCost_;

            /** \brief Random number generator */
            ompl::RNG rng_;

            /** \brief Mutex for thread safety */
            std::mutex treeMutex_;

            /** \brief Collision checking flag */
            bool collisionCheck_;

            /** \brief State validity checker */
            base::StateValidityCheckerPtr validityChecker_;

            /** \brief Motion validator */
            base::MotionValidatorPtr motionValidator_;

            /** \brief Helper functions */
            void extractState(const base::State *state, Eigen::VectorXd &x, Eigen::MatrixXd &P) const;
            double computeCost(const base::State *s1, const base::State *s2) const;
            bool isStateValid(const base::State *state) const;
            bool checkMotion(const base::State *s1, const base::State *s2) const;
        };
    } // namespace geometric
} // namespace ompl

#endif // OMPL_GEOMETRIC_PLANNERS_RIRRTSTAR_
// RIRRTStar.h

#ifndef OMPL_GEOMETRIC_PLANNERS_RIRRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RIRRTSTAR_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <Eigen/Dense>
#include <mutex>
#include <vector>
#include <ompl/base/spaces/RealVectorStateSpace.h> // Include RealVectorStateSpace

namespace ompl
{
    namespace geometric
    {
        /** \brief Risk-Informed RRT* (RIRRTStar) planner */
        class RIRRTStar : public base::Planner
        {
        public:
            /** \brief Constructor */
            RIRRTStar(const base::SpaceInformationPtr &si, int dimension, double alpha, const Eigen::MatrixXd &W);

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
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Setup the planner */
            void setup() override;

            /** \brief Set the problem definition */
            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

        protected:
            /** \brief Represents a node in the tree */
            class Motion
            {
            public:
                /** \brief Constructor */
                Motion(const base::SpaceInformationPtr &si);

                /** \brief The state contained by the motion */
                base::State *state;

                /** \brief The parent motion */
                Motion *parent;

                /** \brief The set of child motions */
                std::vector<Motion *> children;

                /** \brief The cost to reach this motion */
                base::Cost cost;

                /** \brief The cost of the parent to this motion */
                base::Cost incCost;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Add a motion to the tree */
            void addMotion(Motion *motion);

            /** \brief Calculate the neighborhood radius */
            double calculateRadius() const;

            /** \brief State sampler */
            base::ValidStateSamplerPtr sampler_;

            /** \brief Nearest neighbor data structure */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The optimization objective */
            base::OptimizationObjectivePtr opt_;

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
            base::Cost bestCost_;

            /** \brief Random number generator */
            ompl::RNG rng_;

            /** \brief Mutex for thread safety */
            std::mutex treeMutex_;

            /** \brief Collision checking flag */
            bool collisionCheck_;

            /** \brief State validity checker */
            base::StateValidityCheckerPtr validityChecker_;

            /** \brief Motion validator */
            base::MotionValidatorPtr motionValidator_;

            /** \brief Helper functions */
            void extractState(const base::State *state, Eigen::VectorXd &x, Eigen::MatrixXd &P) const;
            double computeCost(const base::State *s1, const base::State *s2) const;
            bool isStateValid(const base::State *state) const;
            bool checkMotion(const base::State *s1, const base::State *s2) const;
        };
    } // namespace geometric
} // namespace ompl

#endif // OMPL_GEOMETRIC_PLANNERS_RIRRTSTAR_

