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
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESSaddLinearConstraint INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Bryce Willey */

#ifndef OMPL_BASE_OBJECTIVES_OBSTACLE_CONSTRAINT_
#define OMPL_BASE_OBJECTIVES_OBSTACLE_CONSTRAINT_

#include "ompl/base/objectives/CollisionEvaluator.h"
#include "ompl/base/objectives/ConvexifiableConstraint.h"
#include "ompl/trajopt/modeling_utils.h"
#include "ompl/trajopt/typedefs.h"

namespace ompl
{
    namespace base
    {
        /**
         * \brief A glue function that gets the distance to the nearest object for each
         *        configuration in a TrajOpt trajectory
         */
        struct ObstacleDistanceFunction : public sco::VectorOfVector
        {
            StateValidityCheckerPtr sv_;
            StateSpacePtr ss_;
            double safeDist_;

            ObstacleDistanceFunction(StateValidityCheckerPtr sv, StateSpacePtr ss, double safeDist)
              : sv_(sv), ss_(ss), safeDist_(safeDist)
            {
            }

            Eigen::VectorXd operator()(const Eigen::VectorXd &x) const;
        };

        /**
         * \brief A TrajOpt constraint that approximates the distance gradient from a function.
         *        Should be used if you don't have access to a Jacobian of the system.
         */
        struct NumericalCollisionTrajOptConstraint : public sco::ConstraintFromFunc
        {
            /**
             * \brief Constructor for the TrajOpt Obstacle constraint. The StateValidityCheckerPtr
             *        is used to get the system's clearence from obstacles.
             *        StateSpacePtr is used to convert TrajOpt representation into OMPL
             *        representation.
             *        safe_dist is the minimum distance the system can be from an obsacle before
             *        it is penalized.
             */
            NumericalCollisionTrajOptConstraint(StateValidityCheckerPtr sv, StateSpacePtr ss,
                                                const sco::VarVector &vars, double safe_dist);
        };

        class JacobianCollisionTrajOptConstraint : public sco::IneqConstraint
        {
        public:
            JacobianCollisionTrajOptConstraint(double safeDist, WorkspaceCollisionFn collision, StateSpacePtr ss,
                                               JacobianFn J, sco::VarVector vars);
            virtual sco::ConvexConstraintsPtr convex(const std::vector<double> &x, sco::Model *model);
            virtual std::vector<double> value(const std::vector<double> &x);
            sco::VarVector getVars()
            {
                return eval_->getVars();
            }

        private:
            std::shared_ptr<JacobianCollisionEvaluator> eval_;
            double safeDist_;
            double coeff_ = 10;
        };

        class JacobianContinuousTrajOptConstraint : public sco::IneqConstraint
        {
        public:
            JacobianContinuousTrajOptConstraint(double safeDist, WorkspaceContinuousCollisionFn collision,
                                                StateSpacePtr ss, JacobianFn J, sco::VarVector vars0,
                                                sco::VarVector vars1);
            virtual sco::ConvexConstraintsPtr convex(const std::vector<double> &x, sco::Model *model);
            virtual std::vector<double> value(const std::vector<double> &x);
            sco::VarVector getVars()
            {
                return eval_->getVars();
            }

        private:
            std::shared_ptr<JacobianContinuousCollisionEvaluator> eval_;
            double safeDist_;
            double coeff_ = 10;
        };

        /**
         * \brief An OMPL OptimizationObjective that constrains the robot to avoid obstacles
         *        using signed distances.
         */
        class ObstacleConstraint : public ConvexifiableConstraint
        {
        public:
            /**
             * \brief Creates an obstacle constraint.
             * The SpaceInformationPtr is used to answer distance queries and to copy states from
             * OMPL to TrajOpt representation and back.
             * The safeDist is the minimum distance that the robot is allowed to be from an
             * obstacle. After that, a hinge cost will be added to the optimization objective.
             */
            ObstacleConstraint(const SpaceInformationPtr &si, double safeDist);
            ObstacleConstraint(const SpaceInformationPtr &si, double safeDist, WorkspaceCollisionFn collision,
                               JacobianFn J);
            ObstacleConstraint(const SpaceInformationPtr &si, double safeDist, WorkspaceContinuousCollisionFn collision,
                               JacobianFn J);

            Cost stateCost(const State *s) const override;
            Cost motionCost(const State *s1, const State *s2) const override;

        protected:
            std::vector<sco::ConstraintPtr> toConstraint(sco::OptProbPtr problem) override;
            WorkspaceCollisionFn collision_;
            WorkspaceContinuousCollisionFn continuousCollision_;
            JacobianFn J_;

            bool useJacobians_ = false;
            bool continuous_ = false;
            double safeDist_;
        };
    }
}

#endif
