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

/* Authors: Bryce Willey */

#ifndef OMPL_BASE_OBJECTIVES_COLLISION_COLLISION_EVALUATOR_
#define OMPL_BASE_OBJECTIVES_COLLISION_COLLISION_EVALUATOR_

#include <functional>
#include "ompl/base/objectives/ConvexifiableConstraint.h"
#include "ompl/base/StateSpace.h"
#include "ompl/trajopt/modeling_utils.h"
#include "ompl/trajopt/typedefs.h"

namespace ompl
{
    namespace base
    {
        /**
         * \brief A callback that generates a Jacobian from a system configuration, and a point on a
         *        particular body/link of the system.
         */
        typedef std::function<Eigen::MatrixXd(std::vector<double>, Eigen::Vector3d, std::string)>
            JacobianFn;

        /**
         * \brief A callback that checks for collision for a system configuration, as well as
         *        returns the closest point on the system to the collision, and the normal vector
         *        (in the workspace) that points to the direction to exit the collision.
         */
        typedef std::function<bool(std::vector<double>,
                                   std::vector<double>& signedDist,
                                   std::vector<Eigen::Vector3d>&,
                                   std::vector<std::string>&,
                                   std::vector<Eigen::Vector3d>&)>
            WorkspaceCollisionFn;

        /**
         * \brief A TrajOpt constraint that uses the Jacobian to approximate the optimization
         *        gradient.
         */
        struct JacobianCollisionEvaluator {
            JacobianCollisionEvaluator(JacobianFn J) : J_(J) {}

            sco::AffExpr distExprFromOne(double signedDist, Eigen::Vector3d normal, Eigen::Vector3d point, Eigen::MatrixXd j, std::vector<double> x_0, std::vector<sco::Var> vars);
            virtual std::vector<sco::AffExpr> calcDistanceExpressions(std::vector<double> x) = 0;
            virtual std::vector<double> calcDistances(std::vector<double> x) = 0;
            ~JacobianCollisionEvaluator() {}
            virtual sco::VarVector getVars() = 0;


            JacobianFn J_;
        };

        struct JacobianDiscreteCollisionEvaluator : public JacobianCollisionEvaluator {
            /**
             * Constructor.
             * vars includes just the model variables that are used for this particular timestep.
             */
            JacobianDiscreteCollisionEvaluator(
                    WorkspaceCollisionFn inCollision,
                    StateSpacePtr ss,
                    JacobianFn J,
                    sco::VarVector vars);

            /**
             * Given the entire trajectory vector (x), calculate the affine expressions for the
             * particular timestep of this evaluator (defined by vars_);
             */
            virtual std::vector<sco::AffExpr> calcDistanceExpressions(std::vector<double> x);

            /**
             * Given the entire trajectory vector (x), calculate the distances from each link for
             * this particular timestep of this evaluator (defined by vars_).
             */
            virtual std::vector<double> calcDistances(std::vector<double> x);
            virtual sco::VarVector getVars();

            WorkspaceCollisionFn inCollision_;
            StateSpacePtr ss_;
            sco::VarVector vars_;
        };

        typedef std::function<bool(std::vector<double>,
                                   std::vector<double>,
                                   std::vector<double>& signedDists,
                                   std::vector<Eigen::Vector3d>&, // p_swept
                                   std::vector<Eigen::Vector3d>&, // p0
                                   std::vector<Eigen::Vector3d>&, // p1
                                   std::vector<std::string>&,
                                   std::vector<Eigen::Vector3d>&)>
            WorkspaceContinuousCollisionFn;

        struct JacobianContinuousCollisionEvaluator : public JacobianCollisionEvaluator {
            // TODO: actually write.
            JacobianContinuousCollisionEvaluator(
                WorkspaceContinuousCollisionFn inCollision,
                StateSpacePtr ss,
                JacobianFn J,
                sco::VarVector vars0,
                sco::VarVector vars1);

            virtual std::vector<sco::AffExpr> calcDistanceExpressions(std::vector<double> x);
            virtual std::vector<double> calcDistances(std::vector<double> x);
            virtual sco::VarVector getVars();

            WorkspaceContinuousCollisionFn inCollision_;
            StateSpacePtr ss_;
            sco::VarVector vars0_;
            sco::VarVector vars1_;
        };

    }
}

#endif
