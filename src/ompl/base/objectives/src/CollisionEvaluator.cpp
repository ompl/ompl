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

#include "ompl/base/objectives/CollisionEvaluator.h"
#include "ompl/trajopt/sco_common.h"
#include "ompl/trajopt/expr_ops.h"
#include "ompl/trajopt/expr_vec_ops.h"
#include "ompl/trajopt/utils.h"
#include "ompl/trajopt/eigen_conversions.h"


sco::AffExpr ompl::base::JacobianCollisionEvaluator::distExprOneVaries(double signedDist, Eigen::Vector3d normal,
        Eigen::MatrixXd j, std::vector<double> x_0, std::vector<sco::Var> vars)
{
    // Make the affine expression:
    // sd(vars) = sd(x_0) + normal^T * J_pa(x_0) * vars - normal^T * J_pa(x_0) * x
    sco::AffExpr dist(signedDist);
    Eigen::VectorXd dist_gradient = normal.transpose() * j;
    sco::exprInc(dist, sco::varDot(dist_gradient, vars));
    sco::exprInc(dist, -dist_gradient.dot(sco::toVectorXd(x_0)));
    return dist;
}

sco::AffExpr ompl::base::JacobianCollisionEvaluator::distExprTwoVaries(double signedDist, Eigen::Vector3d normal,
        Eigen::MatrixXd j_a, Eigen::MatrixXd j_b, std::vector<double> x_0, std::vector<sco::Var>vars)
{
    // Makes the affine expression:
    // signed_distance(vars) = signed_distance(x_0) + (normal^T * J_pa(x_0) - normal^T * J_pb(x_0)) * vars +
    //                                                (normal^T * J_pa(x_0) - normal^T * J_pb(x_0)) * x_0
    sco::AffExpr dist(signedDist);
    Eigen::VectorXd dist_a_grad = normal.transpose() * j_a;
    Eigen::VectorXd dist_b_grad = normal.transpose() * j_b;
    Eigen::VectorXd diff = dist_a_grad - dist_b_grad;
    sco::exprInc(dist, sco::varDot(diff, vars));
    sco::exprInc(dist, -diff.dot(sco::toVectorXd(x_0)));
    return dist;
}

ompl::base::JacobianDiscreteCollisionEvaluator::JacobianDiscreteCollisionEvaluator(
        WorkspaceCollisionFn inCollision, StateSpacePtr ss, JacobianFn J, sco::VarVector vars) :
    JacobianCollisionEvaluator(J), inCollision_(inCollision), ss_(ss), vars_(vars)
{}

std::vector<sco::AffExpr> ompl::base::JacobianDiscreteCollisionEvaluator::calcDistanceExpressions(std::vector<double> x) {
    std::vector<double> x_0 = sco::getDblVec(x, vars_);
    std::vector<sco::AffExpr> exprs;

    // Fields to be filled in by inCollision.
    std::vector<CollisionInfo> collisionStructs;
    if (inCollision_(x_0, collisionStructs)) {
        for (auto collisionStruct : collisionStructs) {
            Eigen::MatrixXd j = J_(collisionStruct, 0);
            if (collisionStruct.points.size() == 2) {
                Eigen::MatrixXd j_b = J_(collisionStruct, 1);
                auto dist = distExprTwoVaries(collisionStruct.signedDist, collisionStruct.normal, j, j_b, x_0, vars_);
                exprs.push_back(dist);
            } else {
                auto dist = distExprOneVaries(collisionStruct.signedDist, collisionStruct.normal, j, x_0, vars_);
                exprs.push_back(dist);
            }
        }
    }
    return exprs;
}

std::vector<double> ompl::base::JacobianDiscreteCollisionEvaluator::calcDistances(std::vector<double> x) {
    std::vector<double> dofVals = sco::getDblVec(x, vars_);
    std::vector<double> distsAtSteps;

    // TODO: We only need the clearance, so maybe pass in another callback that just gives that.
    std::vector<CollisionInfo> collisionStructs;
    bool collision = inCollision_(dofVals, collisionStructs);
    if (collision) {
        for (auto collisionStruct : collisionStructs) {
            distsAtSteps.push_back(collisionStruct.signedDist);
        }
    }
    return distsAtSteps;
}

sco::VarVector ompl::base::JacobianDiscreteCollisionEvaluator::getVars() {
    return vars_;
}

ompl::base::JacobianContinuousCollisionEvaluator::JacobianContinuousCollisionEvaluator(
        WorkspaceContinuousCollisionFn inCollision, StateSpacePtr ss, JacobianFn J, sco::VarVector vars0, sco::VarVector vars1) :
    JacobianCollisionEvaluator(J), inCollision_(inCollision), ss_(ss), vars0_(vars0), vars1_(vars1)
{}

std::vector<sco::AffExpr> ompl::base::JacobianContinuousCollisionEvaluator::calcDistanceExpressions(std::vector<double> x) {
    std::vector<double> x_0 = sco::getDblVec(x, vars0_); // configuration at first time
    std::vector<double> x_1 = sco::getDblVec(x, vars1_); // configuration at the second time
    std::vector<sco::AffExpr> exprs;

    // Fields to be filled in by inCollision.
    std::vector<ContinuousCollisionInfo> collisionStructs;
    if (inCollision_(x_0, x_1, collisionStructs)) {
        for (auto collisionStruct : collisionStructs) {
            // sd(var0, var1) = sd(x_0, x_1) + alpha * normal^T * J_p0(x_0) * (vars0 - x_0) ...
            //                  + (1 - alpha) * normal^T * J_p1(x_1) * (vars1 - x_1)
            Eigen::MatrixXd j0 = J_(collisionStruct.getTimeOne(), 0);
            Eigen::MatrixXd j1 = J_(collisionStruct.getTimeTwo(), 0);
            sco::AffExpr dist0 = distExprOneVaries(collisionStruct.signedDist, collisionStruct.normal, j0, x_0, vars0_);
            sco::AffExpr dist1 = distExprOneVaries(collisionStruct.signedDist, collisionStruct.normal, j1, x_1, vars1_);

            double alpha;
            // Assuming that an incorrect alpha means we have to calulate it ourselves.
            if (collisionStruct.alpha < 0 || collisionStruct.alpha > 1)
            {
                double p1ps = (collisionStruct.p1 - collisionStruct.p_swept).norm();
                alpha = p1ps / (p1ps + (collisionStruct.p0 - collisionStruct.p_swept).norm());
            }
            else
            {
                alpha = collisionStruct.alpha;
            }
            exprScale(dist0, alpha);
            exprScale(dist1, 1 - alpha);

            sco::AffExpr dist(0);
            exprInc(dist, dist0);
            exprInc(dist, dist1);
            cleanupAff(dist);
            exprs.push_back(dist);
        }
    }
    return exprs;
}

std::vector<double> ompl::base::JacobianContinuousCollisionEvaluator::calcDistances(std::vector<double> x) {
    std::vector<double> dof0 = sco::getDblVec(x, vars0_);
    std::vector<double> dof1 = sco::getDblVec(x, vars1_);
    std::vector<double> distsAtSteps;

    std::vector<ContinuousCollisionInfo> collisionStructs;

    bool collision = inCollision_(dof0, dof1, collisionStructs);
    if (collision) {
        for (auto collisionStruct : collisionStructs) {
            distsAtSteps.push_back(collisionStruct.signedDist);
        }
    }
    return distsAtSteps;
}

sco::VarVector ompl::base::JacobianContinuousCollisionEvaluator::getVars() {
    // What a waste of memory, find a better solution for this.
    std::vector<sco::Var> allVars;
    allVars.insert(allVars.begin(), vars1_.begin(), vars1_.begin());
    allVars.insert(allVars.begin(), vars0_.begin(), vars0_.end());
    return allVars;
}
