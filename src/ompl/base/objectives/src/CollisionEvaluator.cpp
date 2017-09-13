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

ompl::base::JacobianDiscreteCollisionEvaluator::JacobianDiscreteCollisionEvaluator(
        WorkspaceCollisionFn inCollision, StateSpacePtr ss, JacobianFn J, sco::VarVector vars) :
    inCollision_(inCollision), ss_(ss), J_(J), vars_(vars)
{}

std::vector<sco::AffExpr> ompl::base::JacobianDiscreteCollisionEvaluator::calcDistanceExpressions(std::vector<double> x) {
    std::vector<double> x_0 = sco::getDblVec(x, vars_);
    //std::cout << "x_0: ";
    //for (int i = 0; i < x_0.size(); i++) {
    //    std::cout << x_0[i] << ", ";
    //}
    //std::cout << std::endl;
    std::vector<sco::AffExpr> exprs;

    // Fields to be filled in by inCollision.
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> normals;
    std::vector<double> signedDists;
    std::vector<std::string> link_names;
    if (inCollision_(x_0, signedDists, points, link_names, normals)) {
        // For each collision, make the affine expression,
        // sd(vars) = sd(x_0) + normal^T * J_pa(x_0) * vars - normal^T * J_pa(x_0) * x
        for (int i = 0; i < signedDists.size(); i++) {
            double signedDist = signedDists[i];
            Eigen::Vector3d normal = normals[i];
            Eigen::Vector3d point = points[i];
            printf("Collided point: %f, %f, %f\n", point[0], point[1], point[2]);
            std::string link_name = link_names[i];

            printf("A collision!: dist: %f\n\tlink_name: %s\n\tnormal: %f, %f, %f\n",
                   signedDist, link_name.c_str(), normal[0], normal[1], normal[2]);
            sco::AffExpr dist(signedDist);
            Eigen::VectorXd dist_gradient = normal.transpose() * J_(x_0, point, link_name);
            sco::exprInc(dist, sco::varDot(dist_gradient, vars_));
            sco::exprInc(dist, -dist_gradient.dot(sco::toVectorXd(x_0)));
            exprs.push_back(dist);
            //std::cout << "dist expression: " << dist << std::endl;
        }
    }
    return exprs;
}

std::vector<double> ompl::base::JacobianDiscreteCollisionEvaluator::calcDistances(std::vector<double> x) {
    std::vector<double> dofVals = sco::getDblVec(x, vars_);
    std::vector<double> distsAtSteps;

    // TODO: We only need the clearance, so maybe pass in another callback that just gives that.
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> normals;
    std::vector<double> signedDists;
    std::vector<std::string> link_names;
    bool collision = inCollision_(dofVals, signedDists, points, link_names, normals);
    if (collision) {
        std::cout << "dofVals: ";
        for (int i = 0; i < dofVals.size(); i++) {
            std::cout << dofVals[i] << ", ";
        }
        std::cout << std::endl;
        printf("dists: ");
        for (int i = 0; i < signedDists.size(); i++) {
            printf("%f, ", signedDists[i]);
        }
        printf("\n\tlink_name: %s\n\tnormal: %f, %f, %f",
            link_names[0].c_str(), normals[0][0], normals[0][1], normals[0][2]);
        distsAtSteps.insert(distsAtSteps.begin(), signedDists.begin(), signedDists.end());
    }
    printf("\n");
    return distsAtSteps;
}

sco::VarVector ompl::base::JacobianDiscreteCollisionEvaluator::getVars() {
    return vars_;
}
