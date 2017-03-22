/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Zachary Kingston */

#include "ompl/base/Constraint.h"

void ompl::base::Constraint::function(const State *state, Eigen::Ref<Eigen::VectorXd> out) const
{
    function(toVector(state), out);
}

void ompl::base::Constraint::jacobian(const State *state, Eigen::Ref<Eigen::MatrixXd> out) const
{
    jacobian(toVector(state), out);
}

bool ompl::base::Constraint::project(State *state) const
{
    Eigen::Ref<Eigen::VectorXd> x = toVector(state);
    bool ret = project(x);
    fromVector(state, x);
    return ret;
}

double ompl::base::Constraint::distance(const State *state) const
{
    return distance(toVector(state));
}

bool ompl::base::Constraint::isSatisfied(const State *state) const
{
    return isSatisfied(toVector(state));
}

Eigen::Ref<Eigen::VectorXd> ompl::base::Constraint::toVector(const State *state) const
{
    for (unsigned int i = 0; i < n_; ++i)
        vector_[i] = *ambientSpace_->getValueAddressAtIndex(state, i);

    return vector_;
}

void ompl::base::Constraint::fromVector(State *state, const Eigen::VectorXd &x) const
{
    for (unsigned int i = 0; i < n_; ++i)
        *ambientSpace_->getValueAddressAtIndex(state, i) = x[i];
}

void ompl::base::Constraint::jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
{
    Eigen::VectorXd y1 = x;
    Eigen::VectorXd y2 = x;
    Eigen::VectorXd t1(n_ - k_);
    Eigen::VectorXd t2(n_ - k_);

    // Use a 7-point central difference stencil on each column.
    for (std::size_t j = 0; j < n_; j++)
    {
        // Make step size as small as possible while still giving usable accuracy.
        const double h = std::sqrt(std::numeric_limits<double>::epsilon()) * (x[j] >= 1 ? x[j] : 1);

        // Can't assume y1[j]-y2[j] == 2*h because of precision errors.
        y1[j] += h;
        y2[j] -= h;
        function(y1, t1);
        function(y2, t2);
        const Eigen::VectorXd m1 = (t1 - t2) / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        function(y1, t1);
        function(y2, t2);
        const Eigen::VectorXd m2 = (t1 - t2) / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        function(y1, t1);
        function(y2, t2);
        const Eigen::VectorXd m3 = (t1 - t2) / (y1[j] - y2[j]);

        out.col(j) = 1.5 * m1 - 0.6 * m2 + 0.1 * m3;

        // Reset for next iteration.
        y1[j] = y2[j] = x[j];
    }
}

bool ompl::base::Constraint::project(Eigen::Ref<Eigen::VectorXd> x) const
{
    // Newton's method
    unsigned int iter = 0;
    Eigen::VectorXd f(n_ - k_);
    Eigen::MatrixXd j(n_ - k_, n_);

    function(x, f);
    while (f.norm() > projectionTolerance_ && iter++ < projectionMaxIterations_)
    {
        // Compute pseudoinverse of Jacobian
        jacobian(x, j);

        // Eigen::JacobiSVD<Eigen::MatrixXd> svd = j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

        // const double tolerance =
        //     std::numeric_limits<double>::epsilon() * n_ * svd.singularValues().array().abs().maxCoeff();

        // sigma = Eigen::MatrixXd((svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0)).asDiagonal();
        // x -= svd.matrixV() * sigma * svd.matrixU().adjoint() * f;

        // Eigen::JacobiSVD<Eigen::MatrixXd> svd = j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        // x -= svd.solve(f);

        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr = j.colPivHouseholderQr();
        x -= qr.solve(f);

        function(x, f);
    }

    if (iter > projectionMaxIterations_)
        return false;

    return true;
}

double ompl::base::Constraint::distance(const Eigen::VectorXd &x) const
{
    Eigen::VectorXd f(n_ - k_);
    function(x, f);
    return f.norm();
}

bool ompl::base::Constraint::isSatisfied(const Eigen::VectorXd &x) const
{
    return x.allFinite() && distance(x) <= projectionTolerance_;
}
