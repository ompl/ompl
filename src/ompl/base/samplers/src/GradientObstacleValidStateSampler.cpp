/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
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

/* Author: Bryce Willey */

#include "ompl/base/samplers/GradientObstacleValidStateSampler.h"
#include <Eigen/Core>
#include <Eigen/SVD>

ompl::base::GradientObstacleValidStateSampler::GradientObstacleValidStateSampler(const SpaceInformation *si, double epsilon)
  : GradientValidStateSampler(si, epsilon)
{
    static int i = 0;
    of_.open(std::string("/tmp/tmpfile_obs_") + std::to_string(i++) + std::string(".txt"));
}

bool ompl::base::GradientObstacleValidStateSampler::sampleWithEpsilon(State *state, double defaultDist)
{
    // Sample the starting state.
    sampler_->sampleUniform(state);
    bool grad_avaliable;
    // Generalize to Matrix...
    Eigen::MatrixXd grad(1, dof_);
    double dist = si_->getStateValidityChecker()->clearanceWithClosestGradient(state, grad, grad_avaliable);
    if (!grad_avaliable)
    {
        // Can't do anything!
        OMPL_ERROR("Need the gradient to function properly.");
        return false;
    }


    // TODO(brycew): move this variable out
    int max_iterations = 50;
    int iter = 0;
    std::vector<double> state_vec(dof_);
    si_->getStateSpace()->copyToReals(state_vec, state);
    Eigen::Map<Eigen::VectorXd> eig_state(state_vec.data(), dof_);
    while (dist < defaultDist && iter++ < max_iterations)
    {
        si_->getStateSpace()->copyToReals(state_vec, state);
        // Convert to eigen.
        Eigen::Matrix<double, 1, 1> f; f << dist - defaultDist;

        of_ << eig_state.transpose() << std::endl;
        
        eig_state -= grad.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);

        si_->getStateSpace()->copyFromReals(state, state_vec);
        // Need to handle when the state isn't valid at this point.
        // TODO(brycew): do the CHOMP thing of smoothly projecting to the boundary
        // Until then, just do a hard cap and exit?
        si_->getStateSpace()->enforceBounds(state);

        dist = si_->getStateValidityChecker()->clearanceWithClosestGradient(state, grad, grad_avaliable);
        
        if (dist >= defaultDist || iter >= max_iterations)
        {
            of_ << eig_state.transpose() << std::endl;
            of_ << std::endl;
        }
    }
    return dist > 0.0;
}