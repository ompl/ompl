/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Caleb Voss and Wilson Beebe
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

/* Authors: Caleb Voss, Wilson Beebe */

#include <utility>

#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"

namespace ompl
{
    namespace magic
    {
        /// Number of sampler to determine mean vector field norm in \ref gVFRRT
        static const unsigned int VFRRT_MEAN_NORM_SAMPLES = 1000;
    }
}

ompl::geometric::VFRRT::VFRRT(const base::SpaceInformationPtr &si, VectorField vf, double exploration,
                              double initial_lambda, unsigned int update_freq)
  : RRT(si)
  , vf_(std::move(vf))
  , explorationSetting_(exploration)
  , lambda_(initial_lambda)
  , nth_step_(update_freq)
{
    setName("VFRRT");
    maxDistance_ = si->getStateValidityCheckingResolution();
}

ompl::geometric::VFRRT::~VFRRT() = default;

void ompl::geometric::VFRRT::clear()
{
    RRT::clear();
    efficientCount_ = 0;
    inefficientCount_ = 0;
    explorationInefficiency_ = 0.;
    step_ = 0;
}

void ompl::geometric::VFRRT::setup()
{
    RRT::setup();
    vfdim_ = si_->getStateSpace()->getValueLocations().size();
}

double ompl::geometric::VFRRT::determineMeanNorm()
{
    ompl::base::State *rstate = si_->allocState();
    double sum = 0.;
    for (unsigned int i = 0; i < magic::VFRRT_MEAN_NORM_SAMPLES; i++)
    {
        sampler_->sampleUniform(rstate);
        sum += vf_(rstate).norm();
    }
    si_->freeState(rstate);
    return sum / magic::VFRRT_MEAN_NORM_SAMPLES;
}

Eigen::VectorXd ompl::geometric::VFRRT::getNewDirection(const base::State *qnear, const base::State *qrand)
{
    // Set vrand to be the normalized vector from qnear to qrand
    Eigen::VectorXd vrand(vfdim_);
    for (unsigned int i = 0; i < vfdim_; i++)
        vrand[i] = *si_->getStateSpace()->getValueAddressAtIndex(qrand, i) -
                   *si_->getStateSpace()->getValueAddressAtIndex(qnear, i);
    vrand /= si_->distance(qnear, qrand);

    // Get the vector at qnear, and normalize
    Eigen::VectorXd vfield = vf_(qnear);
    const double lambdaScale = vfield.norm();
    // In the case where there is no vector field present, vfield.norm() == 0,
    // return the direction of the random state.
    if (lambdaScale < std::numeric_limits<float>::epsilon())
        return vrand;
    vfield /= lambdaScale;
    // Sample a weight from the distribution
    const double omega = biasedSampling(vrand, vfield, lambdaScale);
    // Determine updated direction
    return computeAlphaBeta(omega, vrand, vfield);
}

double ompl::geometric::VFRRT::biasedSampling(const Eigen::VectorXd &vrand, const Eigen::VectorXd &vfield,
                                              double lambdaScale)
{
    double sigma = .25 * (vrand - vfield).squaredNorm();
    updateGain();
    double scaledLambda = lambda_ * lambdaScale / meanNorm_;
    double phi = scaledLambda / (1. - std::exp(-2. * scaledLambda));
    double z = -std::log(1. - sigma * scaledLambda / phi) / scaledLambda;
    return std::sqrt(2. * z);
}

void ompl::geometric::VFRRT::updateGain()
{
    if (step_ == nth_step_)
    {
        lambda_ = lambda_ * (1 - explorationInefficiency_ + explorationSetting_);
        efficientCount_ = inefficientCount_ = 0;
        explorationInefficiency_ = 0;
        step_ = 0;
    }
    else
        step_++;
}

Eigen::VectorXd ompl::geometric::VFRRT::computeAlphaBeta(double omega, const Eigen::VectorXd &vrand,
                                                         const Eigen::VectorXd &vfield)
{
    double w2 = omega * omega;
    double c = vfield.dot(vrand);
    double cc_1 = c * c - 1.;
    double root = std::sqrt(cc_1 * w2 * (w2 - 4.));
    double beta = -root / (2. * cc_1);
    double sign = (beta < 0.) ? -1. : 1.;
    beta *= sign;
    double alpha = (sign * c * root + cc_1 * (2. - w2)) / (2. * cc_1);
    return alpha * vfield + beta * vrand;
}

ompl::geometric::VFRRT::Motion *ompl::geometric::VFRRT::extendTree(Motion *m, base::State *rstate,
                                                                   const Eigen::VectorXd &v)
{
    base::State *newState = si_->allocState();
    si_->copyState(newState, m->state);

    double d = si_->distance(m->state, rstate);
    if (d > maxDistance_)
        d = maxDistance_;

    const base::StateSpacePtr &space = si_->getStateSpace();
    for (unsigned int i = 0; i < vfdim_; i++)
        *space->getValueAddressAtIndex(newState, i) += d * v[i];
    if (!v.hasNaN() && si_->checkMotion(m->state, newState))
    {
        auto *motion = new Motion();
        motion->state = newState;
        motion->parent = m;
        updateExplorationEfficiency(motion);
        nn_->add(motion);
        return motion;
    }
    else
    {
        si_->freeState(newState);
        inefficientCount_++;
        return nullptr;
    }
}

void ompl::geometric::VFRRT::updateExplorationEfficiency(Motion *m)
{
    Motion *near = nn_->nearest(m);
    if (distanceFunction(m, near) < si_->getStateValidityCheckingResolution())
        inefficientCount_++;
    else
        efficientCount_++;
    explorationInefficiency_ = inefficientCount_ / (double)(efficientCount_ + inefficientCount_);
}

ompl::base::PlannerStatus ompl::geometric::VFRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    meanNorm_ = determineMeanNorm();

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (ptc == false)
    {
        // Sample random state (with goal biasing)
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // Find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        // Modify direction based on vector field before extending
        Motion *motion = extendTree(nmotion, rstate, getNewDirection(nmotion->state, rstate));
        if (!motion)
            continue;

        // Check if we can connect to the goal
        double dist = 0;
        bool sat = goal->isSatisfied(motion->state, &dist);
        if (sat)
        {
            approxdif = dist;
            solution = motion;
            break;
        }
        if (dist < approxdif)
        {
            approxdif = dist;
            approxsol = motion;
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        // Construct the solution path
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // Set the solution path
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, name_);
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}
