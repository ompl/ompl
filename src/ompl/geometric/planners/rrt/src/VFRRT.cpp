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

#include "ompl/geometric/planners/rrt/VFRRT.h"

ompl::geometric::VFRRT::VFRRT (const base::SpaceInformationPtr &si, const VectorField &vf,
                               double exploration, double initial_lambda, unsigned int update_freq)
    : RRT(si), vf_(vf), explorationSetting(exploration), lambda(initial_lambda),
      nth_step(update_freq), meanNorm_(0)
{
    setName("VFRRT");
    efficientCount = 0;
    inefficientCount = 0;
    explorationInefficiency = 0;
    step = 0;
    maxDistance_ = si->getStateValidityCheckingResolution();
}
            
ompl::geometric::VFRRT::~VFRRT () {}
            
void ompl::geometric::VFRRT::clear ()
{
    efficientCount = 0;
    inefficientCount = 0;
    explorationInefficiency = 0;
    step = 0;
    RRT::clear();
}
            
double ompl::geometric::VFRRT::determineMeanNorm()
{
    ompl::base::State *rstate = si_->allocState();
    double norm = 0;
    for (int i = 0; i < 1000; i++)
    {
        sampler_->sampleUniform(rstate);
        norm += vf_(rstate).norm()/1000;
    }
    si_->freeState(rstate);
    return norm;
}
            
Eigen::VectorXd ompl::geometric::VFRRT::getNewDirection (const base::State *qnear, const base::State *qrand)
{
    // Set vrand to be the normalized vector from qnear to qrand
    base::ScopedState<> q1(si_), q2(si_);
    q1 = qnear;
    q2 = qrand;
                
    const unsigned int d = q1.reals().size();
    Eigen::VectorXd vrand(d);
    for (unsigned int i = 0; i < d; i++)
    {
        vrand[i] = q2[i] - q1[i];
    }
    vrand /= si_->distance(qnear, qrand);
                
    // Get the vector at qnear, and normalize
    Eigen::VectorXd vfield = vf_(qnear);
    const double lambdaScale = vfield.norm();
    // In the case where there is no vector field present, vfield.norm() == 0, return the direction of the random state.
    if (lambdaScale < 0.0001)
        return vrand;

    vfield /= lambdaScale;
                
    // Sample a weight from the distribution
    const double omega = biasedSampling(vrand, vfield, lambdaScale);
                
    // Determine updated direction
    return computeAlphaBeta(omega, vrand, vfield);
}
            
double ompl::geometric::VFRRT::biasedSampling (const Eigen::VectorXd &vrand, const Eigen::VectorXd &vfield, double lambdaScale)
{
    double sigma = (vrand - vfield).squaredNorm() / 4.0;
    updateGain();
    double scaledLambda = lambda*lambdaScale/meanNorm_;
    double phi = scaledLambda / (1 - std::exp(-2*scaledLambda));
    double z = - std::log(1 - sigma * scaledLambda / phi) / scaledLambda;
    return std::sqrt(2*z);
}
            
void ompl::geometric::VFRRT::updateGain ()
{
    if (step == nth_step)
    {
        lambda = lambda * (1 - explorationInefficiency + explorationSetting);
        efficientCount = inefficientCount = 0;
        explorationInefficiency = 0;
        step = -1;
    }
    step++;
}
            
Eigen::VectorXd ompl::geometric::VFRRT::computeAlphaBeta (double omega, const Eigen::VectorXd &vrand,
                                                          const Eigen::VectorXd &vfield)
{
    double w2 = omega*omega;
    double c = vfield.dot(vrand);
    double cc_1 = c*c - 1;
    double root = std::sqrt(cc_1 * w2 * (w2-4));
    double beta = -root / (2*cc_1);
    double sign = (beta < 0) ? -1 : 1;
    beta *= sign;
    double alpha = (sign * c * root + (cc_1)*(2-w2)) / (2*cc_1);
    return alpha*vfield + beta*vrand;
}
            
ompl::geometric::VFRRT::Motion *ompl::geometric::VFRRT::extendTree (Motion *m, base::State* rstate, const Eigen::VectorXd &v)
{
    base::ScopedState<> newState(si_);
    si_->copyState(newState.get(), m->state);
    
    double d = si_->distance(m->state, rstate);
    if (d > maxDistance_)
        d = maxDistance_;
    const unsigned int dim = newState.reals().size();

    for (unsigned int i = 0; i < dim; i++)
    {
        newState[i] += d * v[i];
    }
    if (!v.hasNaN() && si_->checkMotion(m->state, newState.get()))
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, newState.get());
        motion->parent = m;
        updateExplorationEfficiency(motion);
        nn_->add(motion);
        return motion;
    }
    else
    {
        inefficientCount++;
        return NULL;
    }
}
            
void ompl::geometric::VFRRT::updateExplorationEfficiency(Motion *m)
{
    Motion *near = nn_->nearest(m);
    if (nn_->getDistanceFunction()(m, near) < si_->getStateValidityCheckingResolution())
        inefficientCount++;
    else
        efficientCount++;
    explorationInefficiency = inefficientCount/(double)(efficientCount + inefficientCount);
}

ompl::base::PlannerStatus ompl::geometric::VFRRT::solve (const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
                
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
                
    meanNorm_ = determineMeanNorm();
                
    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }
                
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
                
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());
                
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
                
    while (ptc == false)
    {
        // Sample random state
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
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }
                
    if (solution != NULL)
    {
        lastGoalMotion_ = solution;
                    
        // Construct the solution path
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }
                    
        // Set the solution path
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
        solved = true;
    }
                
    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;
                
    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
                
    return base::PlannerStatus(solved, approximate);
}
