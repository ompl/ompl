/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

/* Author: Beck Chen, Mark Moll */

#include "KoulesSetup.h"
#include "KoulesGoal.h"
#include "KoulesStateSpace.h"
#include "KoulesControlSpace.h"
#include "KoulesStatePropagator.h"
#include "KoulesDirectedControlSampler.h"
#include "KoulesDecomposition.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>


namespace ob = ompl::base;
namespace oc = ompl::control;

/// @cond IGNORE
class KoulesStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    KoulesStateValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }

    bool isValid(const ob::State *state) const override
    {
        return si_->satisfiesBounds(state);
    }
};
/// @endcond

ompl::control::DirectedControlSamplerPtr KoulesDirectedControlSamplerAllocator(
    const ompl::control::SpaceInformation *si, const ompl::base::GoalPtr &goal, bool propagateMax)
{
    return std::make_shared<KoulesDirectedControlSampler>(si, goal, propagateMax);
}


KoulesSetup::KoulesSetup(unsigned int numKoules, const std::string& plannerName,
    const std::vector<double>& stateVec)
    : ompl::control::SimpleSetup(std::make_shared<KoulesControlSpace>(numKoules))
{
    initialize(numKoules, plannerName, stateVec);
}

KoulesSetup::KoulesSetup(unsigned int numKoules, const std::string& plannerName, double kouleVel)
    : ompl::control::SimpleSetup(std::make_shared<KoulesControlSpace>(numKoules))
{
    initialize(numKoules, plannerName);
    double* state = getProblemDefinition()->getStartState(0)->as<KoulesStateSpace::StateType>()->values;
    double theta;
    ompl::RNG rng;
    for (unsigned int i = 0; i < numKoules; ++i)
    {
        theta = rng.uniformReal(0., 2. * boost::math::constants::pi<double>());
        state[4 * i + 7] = kouleVel * cos(theta);
        state[4 * i + 8] = kouleVel * sin(theta);
    }
}

void KoulesSetup::initialize(unsigned int numKoules, const std::string& plannerName,
    const std::vector<double>& stateVec)
{
    const ob::StateSpacePtr& space = getStateSpace();
    space->setup();
    // setup start state
    ob::ScopedState<> start(space);
    if (stateVec.size() == space->getDimension())
        space->copyFromReals(start.get(), stateVec);
    else
    {
        // Pick koule positions evenly radially distributed, but at a linearly
        // increasing distance from the center. The ship's initial position is
        // at the center. Initial velocities are 0.
        std::vector<double> startVec(space->getDimension(), 0.);
        double r, theta = boost::math::constants::pi<double>(), delta = 2.*theta / numKoules;
        startVec[0] = .5 * sideLength;
        startVec[1] = .5 * sideLength;
        startVec[4] = .5 * delta;
        for (unsigned int i = 0; i < numKoules; ++i, theta += delta)
        {
            r = .1 + i * .1 / numKoules;
            startVec[4 * i + 5] = .5 * sideLength + r * cos(theta);
            startVec[4 * i + 6] = .5 * sideLength + r * sin(theta);
        }
        space->copyFromReals(start.get(), startVec);
    }
    setStartState(start);
    // set goal
    setGoal(std::make_shared<KoulesGoal>(si_));
    // set propagation step size
    si_->setPropagationStepSize(propagationStepSize);
    // set min/max propagation steps
    si_->setMinMaxControlDuration(propagationMinSteps, propagationMaxSteps);
    // set directed control sampler; when using the PDST planner, propagate as long as possible
    bool isPDST = plannerName == "pdst";
    const ompl::base::GoalPtr& goal = getGoal();
    si_->setDirectedControlSamplerAllocator(
        [&goal, isPDST](const ompl::control::SpaceInformation *si)
        {
            return KoulesDirectedControlSamplerAllocator(si,  goal, isPDST);
        });
    // set planner
    setPlanner(getConfiguredPlannerInstance(plannerName));
    // set validity checker
    setStateValidityChecker(std::make_shared<KoulesStateValidityChecker>(si_));
    // set state propagator
    setStatePropagator(std::make_shared<KoulesStatePropagator>(si_));
}

ob::PlannerPtr KoulesSetup::getConfiguredPlannerInstance(const std::string& plannerName)
{
    if (plannerName == "rrt")
    {
        auto rrtplanner(std::make_shared<oc::RRT>(si_));
        rrtplanner->setIntermediateStates(true);
        return rrtplanner;
    }
    if (plannerName == "est")
        return std::make_shared<oc::EST>(si_);
    if (plannerName == "kpiece")
        return std::make_shared<oc::KPIECE1>(si_);
    if (plannerName == "sst")
    {
        auto sstplanner(std::make_shared<oc::SST>(si_));
        sstplanner->setSelectionRadius(0.05);
        sstplanner->setPruningRadius(0.001);
        return sstplanner;
    }
    if (plannerName == "sycloprrt")
        return std::make_shared<oc::SyclopRRT>(si_, std::make_shared<KoulesDecomposition>(getStateSpace()));
    if (plannerName == "syclopest")
        return std::make_shared<oc::SyclopEST>(si_, std::make_shared<KoulesDecomposition>(getStateSpace()));

    auto pdstplanner(std::make_shared<oc::PDST>(si_));
    pdstplanner->setProjectionEvaluator(
        si_->getStateSpace()->getProjection("PDSTProjection"));
    return pdstplanner;
}
