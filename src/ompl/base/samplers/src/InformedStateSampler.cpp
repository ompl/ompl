/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
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
*   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Jonathan Gammell */

#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/util/Exception.h"
#include "ompl/base/OptimizationObjective.h"
// The goal definitions
#include "ompl/base/Goal.h"

namespace ompl
{
    namespace base
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // InformedSampler
        InformedSampler::InformedSampler(const ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls)
          : probDefn_(probDefn), space_(probDefn->getSpaceInformation()->getStateSpace()), numIters_(maxNumberCalls)
        {
            // Sanity check the problem.
            // Check that there is an optimization objective
            if (!probDefn_->hasOptimizationObjective())
            {
                throw Exception("InformedSampler: An optimization objective must be specified at construction.");
            }
            // No else

            // Make sure we have at least one start and warn if we have more than one
            if (probDefn_->getStartStateCount() == 0u)
            {
                throw Exception("InformedSampler: At least one start state must be specified at construction.");
            }
            // No else

            // Store the optimization objective for later ease.
            opt_ = probDefn_->getOptimizationObjective();
        }

        double InformedSampler::getInformedMeasure(const Cost &minCost, const Cost &maxCost) const
        {
            // Subtract the measures defined by the max and min costs. These will be defined in the deriving class.
            return getInformedMeasure(maxCost) - getInformedMeasure(minCost);
        }

        Cost InformedSampler::heuristicSolnCost(const State *statePtr) const
        {
            // Return the best heuristic estimate of the cost-to-come and cost-to-go from the state considering all
            // starts.

            // If there's only one start, be simple:
            if (probDefn_->getStartStateCount() == 1u)
            {
                // Calculate and from the one and only start
                return opt_->combineCosts(opt_->motionCostHeuristic(probDefn_->getStartState(0u), statePtr),
                                          opt_->costToGo(statePtr, probDefn_->getGoal().get()));
            }


            // Calculate and return the best

            // Variable
            // The best cost so far
            Cost bestCost = opt_->infiniteCost();

            // Iterate over each start and store the best
            for (unsigned int i = 0u; i < probDefn_->getStartStateCount(); ++i)
            {
                // Store the best
                bestCost = opt_->betterCost(
                    bestCost, opt_->combineCosts(opt_->motionCostHeuristic(probDefn_->getStartState(i), statePtr),
                                                 opt_->costToGo(statePtr, probDefn_->getGoal().get())));
            }

            // Return the best
            return bestCost;
        }

        ProblemDefinitionPtr InformedSampler::getProblemDefn() const
        {
            return probDefn_;
        }

        unsigned int InformedSampler::getMaxNumberOfIters() const
        {
            return numIters_;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // InformedStateSampler
        InformedStateSampler::InformedStateSampler(const ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls,
                                                   const GetCurrentCostFunc &costFunc)
          : StateSampler(probDefn->getSpaceInformation()->getStateSpace().get())
        {
            // Call the common constructor with the default informed sampler
            commonConstructor(
                costFunc, probDefn->getOptimizationObjective()->allocInformedStateSampler(probDefn, maxNumberCalls));
        }

        InformedStateSampler::InformedStateSampler(const ProblemDefinitionPtr &probDefn,
                                                   const GetCurrentCostFunc &costFunc,
                                                   const InformedSamplerPtr &infSampler)
          : StateSampler(probDefn->getSpaceInformation()->getStateSpace().get())
        {
            // Call the common constructor with the given informed sampler
            commonConstructor(costFunc, infSampler);
        }

        void InformedStateSampler::commonConstructor(const GetCurrentCostFunc &costFunc,
                                                     const InformedSamplerPtr &infSampler)
        {
            // Store the cost function
            bestCostFunc_ = costFunc;

            // Store the informed sampler
            infSampler_ = infSampler;

            // Allocate a base sampler
            baseSampler_ = StateSampler::space_->allocDefaultStateSampler();
        }

        void InformedStateSampler::sampleUniform(State *statePtr)
        {
            // Variable
            // Whether informed sampling was successful
            bool informedSuccess;

            // Call sample uniform with the current best cost, check returning function:
            informedSuccess = infSampler_->sampleUniform(statePtr, bestCostFunc_());

            // If we were unsuccessful, return a regular sample
            if (!informedSuccess)
            {
                baseSampler_->sampleUniform(statePtr);
            }
            // No else.
        }

        void InformedStateSampler::sampleUniformNear(State *statePtr, const State *near, const double distance)
        {
            // Warn:
            OMPL_WARN("sampleUniformNear is not informed.");
            return baseSampler_->sampleUniformNear(statePtr, near, distance);
        }

        void InformedStateSampler::sampleGaussian(State *statePtr, const State *mean, const double stdDev)
        {
            // Warn:
            OMPL_WARN("sampleGaussian is not informed.");
            return baseSampler_->sampleGaussian(statePtr, mean, stdDev);
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    };  // base
};      // ompl
