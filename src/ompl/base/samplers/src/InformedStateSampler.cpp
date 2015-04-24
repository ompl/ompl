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
//The goal definitions
#include "ompl/base/Goal.h"

namespace ompl
{
    namespace base
    {
        //The base InformedStateSampler class:
        InformedStateSampler::InformedStateSampler(const StateSpace* space, const ProblemDefinitionPtr probDefn, const Cost* bestCost)
          : StateSampler(space),
            probDefn_(probDefn),
            bestCostPtr_(bestCost)
        {
            //Sanity check the problem.
            //Check that we were given a valid pointer
            if (bestCostPtr_ == false)
            {
                throw Exception ("InformedStateSampler: The cost pointer must be valid at construction.");
            }
            //No else

            //Check that there is an optimization objective
            if (probDefn_->hasOptimizationObjective() == false)
            {
                throw Exception ("InformedStateSampler: An optimization objective must be specified at construction.");
            }
            //No else

            //Make sure we have at least one start and warn if we have more than one
            if (probDefn_->getStartStateCount() == 0u)
            {
                throw Exception ("InformedStateSampler: At least one start state must be specified at construction.");
            }
            else if (probDefn_->getStartStateCount() > 1u)
            {
                OMPL_WARN("InformedStateSampler: More than 1 start state present. Informed samplers will only use the first.");
            }
            //No else

            //Store the optimization objective for later ease.
            opt_ = probDefn_->getOptimizationObjective();
        }

        void InformedStateSampler::sampleUniform(State* statePtr)
        {
            //Call sample uniform with the current best cost, this function will be defined in the deriving class
            this->sampleUniform(statePtr, *bestCostPtr_);
        }

        double InformedStateSampler::getInformedMeasure() const
        {
            //Get the informed measure for the current best solution
            return this->getInformedMeasure(*bestCostPtr_);
        }

        double InformedStateSampler::getInformedMeasure(const Cost& minCost, const Cost& maxCost) const
        {
            //Subtract the measures defined by the max and min costs. These will be defined in the deriving class.
            return this->getInformedMeasure(maxCost) - this->getInformedMeasure(minCost);
        }

        void InformedStateSampler::sampleUniformNear(State* statePtr, const State* near, const double distance)
        {
            throw Exception ("%s: No near-state informed sampling method is defined.", opt_->getDescription().c_str());
        }

        void InformedStateSampler::sampleGaussian(State* statePtr, const State* mean, const double stdDev)
        {
            throw Exception ("%s: No Gaussian informed sampling method is defined.", opt_->getDescription().c_str());
        }

        Cost InformedStateSampler::heuristicSolnCost(const State* statePtr) const
        {
            //Combine heuristic estimates of the cost-to-come and cost-to-go from the state.
            return opt_->combineCosts( opt_->motionCostHeuristic(probDefn_->getStartState(0u), statePtr), opt_->costToGo(statePtr, probDefn_->getGoal().get()) );
        }
    }; //base
};  //ompl
