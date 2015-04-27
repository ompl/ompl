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

#ifndef OMPL_BASE_SAMPLERS_INFORMED_SAMPLER_
#define OMPL_BASE_SAMPLERS_INFORMED_SAMPLER_

// We inherit from StateSampler
#include "ompl/base/StateSampler.h"
// Deriving functions must be able to sample within a given cost
#include "ompl/base/Cost.h"
// We use a pointer to the problem definition to access problem and solution data.
#include "ompl/base/ProblemDefinition.h"

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(InformedStateSampler);

        /** \brief An abstract class for state samplers that use information
        about the current solution to limit future search to a planning
        subproblem that contains all possibly better solutions. */
        class InformedStateSampler : public StateSampler
        {
        public:
            /** \brief Construct a sampler that only generates states with a heuristic solution estimate that is less than the cost of the current solution. */
            InformedStateSampler(const StateSpace* space, const ProblemDefinitionPtr probDefn, const Cost* bestCost);
            virtual ~InformedStateSampler(void)
            {
            }

            /** \brief Sample uniformly in the subset of the state space whose heuristic solution estimates are less than the current best cost (as defined by the pointer passed at construction). By default just calls sampleUniform(State*, Cost) with cost given by the member variable. */
            virtual void sampleUniform(State* statePtr);

            /** \brief Sample uniformly in the subset of the state space whose heuristic solution estimates are less than the provided cost. */
            virtual void sampleUniform(State* statePtr, const Cost& maxCost) = 0;

            /** \brief Sample uniformly in the subset of the state space whose heuristic solution estimates are between the provided costs. */
            virtual void sampleUniform(State* statePtr, const Cost& minCost, const Cost& maxCost) = 0;

            /** \brief Whether the sampler can provide a measure of the informed subset */
            virtual bool hasInformedMeasure() const = 0;

            /** \brief The measure of the subset of the state space defined by the current solution cost that is being searched. Does not consider problem boundaries but returns the measure of the entire space if no solution has been found or if a closed form expression for the measure does not exist. By default calls the 1-argument overloaded version with the current best cost; however, there may be more efficient ways to do this for some cost functions. */
            virtual double getInformedMeasure() const;

            /** \brief The measure of the subset of the state space defined by the current solution cost that is being searched. Does not consider problem boundaries but returns the measure of the entire space if no solution has been found or if a closed form expression for the measure does not exist. */
            virtual double getInformedMeasure(const Cost& currentCost) const = 0;

            /** \brief The measure of the subset of the state space defined by the current solution cost that is being searched. Does not consider problem boundaries but returns the measure of the entire space if no solution has been found or if a closed form expression for the measure does not exist.  By default calls the 1-argument overloaded version with the min and max costs and subtracts the differences; however, there may be more efficient ways to do this for some cost functions. */
            virtual double getInformedMeasure(const Cost& minCost, const Cost& maxCost) const;

            /** \brief By default sampleUniformNear throws. This can be overloaded by a specific informed sampler if desired. */
            virtual void sampleUniformNear(State* statePtr, const State* near, const double distance);

            /** \brief By default sampleGaussian throws. This can be overloaded by a specific informed sampler if desired. */
            virtual void sampleGaussian(State* statePtr, const State* mean, const double stdDev);

            /** \brief A helper function to calculate the heuristic estimate of the solution cost for a given state using the optimization objective stored in the problem definition. */
            virtual Cost heuristicSolnCost(const State* statePtr) const;

        protected:
            /** A copy of the problem definition */
            ProblemDefinitionPtr probDefn_;
            /** A copy of the optimization objective */
            OptimizationObjectivePtr opt_;
            /** A pointer to the best cost found so far, the mechanism through which the sampler is "informed". */
            const Cost* bestCostPtr_;

        private:
        };
    }
}


#endif // OMPL_BASE_SAMPLERS_INFORMED_SAMPLER_
