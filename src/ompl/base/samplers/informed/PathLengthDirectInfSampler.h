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

#ifndef OMPL_BASE_SAMPLERS_INFORMED_PATH_LENGTH_DIRECT_INFORMED_SAMPLER_
#define OMPL_BASE_SAMPLERS_INFORMED_PATH_LENGTH_DIRECT_INFORMED_SAMPLER_

// We inherit from InformedStateSampler
#include "ompl/base/samplers/InformedStateSampler.h"

// For std::list
#include <list>

namespace ompl
{
    namespace base
    {
        /**
            @anchor gPathLengthDirectInfSampler

            PathLengthDirectInfSampler is a method to generate uniform samples in the subset of a problem that could
           provide a shorter path from start to goal.
            This subset is a prolate hyperspheroid (PHS), a special type of an hyperellipsoid) and can be sampled
           directly.

            Informed sampling is a method to focus search which continuing to consider all homotopy classes that can
           provide a better solution.
            Directly sampling the informed subset guarantees a non-zero probability of improving a solution regardless
           of the size of the planning domain,
            the number of state dimensions, and how close the current solution is to the theoretical minimum.

            Currently only implemented for problems in R^n (i.e., RealVectorStateSpace), SE(2) (i.e., SE2StateSpace),
           and SE(3) (i.e., SE3StateSpace).
            Until an initial solution is found, this sampler simply passes-through to a uniform distribution over the
           entire state space.

            @par Associated publication:

            J. D. Gammell, T. D. Barfoot, S. S. Srinivasa, "Informed sampling for asymptotically optimal path planning."
            IEEE Transactions on Robotics (T-RO), 34(4): 966-984, Aug. 2018.
            DOI: <a href="https://doi.org/10.1109/TRO.2018.2830331">TRO.2018.2830331</a>.
            arXiv: <a href="https://arxiv.org/pdf/1706.06454">1706.06454 [cs.RO]</a>.
            <a href="https://www.youtube.com/watch?v=d7dX5MvDYTc">Illustration video</a>.
            <a href="https://www.youtube.com/watch?v=nsl-5MZfwu4">Short description video</a>.
        */

        /** \brief An informed sampler for problems seeking to minimize path length.*/
        class PathLengthDirectInfSampler : public InformedSampler
        {
        public:
            /** \brief Construct a sampler that only generates states with a heuristic solution estimate that is less
             * than the cost of the current solution using a direct ellipsoidal method. */
            PathLengthDirectInfSampler(const ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls);
            ~PathLengthDirectInfSampler() override;

            /** \brief Sample uniformly in the subset of the state space whose heuristic solution estimates are less
             * than the provided cost, i.e. in the interval [0, maxCost). Returns false if such a state was not found in
             * the specified number of iterations. */
            bool sampleUniform(State *statePtr, const Cost &maxCost) override;

            /** \brief Sample uniformly in the subset of the state space whose heuristic solution estimates are between
             * the provided costs, [minCost, maxCost). Returns false if such a state was not found in the specified
             * number of iterations. */
            bool sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost) override;

            /** \brief Whether the sampler can provide a measure of the informed subset */
            bool hasInformedMeasure() const override;

            /** \brief The measure of the subset of the state space defined by the current solution cost that is being
             * searched. Does not consider problem boundaries but returns the measure of the entire space if no solution
             * has been found. In the case of multiple goals, this measure assume each individual subset is independent,
             * therefore the resulting measure will be an overestimate if any of the subsets overlap. */
            double getInformedMeasure(const Cost &currentCost) const override;

            /** \brief A helper function to calculate the heuristic estimate of the solution cost for the informed
             * subset of a given state. */
            Cost heuristicSolnCost(const State *statePtr) const override;

        private:
            /** \brief A constant pointer to ProlateHyperspheroid */
            using ProlateHyperspheroidCPtr = std::shared_ptr<const ompl::ProlateHyperspheroid>;

            // Helper functions:
            // High level
            /** \brief Sample uniformly in the subset of the state space whose heuristic solution estimates are less
             * than the provided cost using a persistent iteration counter */
            bool sampleUniform(State *statePtr, const Cost &maxCost, unsigned int *iters);

            /** \brief Sample from the bounds of the problem and keep the sample if it passes the given test. Meant to
             * be used with isInAnyPhs and phsPtr->isInPhs() */
            bool sampleBoundsRejectPhs(State *statePtr, unsigned int *iters);

            /** \brief Sample from the given PHS and return true if the sample is within the boundaries of the problem
             * (i.e., it \e may be kept). */
            bool samplePhsRejectBounds(State *statePtr, unsigned int *iters);

            // Low level
            /** \brief Extract the informed subspace from a state pointer */
            std::vector<double> getInformedSubstate(const State *statePtr) const;

            /** \brief Create a full vector with any uninformed subspaces filled with a uniform random state. Expects
             * the state* to be allocated */
            void createFullState(State *statePtr, const std::vector<double> &informedVector);

            /** \brief Iterate through the list of PHSs and update each one to the new max cost as well as the sum of
             * their measures. Will remove any PHSs that cannot improve a better solution and in that case update the
             * number of goals. */
            void updatePhsDefinitions(const Cost &maxCost);

            /** \brief Select a random PHS from the list of PHSs. The probability of sampling chosing a PHS is it's
             * measure relative to the total measure of all PHSs. Bypasses if only one PHS exists. */
            ompl::ProlateHyperspheroidPtr randomPhsPtr();

            /** \brief Probabilistically decide whether to keep a given sample drawn directly from a PHS. If a sample is
             * in K PHSs, it returns true with probability 1/K. */
            bool keepSample(const std::vector<double> &informedVector);

            /** \brief Iterate through the list of PHSs and return true if the sample is in any of them */
            bool isInAnyPhs(const std::vector<double> &informedVector) const;

            /** \brief Return true if the sample is in the specified PHS. */
            bool isInPhs(const ProlateHyperspheroidCPtr &phsCPtr, const std::vector<double> &informedVector) const;

            /** \brief Iterate through the list of PHSs and return the number of PHSs that the sample is in */
            unsigned int numberOfPhsInclusions(const std::vector<double> &informedVector) const;

            // Variables
            /** \brief The prolate hyperspheroid description of the sub problems. One per goal state. */
            std::list<ompl::ProlateHyperspheroidPtr> listPhsPtrs_;

            /** \brief The summed measure of all the start-goal pairs */
            double summedMeasure_;

            /** \brief The index of the subspace of a compound StateSpace for which we can do informed sampling. Unused
             * if the StateSpace is not compound. */
            unsigned int informedIdx_;

            /** \brief The state space of the planning problem that is informed by the heuristics, i.e., in SE(2), R^2*/
            StateSpacePtr informedSubSpace_;

            /** \brief The index of the subspace of a compound StateSpace for which we cannot do informed sampling.
             * Unused if the StateSpace is not compound. */
            unsigned int uninformedIdx_;

            /** \brief The state space of the planning problem that is \e not informed by the heuristics, i.e., in
             * SE(2), SO(2)*/
            StateSpacePtr uninformedSubSpace_;

            /** \brief A regular sampler for the entire statespace for cases where informed sampling cannot be used or
             * is not helpful. I.e., Before a solution is found, or if the solution does not reduce the search space. */
            StateSamplerPtr baseSampler_;

            /** \brief A regular sampler to use on the uninformed subspace. */
            StateSamplerPtr uninformedSubSampler_;

            /** \brief An instance of a random number generator */
            RNG rng_;
        };  // PathLengthDirectInfSampler
    }
}

#endif  // OMPL_BASE_SAMPLERS_INFORMED_DIRECT_PATH_LENGTH_INFORMED_SAMPLER_
