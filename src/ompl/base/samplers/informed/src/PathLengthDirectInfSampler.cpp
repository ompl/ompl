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

#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#include "ompl/util/Exception.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

// For boost::make_shared
#include "boost/make_shared.hpp"

namespace ompl
{
    namespace base
    {
        // The direct ellipsoid sampling class for path-length:
        PathLengthDirectInfSampler::PathLengthDirectInfSampler(const StateSpace* space, const ProblemDefinitionPtr probDefn, const Cost* bestCost)
          : InformedStateSampler(space, probDefn, bestCost),
            informedIdx_(0u),
            uninformedIdx_(0u)
        {
            // Variables
            // The foci of the ellipse as State* s
            State* startFocusState;
            State* goalFocusState;
            // The foci of the ellipse as std::vectors
            std::vector<double> startFocusVector;
            std::vector<double> goalFocusVector;

            // Sanity check the problem.
            if (probDefn_->getStartStateCount() != 1u)
            {
                throw Exception("The direct path-length informed sampler currently only supports 1 start state.");
            }

            if (probDefn_->getGoal()->hasType(GOAL_STATE) == false)
            {
                throw Exception("The direct path-length informed sampler currently only supports goals that can be cast to goal states.");
            }

            // Check that the provided statespace is compatible and extract the necessary indices.
            // The statespace must either be R^n or SE(2) or SE(3)
            if (StateSampler::space_->isCompound() == false)
            {
                if (StateSampler::space_->getType() == STATE_SPACE_REAL_VECTOR)
                {
                    informedIdx_ = 0u;
                    uninformedIdx_ = 0u;
                }
                else
                {
                    throw Exception("PathLengthDirectInfSampler only supports RealVector, SE2 and SE3 StateSpaces.");
                }
            }
            else if (StateSampler::space_->isCompound() == true)
            {
                // Check that it is SE2 or SE3
                if (StateSampler::space_->getType() == STATE_SPACE_SE2 || StateSampler::space_->getType() == STATE_SPACE_SE3)
                {
                    // Variable:
                    // An ease of use upcasted pointer to the space as a compound space
                    const CompoundStateSpace* compoundSpace;

                    // Get the space as a compound space
                    compoundSpace = StateSampler::space_->as<CompoundStateSpace>();

                    // Sanity check
                    if (compoundSpace->getSubspaceCount() != 2u)
                    {
                        // Pout
                        throw Exception("The provided compound StateSpace is SE(2) or SE(3) but does not have exactly 2 subspaces.");
                    }

                    // Iterate over the state spaces, finding the real vector and SO components.
                    for (unsigned int idx = 0u; idx < StateSampler::space_->as<CompoundStateSpace>()->getSubspaceCount(); ++idx)
                    {
                        // Check if the space is real-vectored, SO2 or SO3
                        if (compoundSpace->getSubspace(idx)->getType() == STATE_SPACE_REAL_VECTOR)
                        {
                            informedIdx_ = idx;
                        }
                        else if (compoundSpace->getSubspace(idx)->getType() == STATE_SPACE_SO2)
                        {
                            uninformedIdx_ = idx;
                        }
                        else if (compoundSpace->getSubspace(idx)->getType() == STATE_SPACE_SO3)
                        {
                            uninformedIdx_ = idx;
                        }
                        else
                        {
                            // Pout
                            throw Exception("The provided compound StateSpace is SE(2) or SE(3) but contains a subspace that is not R^2, R^3, SO(2), or SO(3).");
                        }
                    }
                }
                else
                {
                    throw Exception("PathLengthDirectInfSampler only supports RealVector, SE2 and SE3 statespaces.");
                }
            }


            // Create a sampler for the whole space that we can use if we have no information
            baseSampler_ = StateSampler::space_->allocDefaultStateSampler();

            // Check if the space is compound
            if (StateSampler::space_->isCompound() == false)
            {
                // It is not.

                // Store the foci
                startFocusState = probDefn_->getStartState(0u);
                goalFocusState = probDefn_->getGoal()->as<GoalState>()->getState();

                // The informed subspace is the full space
                informedSubSpace_ = StateSampler::space_;

                // And the uniformed subspace and its associated sampler are null
                uninformedSubSpace_ = NULL;
                uninformedSubSampler_ = StateSamplerPtr();
            }
            else
            {
                // Store the foci
                startFocusState = probDefn_->getStartState(0u)->as<CompoundState>()->components[informedIdx_];
                goalFocusState = probDefn_->getGoal()->as<GoalState>()->getState()->as<CompoundState>()->components[informedIdx_];

                // The informed subset is the real vector space. StateSampler::space_ is a raw pointer, so for this variable to be able to hold all of space_, we need to store the raw pointer to the subspace...
                informedSubSpace_ = StateSampler::space_->as<CompoundStateSpace>()->getSubspace(informedIdx_).get();

                // And the uninformed subspace is the remainder. Raw pointer for consistency with the above.
                uninformedSubSpace_ = StateSampler::space_->as<CompoundStateSpace>()->getSubspace(uninformedIdx_).get();

                // Create a sampler for the uniformed subset:
                uninformedSubSampler_ = uninformedSubSpace_->allocDefaultStateSampler();
            }

            // Now extract the foci of the ellipse
            informedSubSpace_->copyToReals(startFocusVector, startFocusState);
            informedSubSpace_->copyToReals(goalFocusVector, goalFocusState);

            // Create the definition of the PHS
            phsPtr_ = boost::make_shared<ProlateHyperspheroid>(informedSubSpace_->getDimension(), &startFocusVector[0], &goalFocusVector[0]);
        }

        PathLengthDirectInfSampler::~PathLengthDirectInfSampler()
        {
            // dtor
        }


        void PathLengthDirectInfSampler::sampleUniform(State* statePtr, const Cost& maxCost)
        {
            // Check if a solution path has been found
            if (std::isfinite(maxCost.value()) == false)
            {
                // We don't have a solution yet, we sample from our basic sampler instead...
                baseSampler_->sampleUniform(statePtr);
            }
            else // We have a solution
            {
                // Set the new transverse diameter
                phsPtr_->setTransverseDiameter(maxCost.value());

                // Check whether the problem domain (i.e., StateSpace) or PHS has the smaller measure. Sample the smaller directly and reject from the larger.
                if (informedSubSpace_->getMeasure() <= phsPtr_->getPhsMeasure())
                {
                    // The PHS is larger than the subspace, just sample from the subspace directly.
                    // Variables
                    // The informed subset of the sample as a vector
                    std::vector<double> informedVector(informedSubSpace_->getDimension());

                    // Sample from the state space until the sample is in the PHS
                    do
                    {
                        // Generate a random sample
                        baseSampler_->sampleUniform(statePtr);

                        // Is there an extra "uninformed" subspace to trim off before comparing to the PHS?
                        if (StateSampler::space_->isCompound() == false)
                        {
                            // No, space_ == informedSubSpace_
                            informedSubSpace_->copyToReals(informedVector, statePtr);
                        }
                        else
                        {
                            // Yes, we need to do some work to extract the subspace
                            informedSubSpace_->copyToReals(informedVector, statePtr->as<CompoundState>()->components[informedIdx_]);
                        }
                    }
                    // Check if the informed state is in the PHS
                    while (phsPtr_->isInPhs(informedSubSpace_->getDimension(), &informedVector[0]) == false);
                }
                else
                {
                    // The PHS has a smaller volume than the subspace.
                    // Sample from within the PHS until the sample is in the state space
                    do
                    {
                        this->sampleUniformIgnoreBounds(statePtr, maxCost);
                    }
                    while (StateSampler::space_->satisfiesBounds(statePtr) == false);
                }
            }
        }

        void PathLengthDirectInfSampler::sampleUniform(State* statePtr, const Cost& minCost, const Cost& maxCost)
        {
            // Sample from the larger PHS until the sample does not lie within the smaller PHS.
            // Since volume in a sphere/spheroid is proportionately concentrated near the surface, this isn't horribly inefficient, though a direct method would be better
            do
            {
                this->sampleUniform(statePtr, maxCost);
            }
            while (InformedStateSampler::opt_->isCostBetterThan(this->heuristicSolnCost(statePtr), minCost));
        }



        bool PathLengthDirectInfSampler::hasInformedMeasure() const
        {
            return true;
        }



        double PathLengthDirectInfSampler::getInformedMeasure(const Cost& currentCost) const
        {
            // Variable
            // The measure of the informed set
            double informedMeasure;

            // The informed measure is then the measure of the PHS for the given cost:
            informedMeasure = phsPtr_->getPhsMeasure(currentCost.value());

            // And if the space is compound, further multiplied by the measure of the uniformed subspace
            if (StateSampler::space_->isCompound() == true)
            {
                informedMeasure = informedMeasure * uninformedSubSpace_->getMeasure();
            }

            // Return the smaller of the two measures
            return std::min(StateSampler::space_->getMeasure(), informedMeasure);
        }

        void PathLengthDirectInfSampler::sampleUniformIgnoreBounds(State* statePtr, const Cost& maxCost)
        {
            // Variable
            // The informed subset of the sample as a vector
            std::vector<double> informedVector(informedSubSpace_->getDimension());

            // Set the new transverse diameter
            phsPtr_->setTransverseDiameter(maxCost.value());

            // Sample the ellipse
            rng_.uniformProlateHyperspheroid(phsPtr_, informedSubSpace_->getDimension(), &informedVector[0]);

            // If there is an extra "uninformed" subspace, we need to add that to the state before converting the raw vector representation into a state....
            if (StateSampler::space_->isCompound() == false)
            {
                // No, space_ == informedSubSpace_
                // Copy into the state pointer
                informedSubSpace_->copyFromReals(statePtr, informedVector);
            }
            else
            {
                // Yes, we need to also sample the uninformed subspace
                // Variables
                // A state for the uninformed subspace
                State* uninformedState = uninformedSubSpace_->allocState();

                // Copy the informed subspace into the state pointer
                informedSubSpace_->copyFromReals(statePtr->as<CompoundState>()->components[informedIdx_], informedVector);

                // Sample the uniformed subspace
                uninformedSubSampler_->sampleUniform(uninformedState);

                // Copy the informed subspace into the state pointer
                uninformedSubSpace_->copyState(statePtr->as<CompoundState>()->components[uninformedIdx_], uninformedState);

                // Free the state
                uninformedSubSpace_->freeState(uninformedState);
            }
        }

        void PathLengthDirectInfSampler::sampleUniformIgnoreBounds(State* statePtr, const Cost& minCost, const Cost& maxCost)
        {
            // Sample from the larger PHS until the sample does not lie within the smaller PHS.
            // Since volume in a sphere/spheroid is proportionately concentrated near the surface, this isn't horribly inefficient, though a direct method would be better
            do
            {
                this->sampleUniformIgnoreBounds(statePtr, maxCost);
            }
            while (InformedStateSampler::opt_->isCostBetterThan(this->heuristicSolnCost(statePtr), minCost));
        }


        Cost PathLengthDirectInfSampler::heuristicSolnCost(const State* statePtr) const
        {
            // Variable
            // The raw data in the state
            std::vector<double> rawData(informedSubSpace_->getDimension());

            // Get the raw data
            if (StateSampler::space_->isCompound() == false)
            {
                informedSubSpace_->copyToReals(rawData, statePtr);
            }
            else
            {
                informedSubSpace_->copyToReals(rawData, statePtr->as<CompoundState>()->components[informedIdx_]);
            }

            // Calculate and return the length
            return Cost(phsPtr_->getPathLength(informedSubSpace_->getDimension(), &rawData[0]));
        }

    }; // base
};  // ompl
