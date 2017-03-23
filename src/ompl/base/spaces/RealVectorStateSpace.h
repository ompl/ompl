/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_SPACES_REAL_VECTOR_STATE_SPACE_
#define OMPL_BASE_SPACES_REAL_VECTOR_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <vector>
#include <string>
#include <map>

namespace ompl
{
    namespace base
    {
        /** \brief State sampler for the R<sup>n</sup> state space */
        class RealVectorStateSampler : public StateSampler
        {
        public:
            /** \brief Constructor */
            RealVectorStateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            void sampleUniform(State *state) override;
            /** \brief Sample a state such that each component state[i] is
                uniformly sampled from [near[i]-distance, near[i]+distance].
                If this interval exceeds the state space bounds, the
                interval is truncated. */
            void sampleUniformNear(State *state, const State *near, double distance) override;
            /** \brief Sample a state such that each component state[i] has
                a Gaussian distribution with mean mean[i] and standard
                deviation stdDev. If the sampled value exceeds the state
                space boundary, it is thresholded to the nearest boundary. */
            void sampleGaussian(State *state, const State *mean, double stdDev) override;
        };

        /** \brief A state space representing R<sup>n</sup>. The distance function is the L2 norm. */
        class RealVectorStateSpace : public StateSpace
        {
        public:
            /** \brief The definition of a state in R<sup>n</sup> */
            class StateType : public State
            {
            public:
                StateType() = default;

                /** \brief Access element i of values.  This does not
                    check whether the index is within bounds */
                double operator[](unsigned int i) const
                {
                    return values[i];
                }

                /** \brief Access element i of values.  This does not
                    check whether the index is within bounds */
                double &operator[](unsigned int i)
                {
                    return values[i];
                }

                /** \brief The value of the actual vector in R<sup>n</sup> */
                double *values;
            };

            /** \brief Constructor. The dimension of of the space needs to be specified. A space representing
                R<sup>dim</sup> will be instantiated */
            RealVectorStateSpace(unsigned int dim = 0)
              : dimension_(dim), bounds_(dim), stateBytes_(dim * sizeof(double))
            {
                type_ = STATE_SPACE_REAL_VECTOR;
                setName("RealVector" + getName());
                dimensionNames_.resize(dim, "");
            }

            ~RealVectorStateSpace() override = default;

            /** \brief Increase the dimensionality of the state space by 1. Optionally, bounds can be specified for this
             * added dimension. setup() will need to be called after adding dimensions. */
            void addDimension(double minBound = 0.0, double maxBound = 0.0);

            /** \brief Increase the dimensionality of the state space by 1 and specify the name of this dimension.
             * Optionally, bounds can be specified for this added dimension. setup() will need to be called after adding
             * dimensions. This function is a wrapper for the previous definition of addDimension(). */
            void addDimension(const std::string &name, double minBound = 0.0, double maxBound = 0.0);

            /** \brief Set the bounds of this state space. This defines
                the range of the space in which sampling is performed. */
            void setBounds(const RealVectorBounds &bounds);

            /** \brief Set the bounds of this state space. The bounds for
                each dimension will be the same: [\e low, \e high]. */
            void setBounds(double low, double high);

            /** \brief Get the bounds for this state space */
            const RealVectorBounds &getBounds() const
            {
                return bounds_;
            }

            unsigned int getDimension() const override;

            /** \brief Each dimension can optionally have a name associated to it. If it does, this function returns
               that name.
                Return empty string otherwise */
            const std::string &getDimensionName(unsigned int index) const;

            /** \brief Get the index of a specific dimension, by name. Return -1 if name is not found */
            int getDimensionIndex(const std::string &name) const;

            /** \brief Set the name of a dimension */
            void setDimensionName(unsigned int index, const std::string &name);

            double getMaximumExtent() const override;

            double getMeasure() const override;

            void enforceBounds(State *state) const override;

            bool satisfiesBounds(const State *state) const override;

            void copyState(State *destination, const State *source) const override;

            unsigned int getSerializationLength() const override;

            void serialize(void *serialization, const State *state) const override;

            void deserialize(State *state, const void *serialization) const override;

            double distance(const State *state1, const State *state2) const override;

            bool equalStates(const State *state1, const State *state2) const override;

            void interpolate(const State *from, const State *to, double t, State *state) const override;

            StateSamplerPtr allocDefaultStateSampler() const override;

            State *allocState() const override;

            void freeState(State *state) const override;

            double *getValueAddressAtIndex(State *state, unsigned int index) const override;

            void printState(const State *state, std::ostream &out) const override;

            void printSettings(std::ostream &out) const override;

            void registerProjections() override;

            void setup() override;

        protected:
            /** \brief The dimension of the space */
            unsigned int dimension_;

            /** \brief The bounds of the space (used for sampling) */
            RealVectorBounds bounds_;

            /** \brief Optional names for individual dimensions */
            std::vector<std::string> dimensionNames_;

            /** \brief Map from names to index values for dimensions */
            std::map<std::string, unsigned int> dimensionIndex_;

        private:
            /** \brief The size of a state, in bytes */
            std::size_t stateBytes_;
        };
    }
}

#endif
