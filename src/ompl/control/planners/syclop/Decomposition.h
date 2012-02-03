/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_DECOMPOSITION_
#define OMPL_CONTROL_PLANNERS_SYCLOP_DECOMPOSITION_

#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/State.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{
    namespace control
    {
        ClassForward(Decomposition);

        /** \brief A Decomposition is a partition of a bounded Euclidean space into a fixed number of regions which are denoted by integers. */
        class Decomposition
        {
        public:

            /** \brief Constructor. Creates a Decomposition with a given number of regions, a given dimension, and a given set of bounds. */
            Decomposition(const int n, const std::size_t dim, const base::RealVectorBounds& b) : numRegions_(n), dimension_(dim), bounds_(b)
            {
                if (dim > b.low.size())
                    throw Exception("Decomposition", "argument 'dim' exceeds dimension of given bounds");
                else if (dim < b.low.size())
                    msg_.warn("Decomposition: dimension of given bounds exceeds argument 'dim'. Using the first 'dim' values of bounds");
            }

            virtual ~Decomposition()
            {
            }

            /** \brief Returns the number of regions in this Decomposition. */
            virtual int getNumRegions() const
            {
                return numRegions_;
            }

            /** \brief Returns the dimension of this Decomposition. */
            virtual std::size_t getDimension() const
            {
                return dimension_;
            }

            /** \brief Returns the bounds of this Decomposition. */
            virtual const base::RealVectorBounds& getBounds() const
            {
                return bounds_;
            }

            /** \brief Returns the volume of a given region in this Decomposition. */
            virtual double getRegionVolume(const int rid) const = 0;

            /** \brief Returns the index of the region containing a given State.
             * Most often, this is obtained by first calling project(). */
            virtual int locateRegion(const base::State* s) const = 0;

            /** \brief Project a given State to a set of coordinates in R^k, where k is the dimension of this Decomposition. */
            virtual void project(const base::State* s, std::vector<double>& coord) const = 0;

            /** \brief Stores a given region's neighbors into a given vector. */
            virtual void getNeighbors(const int rid, std::vector<int>& neighbors) const = 0;

            /** \brief Samples a State from a given region using a given StateSampler. */
            virtual void sampleFromRegion(const int rid, const base::StateSamplerPtr& sampler, base::State* s) = 0;

            /** \brief Returns the bounds of a given region in this Decomposition. */
            virtual const base::RealVectorBounds& getRegionBounds(const int rid) = 0;

        protected:

            const int numRegions_;
            const std::size_t dimension_;
            const base::RealVectorBounds bounds_;
            msg::Interface msg_;
        };
    }
}
#endif
