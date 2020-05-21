/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

/* Author: Ryan Luna */

#ifndef OMPL_GEOMETRIC_PLANNERS_XXL_XXLDECOMPOSITION_
#define OMPL_GEOMETRIC_PLANNERS_XXL_XXLDECOMPOSITION_

#include <vector>
#include "ompl/base/State.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"

namespace ompl
{
    namespace geometric
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::XXLDecomposition */
        OMPL_CLASS_FORWARD(XXLDecomposition);
        /// @endcond

        /** \class ompl::geometric::XXLDecompositionPtr
            \brief A shared pointer wrapper for ompl::geometric::XXLDecomposition */

        /** \brief */
        class XXLDecomposition
        {
        public:
            /** \brief Constructor. */
            XXLDecomposition()
            {
            }

            virtual ~XXLDecomposition()
            {
            }

            /** \brief Returns the number of regions in this XXLDecomposition. */
            virtual int getNumRegions() const = 0;

            /** \brief Return the dimension of this XXLDecomposition */
            virtual int getDimension() const = 0;

            /** \brief Return the number of layers possible in this decomposition.  Must be at least 1 */
            virtual int numLayers() const = 0;

            /** \brief Returns the index of the region containing a given State.
             * Most often, this is obtained by first calling project().
             * Returns -1 if no region contains the State. */
            virtual int locateRegion(const base::State *s) const = 0;

            /** \brief Return the region that this coordinate lies in.  Returns -1 if this coord is out of bounds */
            virtual int locateRegion(const std::vector<double> &coord) const = 0;

            /** \brief Stores the given region's neighbors into a given vector. These are adjacent neighbors*/
            virtual void getNeighbors(int rid, std::vector<int> &neighbors) const = 0;

            /** \brief Return a list of regions in the neighborhood of rid.  This method is intended to retrieve all
               regions with in a 'radius'.  This method should return a superset of getNeighbors (or equal).  The
               neighborhood may or not be adjacent regions.  Think of this as the 8-connected neighborhood of a grid,
               and getNeighbors as the 4-connected neighborhood of a grid. */
            virtual void getNeighborhood(int rid, std::vector<int> &neighborhood) const
            {
                return getNeighbors(rid, neighborhood);
            }

            /** \brief An admissible and consistent distance heuristic between two regions */
            virtual double distanceHeuristic(int r1, int r2) const = 0;

            /** \brief Sample a state \e s from region r in layer 0. */
            virtual bool sampleFromRegion(int r, base::State *s, const base::State *seed = nullptr) const = 0;

            /** \brief Sample a state \e s from region r in the given layer. */
            virtual bool sampleFromRegion(int r, base::State *s, const base::State *seed, int layer) const = 0;

            /** \brief Project the given State into the XXLDecomposition */
            virtual void project(const base::State *s, std::vector<double> &coord, int layer = 0) const = 0;

            /** \brief Project the state into the decomposition and retrieve the region for all valid layers */
            virtual void project(const base::State *s, std::vector<int> &layers) const = 0;

            /** \brief Returns true if the method steerToRegion is available */
            virtual bool canSteer() const
            {
                return false;
            }

            virtual bool steerToRegion(int /*r*/, int /*layer*/, const ompl::base::State * /*start*/,
                                       std::vector<ompl::base::State *> & /*states*/) const
            {
                OMPL_ERROR("steerToRegion has no default implementation");
                return false;
            }
        };
    }  // namespace geometric
}  // namespace ompl
#endif
