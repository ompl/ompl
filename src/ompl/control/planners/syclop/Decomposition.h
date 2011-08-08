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

#include <iostream>
#include <set>
#include <vector>
#define BOOST_NO_HASH
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"

namespace ompl
{
    namespace control
    {
        class Decomposition
        {
        public:

            /* A decomposition consists of a fixed number of regions and fixed bounds. */
            Decomposition(const int n, const base::RealVectorBounds& b) : numRegions(n), bounds(b)
            {
            }

            virtual ~Decomposition()
            {
            }

            virtual int getNumRegions() const
            {
                return numRegions;
            }

            virtual const base::RealVectorBounds& getBounds() const
            {
                return bounds;
            }

            virtual double getRegionVolume(const int rid) const = 0;

            /* Returns the ID of the decomposition region containing the state s.
             * Most often, this is obtained by projecting s into the workspace and finding the appropriate region. */
            virtual int locateRegion(const base::State* s) const = 0;

            /* Project the state to a vector in R^k, where k is the dimension of this Decomposition.
               TODO Consider using the projection code used by KPIECE. */
            virtual void project(const base::State* s, std::valarray<double>& coord) const = 0;

            /* Stores the neighboring regions of region into the vector neighbors. */
            virtual void getNeighbors(const int rid, std::vector<int>& neighbors) const = 0;

        protected:

            const int numRegions;
            const base::RealVectorBounds &bounds;
        };
    }
}
#endif
