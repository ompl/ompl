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

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_GRIDDECOMPOSITION_
#define OMPL_CONTROL_PLANNERS_SYCLOP_GRIDDECOMPOSITION_

#include <cstdlib>
#include <valarray>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/datastructures/Grid.h"

namespace ompl
{
    namespace control
    {
        class GridDecomposition : public Decomposition
        {
        public:
            /* In some areas of this class, we are assuming a 2-dimensional grid. This will change. */
            GridDecomposition(const int len, const int dim, const base::RealVectorBounds& b);

            virtual ~GridDecomposition()
            {
            }

            virtual double getRegionVolume(const int rid) const
            {
                return cellVolume;
            }

            /* This implementation requires time linear with the number of regions.
             * We can do constant time if we know the dimension offline (oopsmp-syclop has cases for 2 and 3),
             * but can we beat linear time with arbitrary dimension? */
            virtual void getNeighbors(const int rid, std::vector<int>& neighbors);

            virtual int locateRegion(const base::State* s);

        protected:
            virtual bool areNeighbors(int r, int s);

        private:
            /* Convert a region ID to a grid coordinate, which is a vector of length equivalent
             * to the dimension of the grid. */
            void regionToCoord(int rid, std::vector<int>& coord);

            const int length;
            const int dimension;
            double cellVolume;
        };
    }
}
#endif
