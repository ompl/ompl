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

/* Author: Ryan Luna */

#ifndef OMPL_GEOMETRIC_PLANNERS_XXL_XXLPOSITIONDECOMPOSITION_
#define OMPL_GEOMETRIC_PLANNERS_XXL_XXLPOSITIONDECOMPOSITION_

#include "ompl/geometric/planners/xxl/XXLDecomposition.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/util/RandomNumbers.h"

namespace ompl
{
    namespace geometric
    {
        // A decomposition of an n-dimensional Euclidean space
        class XXLPositionDecomposition : public XXLDecomposition
        {
        public:
            XXLPositionDecomposition(const base::RealVectorBounds &bounds, const std::vector<int> &slices,
                                     bool diagonalEdges = false);
            virtual ~XXLPositionDecomposition();

            virtual int getNumRegions() const;

            /// \brief Return the dimension of this HiLoDecomposition
            virtual int getDimension() const;

            /** \brief Return the number of layers possible in this decomposition.  Must be at least 1 */
            virtual int numLayers() const = 0;

            /// \brief Return the id of the region that this state falls in
            virtual int locateRegion(const base::State *s) const;

            /// \brief Return the id of the region that this coordinate falls in
            virtual int locateRegion(const std::vector<double> &coord) const;

            /// \brief Stores the given region's neighbors into a given vector.
            virtual void getNeighbors(int rid, std::vector<int> &neighbors) const;

            /// \brief Stores the given region's neighbors into the vector.  This returns the 8-connected grid neighbors
            /// of the cell, regardless of whether diagonal edges exist.
            virtual void getNeighborhood(int rid, std::vector<int> &neighborhood) const;

            /// \brief An admissible and consistent distance heuristic between two regions.  Manhattan distance on grid
            virtual double distanceHeuristic(int r1, int r2) const;

            /** \brief Sample a state \e s from region r in layer 0. */
            virtual bool sampleFromRegion(int r, base::State *s, const base::State *seed = nullptr) const = 0;

            /** \brief Sample a state \e s from region r in the given layer. */
            virtual bool sampleFromRegion(int r, base::State *s, const base::State *seed, int layer) const = 0;

            /** \brief Project the given State into the XXLDecomposition */
            virtual void project(const base::State *s, std::vector<double> &coord, int layer = 0) const = 0;

            /** \brief Project the state into the decomposition and retrieve the region for all valid layers */
            virtual void project(const base::State *s, std::vector<int> &layers) const = 0;

            // \brief Return the (discrete) grid cell coordinates of the given region id
            void ridToGridCell(int rid, std::vector<int> &cell) const;

            /// \brief Return the region id corresponding to the (discrete) grid cell coordinates
            int gridCellToRid(const std::vector<int> &cell) const;

            /// \brief Return the region id of the given coordinate in the decomposition
            int coordToRegion(const std::vector<double> &coord) const;
            int coordToRegion(const double *coord) const;

            /// \brief Return true if the decomposition has diagonal edges
            bool hasDiagonalEdges() const;

        protected:
            // Compute the neighbors of rid (no diagonal edges)
            void getNonDiagonalNeighbors(int rid, std::vector<int> &neighbors) const;

            // Compute the neighbors of rid (including diagonal edges)
            void getDiagonalNeighbors(int rid, std::vector<int> &neighbors) const;

            // The bounds on the Euclidean part of the decomposition
            base::RealVectorBounds bounds_;

            // The number of divisions in each Euclidean dimension
            const std::vector<int> slices_;
            // The extent of the grid cells in each dimension
            std::vector<double> cellSizes_;

            // If true, the decomposition has diagonal edges
            bool diagonalEdges_;

            int numRegions_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif
