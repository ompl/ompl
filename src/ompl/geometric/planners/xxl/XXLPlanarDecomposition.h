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

#ifndef OMPL_GEOMETRIC_PLANNERS_XXL_XXLPLANARDECOMPOSITION_
#define OMPL_GEOMETRIC_PLANNERS_XXL_XXLPLANARDECOMPOSITION_

#include <boost/math/constants/constants.hpp>
#include "ompl/geometric/planners/xxl/XXLDecomposition.h"
#include "ompl/util/RandomNumbers.h"

namespace ompl
{
    namespace geometric
    {
        // A decomposition of a 2D planar space (position and orientation in 2D)
        // The orientation part of the decomposition is circular (2pi = 0)
        class XXLPlanarDecomposition : public XXLDecomposition
        {
        public:
            XXLPlanarDecomposition(const base::RealVectorBounds& xyBounds, const std::vector<int>& xySlices,
                                   const int thetaSlices, bool diagonalEdges = false);

            XXLPlanarDecomposition(const base::RealVectorBounds& xyBounds, const std::vector<int>& xySlices,
                                   const int thetaSlices, double thetaLowerBound, double thetaUpperBound, bool diagonalEdges = false);

            virtual ~XXLPlanarDecomposition();

            /// \brief Return the total number of regions in this decomposition
            virtual int getNumRegions() const;

            /// \brief Return the dimension of this HiLoDecomposition
            virtual int getDimension() const;

            /** \brief Return the number of layers possible in this decomposition.  Must be at least 1 */
            virtual int numLayers() const = 0;

            /// \brief Return the id of the region that this state falls in
            virtual int locateRegion(const base::State *s) const;

            /// \brief Return the id of the region that this coordinate falls in
            virtual int locateRegion(const std::vector<double>& coord) const;

            /// \brief Stores the given region's neighbors into a given vector.
            virtual void getNeighbors(int rid, std::vector<int>& neighbors) const;

            /// \brief Stores the given region's neighbors into the vector.  This returns the 8-connected grid neighbors
            /// of the cell, regardless of whether diagonal edges exist.
            virtual void getNeighborhood(int rid, std::vector<int>& neighborhood) const;

            /// \brief An admissible and consistent distance heuristic between two regions.  Manhattan distance on grid
            virtual double distanceHeuristic(int r1, int r2) const;

            /** \brief Sample a state \e s from region r in layer 0. */
            virtual bool sampleFromRegion(int r, base::State* s, const base::State* seed = nullptr) const = 0;

            /** \brief Sample a state \e s from region r in the given layer. */
            virtual bool sampleFromRegion(int r, base::State* s, const base::State* seed, int layer) const = 0;

            /** \brief Project the given State into the XXLDecomposition */
            virtual void project(const base::State *s, std::vector<double>& coord, int layer = 0) const = 0;

            /** \brief Project the state into the decomposition and retrieve the region for all valid layers */
            virtual void project(const base::State *s, std::vector<int>& layers) const = 0;



            // \brief Return the (discrete) grid cell coordinates of the given region id
            void ridToGridCell(int rid, std::vector<int>& cell) const;

            /// \brief Return the region id corresponding to the (discrete) grid cell coordinates
            int gridCellToRid(const std::vector<int>& cell) const;

            /// \brief Return the region id of the given coordinate in the decomposition
            int coordToRegion(const std::vector<double>& coord) const;
            int coordToRegion(const double* coord) const;

            /// \brief Return true if the decomposition has diagonal edges
            bool hasDiagonalEdges() const;

        protected:
            // Create the new graph structure, mostly edge weights
            void constructGraph();

            // Compute the neighbors of rid (no diagonal edges)
            void getNonDiagonalNeighbors(int rid, std::vector<int>& neighbors) const;

            // Compute the neighbors of rid (including diagonal edges)
            void getDiagonalNeighbors(int rid, std::vector<int>& neighbors) const;

            // Sample a point in the SE(2) decomposition (position and orientation)
            void sampleCoordinateFromRegion(int r, std::vector<double>& coord) const;
            void sampleCoordinateFromRegion(int r, double* coord) const;

            // If true, the decomposition has diagonal edges
            bool diagonalEdges_;

            // The bounds on the Euclidean part of the decomposition
            base::RealVectorBounds xyBounds_;
            // The bounds on the orientation part of the decomposition
            double thetaLow_{-boost::math::constants::pi<double>()};
            double thetaHigh_{boost::math::constants::pi<double>()};

            // The number of splits in the X and Y dimensions
            std::vector<int> xySlices_;
            // The number of splits in orientation
            int thetaSlices_;

            // The total number of regions
            int numRegions_;

            // The total range of x and y
            double dx_, dy_;
            // The size of an individual cell in x and y
            double xSize_, ySize_;

            // The total range of theta
            double dTheta_;
            // The size of an individual cell in theta
            double thetaSize_;

            // The realized dimension of this decomposition (from 1-3)
            int dimension_;

            // Random number generator.
            mutable ompl::RNG rng_;
        };
    }
}


#endif
