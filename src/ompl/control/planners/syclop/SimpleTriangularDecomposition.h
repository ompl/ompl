/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_SIMPLETRIANGULARDECOMPOSITION_
#define OMPL_CONTROL_PLANNERS_SYCLOP_SIMPLETRIANGULARDECOMPOSITION_

extern "C"
{
    #define REAL double
    #define VOID void
    #define ANSI_DECLARATORS
    #include <stdio.h>
    #include <stdlib.h>
    #include <triangle.h>
}
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"

namespace ompl
{
    namespace control
    {
        /** \brief A SimpleTriangularDecomposition is a triangulation that ignores obstacles. */
        class SimpleTriangularDecomposition : public Decomposition
        {
        public:
            /** \brief Constructor. Creates a SimpleTriangularDecomposition over the given bounds, which must be 2-dimensional. */
            SimpleTriangularDecomposition(const std::size_t dim, const base::RealVectorBounds& b);

            virtual ~SimpleTriangularDecomposition()
            {
            }

            virtual double getRegionVolume(const int rid) const;

            virtual void getNeighbors(const int rid, std::vector<int>& neighbors) const;

            virtual int locateRegion(const base::State* s) const;

            virtual void sampleFromRegion(const int triID, RNG& rng, std::vector<double>& coord) const;

        protected:
            std::vector<std::vector<std::pair<double,double> > > triVertices_;
            std::vector<std::vector<int> > triNeighbors_;
            mutable std::vector<double> triVolume_;

        private:
            class LocatorGrid : public GridDecomposition
            {
            public:
                LocatorGrid(const int len, const Decomposition* d) :
                    GridDecomposition(len, d->getDimension(), d->getBounds()),
                    triDecomp(d)
                {
                }

                virtual ~LocatorGrid()
                {
                }

                virtual void project(const base::State* s, std::vector<double>& coord) const
                {
                    triDecomp->project(s, coord);
                }

                virtual void sampleFullState(const base::StateSamplerPtr& sampler, const std::vector<double>& coord, base::State* s) const
                {
                }

                const std::vector<int>& locateTriangles(const base::State* s) const
                {
                    return regToTriangles_[locateRegion(s)];
                }

                void buildTriangleMap(const std::vector<std::vector<std::pair<double,double> > >& triVertices)
                {
                    regToTriangles_.resize(getNumRegions());
                    std::vector<double> bboxLow(2);
                    std::vector<double> bboxHigh(2);
                    std::vector<int> gridCoord[2];
                    for (int i = 0; i < triVertices.size(); ++i)
                    {
                        const std::vector<std::pair<double,double> >& pts = triVertices[i];
                        bboxLow[0] = pts[0].first;
                        bboxLow[1] = pts[0].second;
                        bboxHigh[0] = bboxLow[0];
                        bboxHigh[1] = bboxLow[1];

                        for (int j = 1; j < 3; ++j)
                        {
                            if (pts[j].first < bboxLow[0])
                                bboxLow[0] = pts[j].first;
                            else if (pts[j].first > bboxHigh[0])
                                bboxHigh[0] = pts[j].first;
                            if (pts[j].second < bboxLow[1])
                                bboxLow[1] = pts[j].second;
                            else if (pts[j].second > bboxHigh[1])
                                bboxHigh[1] = pts[j].second;
                        }

                        coordToGridCoord(bboxLow, gridCoord[0]);
                        coordToGridCoord(bboxHigh, gridCoord[1]);

                        /* every grid cell within bounding box gets
                           current triangle added to its map entry */
                        std::vector<int> c(2);
                        for (int x = gridCoord[0][0]; x <= gridCoord[1][0]; ++x)
                        {
                            for (int y = gridCoord[0][1]; y <= gridCoord[1][1]; ++y)
                            {
                                c[0] = x;
                                c[1] = y;
                                int cellID = gridCoordToRegion(c);
                                regToTriangles_[cellID].push_back(i);
                            }
                        }

/*                        printf("triangle ");
                        for (int j = 0; j < 3; ++j)
                            printf(" (%f, %f)", pts[j].first, pts[j].second);
                        printf(" lies within bounding box\n");
                        printf(" (%f,%f)", bboxLow[0], bboxLow[1]);
                        printf(" (%f,%f)\n", bboxHigh[0], bboxHigh[1]);
                        printf("which maps to grid bounding box\n");
                        printf(" (%d,%d)", gridCoord[0][0], gridCoord[0][1]);
                        printf(" (%d,%d)\n", gridCoord[1][0], gridCoord[1][1]);
*/
                    }

                    /*for (int i = 0; i < getNumRegions(); ++i)
                    {
                        printf("grid cell %d maps to triangles\n", i);
                        const std::vector<int>& triangles = regToTriangles_[i];
                        for (int j = 0; j < triangles.size(); ++j)
                            printf(" %d", triangles[j]);
                        printf("\n");
                    }*/
                }

            protected:
                const Decomposition* triDecomp;
                /* map from locator grid cell ID to set of triangles with which
                 * that cell intersects */
                std::vector<std::vector<int> > regToTriangles_;
            };

            /** \brief Helper method to triangulate the space and return the number of triangles. */
            int createTriangles();

            /** \brief Helper method to build a locator grid to help locate states in triangles. */
            void buildLocatorGrid();

            /** \brief Helper method to determine whether a point lies within a triangle. */
            bool triContains(const int triID, const std::vector<double>& coord) const;

            LocatorGrid locator;
        };
    }
}
#endif
