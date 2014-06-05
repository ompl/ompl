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

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_TRIANGULARDECOMPOSITION_
#define OMPL_CONTROL_PLANNERS_SYCLOP_TRIANGULARDECOMPOSITION_

#include "ompl/base/State.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"

namespace ompl
{
    namespace control
    {
        /** \brief A TriangularDecomposition is a triangulation that ignores obstacles. */
        class TriangularDecomposition : public Decomposition
        {
        public:
            virtual ~TriangularDecomposition()
            {
            }

            virtual double getRegionVolume(unsigned int triID);

            virtual void getNeighbors(unsigned int triID, std::vector<unsigned int>& neighbors) const;

            virtual int locateRegion(const base::State *s) const;

            virtual void sampleFromRegion(unsigned int triID, RNG &rng, std::vector<double>& coord) const;

            //Debug method: prints this decomposition as a list of polygons
            void print(std::ostream& out) const;


        protected:
            struct Vertex
            {
                double x, y;
            };

            //A polygon is a list of vertices in counter-clockwise order.
            struct Polygon
            {
                Polygon(unsigned int nv) : pts(nv) {}
                virtual ~Polygon() {}
                std::vector<Vertex> pts;
            };

            struct Triangle : public Polygon
            {
                Triangle(void) : Polygon(3) {}
                virtual ~Triangle() {}
                std::vector<unsigned int> neighbors;
                double volume;
            };

            /** \brief Constructor. Creates a TriangularDecomposition over the given bounds, which must be 2-dimensional.
                The triangulation will respect any given obstacles, which are assumed to be convex polygons.
             */
            TriangularDecomposition(unsigned int dim, const base::RealVectorBounds &b,
                const std::vector<Polygon>& holes = std::vector<Polygon>());

            /** \brief Helper method to triangulate the space and return the number of triangles. */
            virtual unsigned int createTriangles();

            std::vector<Triangle> triangles_;
            std::vector<Polygon> holes_;

        private:
            class LocatorGrid : public GridDecomposition
            {
            public:
                LocatorGrid(unsigned int len, const Decomposition *d) :
                    GridDecomposition(len, d->getDimension(), d->getBounds()),
                    triDecomp(d)
                {
                }

                virtual ~LocatorGrid()
                {
                }

                virtual void project(const base::State *s, std::vector<double>& coord) const
                {
                    triDecomp->project(s, coord);
                }

                virtual void sampleFullState(const base::StateSamplerPtr& /*sampler*/, const std::vector<double>& /*coord*/, base::State* /*s*/) const
                {
                }

                const std::vector<unsigned int>& locateTriangles(const base::State *s) const
                {
                    return regToTriangles_[locateRegion(s)];
                }

                void buildTriangleMap(const std::vector<Triangle>& triangles);

            protected:
                const Decomposition *triDecomp;
                /* map from locator grid cell ID to set of triangles with which
                 * that cell intersects */
                std::vector<std::vector<unsigned int> > regToTriangles_;
            };

            /** \brief Helper method to build a locator grid to help locate states in triangles. */
            void buildLocatorGrid();

            /** \brief Helper method to determine whether a point lies within a triangle. */
            bool triContains(const Triangle& tri, const std::vector<double>& coord) const;

            /** \brief Helper method to generate a point within a convex polygon. */
            Vertex pointWithinPoly(const Polygon& poly) const;

            LocatorGrid locator;
        };
    }
}
#endif
