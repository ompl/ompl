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

#ifndef OMPL_EXTENSIONS_TRIANGLE_TRIANGULARDECOMPOSITION_
#define OMPL_EXTENSIONS_TRIANGLE_TRIANGULARDECOMPOSITION_

#include "ompl/base/State.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/util/RandomNumbers.h"
#include <ostream>
#include <vector>
#include <set>

namespace ompl
{
    namespace control
    {
        /** \brief A TriangularDecomposition is a triangulation that ignores obstacles. */
        class TriangularDecomposition : public Decomposition
        {
            // \todo: Switch all geometry code to use boost::geometry.
            // This requires that we use boost version 1.47 or greater.
        public:
            struct Vertex
            {
                Vertex() = default;
                Vertex(double vx, double vy);
                bool operator==(const Vertex &v) const;
                double x, y;
            };

            // A polygon is a list of vertices in counter-clockwise order.
            struct Polygon
            {
                Polygon(int nv) : pts(nv)
                {
                }
                virtual ~Polygon() = default;
                std::vector<Vertex> pts;
            };

            struct Triangle : public Polygon
            {
                Triangle() : Polygon(3)
                {
                }
                ~Triangle() override = default;
                std::vector<int> neighbors;
                double volume;
            };

            /** \brief Creates a TriangularDecomposition over the given bounds, which must be 2-dimensional.
                The underlying mesh will be a conforming Delaunay triangulation.
                The triangulation will ignore any obstacles, given as a list of polygons.
                The triangulation will respect the boundaries of any regions of interest, given as a list of
                polygons. No two obstacles may overlap, and no two regions of interest may overlap.*/
            TriangularDecomposition(const base::RealVectorBounds &bounds,
                                    std::vector<Polygon> holes = std::vector<Polygon>(),
                                    std::vector<Polygon> intRegs = std::vector<Polygon>());

            ~TriangularDecomposition() override;

            int getNumRegions() const override
            {
                return triangles_.size();
            }

            double getRegionVolume(int triID) override;

            void getNeighbors(int triID, std::vector<int> &neighbors) const override;

            int locateRegion(const base::State *s) const override;

            void sampleFromRegion(int triID, RNG &rng, std::vector<double> &coord) const override;

            void setup();

            void addHole(const Polygon &hole);

            void addRegionOfInterest(const Polygon &region);

            int getNumHoles() const;

            int getNumRegionsOfInterest() const;

            const std::vector<Polygon> &getHoles() const;

            const std::vector<Polygon> &getAreasOfInterest() const;

            /** \brief Returns the region of interest that contains the given triangle ID.
                Returns -1 if the triangle ID is not within a region of interest. */
            int getRegionOfInterestAt(int triID) const;

            // Debug method: prints this decomposition as a list of polygons
            void print(std::ostream &out) const;

        protected:
            /** \brief Helper method to triangulate the space and return the number of triangles. */
            virtual int createTriangles();

            std::vector<Triangle> triangles_;
            std::vector<Polygon> holes_;
            std::vector<Polygon> intRegs_;
            /** \brief Maps from triangle ID to index of Polygon in intReg_ that
                contains the triangle ID. Maps to -1 if the triangle ID is not in
                a region of interest. */
            std::vector<int> intRegInfo_;
            double triAreaPct_;

        private:
            class LocatorGrid : public GridDecomposition
            {
            public:
                LocatorGrid(int len, const Decomposition *d)
                  : GridDecomposition(len, d->getDimension(), d->getBounds()), triDecomp(d)
                {
                }

                ~LocatorGrid() override = default;

                void project(const base::State *s, std::vector<double> &coord) const override
                {
                    triDecomp->project(s, coord);
                }

                void sampleFullState(const base::StateSamplerPtr & /*sampler*/, const std::vector<double> & /*coord*/,
                                     base::State * /*s*/) const override
                {
                }

                const std::vector<int> &locateTriangles(const base::State *s) const
                {
                    return regToTriangles_[locateRegion(s)];
                }

                void buildTriangleMap(const std::vector<Triangle> &triangles);

            protected:
                const Decomposition *triDecomp;
                /* map from locator grid cell ID to set of triangles with which
                 * that cell intersects */
                std::vector<std::vector<int>> regToTriangles_;
            };

            /** \brief Helper method to build a locator grid to help locate states in triangles. */
            void buildLocatorGrid();

            /** \brief Helper method to determine whether a point lies within a triangle. */
            static bool triContains(const Triangle &tri, const std::vector<double> &coord);

            /** \brief Helper method to generate a point within a convex polygon. */
            static Vertex getPointInPoly(const Polygon &poly);

            LocatorGrid locator;
        };
    }
}
#endif
