/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

#ifndef OMPL_EXTENSIONS_TRIANGLE_PROPOSITIONALTRIANGULARDECOMPOSITION_
#define OMPL_EXTENSIONS_TRIANGLE_PROPOSITIONALTRIANGULARDECOMPOSITION_

#include "ompl/extensions/triangle/TriangularDecomposition.h"
#include "ompl/control/planners/ltl/PropositionalDecomposition.h"
#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/base/State.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <ostream>
#include <vector>

namespace ompl
{
    namespace control
    {
        /** \brief A PropositionalTriangularDecomposition is a triangulation that ignores
            obstacles and respects propositional regions of interest. Practically speaking,
            it is both a TriangularDecomposition and a PropositionalDecomposition, but it is
            implemented without using multiple inheritance. */
        class PropositionalTriangularDecomposition : public PropositionalDecomposition
        {
        public:
            using Polygon = TriangularDecomposition::Polygon;
            using Vertex = TriangularDecomposition::Vertex;

            /** \brief Creates a PropositionalTriangularDecomposition over the given bounds,
                which must be 2-dimensional.
                The underlying mesh will be a conforming Delaunay triangulation.
                The triangulation will ignore any obstacles, given as a list of polygons.
                The triangulation will respect the boundaries of any propositional regions
                of interest, given as a list of polygons. */
            PropositionalTriangularDecomposition(const base::RealVectorBounds &bounds,
                                                 const std::vector<Polygon> &holes = std::vector<Polygon>(),
                                                 const std::vector<Polygon> &props = std::vector<Polygon>());

            ~PropositionalTriangularDecomposition() override = default;

            int getNumProps() const override;

            World worldAtRegion(int triID) override;

            void setup();

            void addHole(const Polygon &hole);

            void addProposition(const Polygon &prop);

            const std::vector<Polygon> &getHoles() const;

            const std::vector<Polygon> &getPropositions() const;

            // Debug method: prints this decomposition as a list of polygons
            void print(std::ostream &out) const;

        protected:
            TriangularDecomposition *triDecomp_;
        };
    }
}
#endif
