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

#include "ompl/extensions/triangle/PropositionalTriangularDecomposition.h"
#include "ompl/extensions/triangle/TriangularDecomposition.h"
#include "ompl/control/planners/ltl/PropositionalDecomposition.h"
#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/base/State.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <ostream>
#include <set>
#include <vector>

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace
{
    /* PropositionalTriangularDecomposition creates a WrapperDecomposition under-the-hood,
       and hands it upward to the PropositionalDecomposition superclass constructor. */
    class WrapperDecomposition : public oc::TriangularDecomposition
    {
    public:
        typedef TriangularDecomposition::Polygon Polygon;
        typedef TriangularDecomposition::Vertex Vertex;
        WrapperDecomposition(const oc::Decomposition* decomp,
                             const ob::RealVectorBounds& bounds,
                             const std::vector<Polygon>& holes,
                             const std::vector<Polygon>& props);
        virtual ~WrapperDecomposition(void) {}
        virtual void project(const ob::State* s, std::vector<double>& coord) const;
        virtual void sampleFromRegion(unsigned int rid, ompl::RNG& rng, std::vector<double>& coord) const;
        virtual void sampleFullState(const ob::StateSamplerPtr& sampler,
            const std::vector<double>& coord, ob::State* s) const;
    protected:
        const oc::Decomposition* decomp_;
    };
}

ompl::control::PropositionalTriangularDecomposition::PropositionalTriangularDecomposition(
    const base::RealVectorBounds& bounds, const std::vector<Polygon>& holes, const std::vector<Polygon>& props) :
    PropositionalDecomposition(
        DecompositionPtr(new WrapperDecomposition(this, bounds, holes, props))
    ),
    triDecomp_(static_cast<TriangularDecomposition*>(decomp_.get()))
{
}

ompl::control::PropositionalTriangularDecomposition::~PropositionalTriangularDecomposition(void)
{
}

unsigned int ompl::control::PropositionalTriangularDecomposition::getNumProps(void) const
{
    return triDecomp_->getNumRegionsOfInterest();
}

ompl::control::World ompl::control::PropositionalTriangularDecomposition::worldAtRegion(unsigned int triID)
{
    unsigned int numProps = getNumProps();
    World world(numProps);
    for (unsigned int p = 0; p < numProps; ++p)
        world[p] = false;
    if (triID == -1) return world;
    const std::set<unsigned int>& props = triDecomp_->getRegionsOfInterestAt(triID);
    typedef std::set<unsigned int>::const_iterator PropIter;
    for (PropIter pi = props.begin(); pi != props.end(); ++pi)
        world[*pi] = true;
    return world;
}

void ompl::control::PropositionalTriangularDecomposition::setup(void)
{
    triDecomp_->setup();
}

void ompl::control::PropositionalTriangularDecomposition::addHole(const Polygon& hole)
{
    triDecomp_->addHole(hole);
}

void ompl::control::PropositionalTriangularDecomposition::addProposition(const Polygon& prop)
{
    triDecomp_->addRegionOfInterest(prop);
}

const std::vector<ompl::control::PropositionalTriangularDecomposition::Polygon>&
    ompl::control::PropositionalTriangularDecomposition::getHoles(void) const
{
    return triDecomp_->getHoles();
}

const std::vector<ompl::control::PropositionalTriangularDecomposition::Polygon>&
    ompl::control::PropositionalTriangularDecomposition::getPropositions(void) const
{
    return triDecomp_->getAreasOfInterest();
}

void ompl::control::PropositionalTriangularDecomposition::print(std::ostream& out) const
{
    triDecomp_->print(out);
}

namespace
{
    WrapperDecomposition::WrapperDecomposition(const oc::Decomposition* decomp,
                                               const ob::RealVectorBounds& bounds,
                                               const std::vector<Polygon>& holes,
                                               const std::vector<Polygon>& props)
        : oc::TriangularDecomposition(bounds, holes, props),
          decomp_(decomp)
    {
    }

    void WrapperDecomposition::project(const ob::State* s, std::vector<double>& coord) const
    {
        decomp_->project(s, coord);
    }

    void WrapperDecomposition::sampleFromRegion(unsigned int rid,
                                                ompl::RNG& rng,
                                                std::vector<double>& coord) const
    {
        decomp_->sampleFromRegion(rid, rng, coord);
    }

    void WrapperDecomposition::sampleFullState(const ob::StateSamplerPtr& sampler,
                                               const std::vector<double>& coord,
                                               ob::State* s) const
    {
        decomp_->sampleFullState(sampler, coord, s);
    }
}
