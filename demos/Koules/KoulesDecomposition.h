/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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

/* Author: Mark Moll */

#ifndef DEMOS_KOULES_DECOMPOSITION_
#define DEMOS_KOULES_DECOMPOSITION_

#include <ompl/control/planners/syclop/GridDecomposition.h>

class KoulesDecomposition : public ompl::control::GridDecomposition
{
public:
    // 16 x 16 x 16 grid
    KoulesDecomposition(const ompl::base::StateSpacePtr &space)
        : GridDecomposition(16, 3, bounds3(space->as<KoulesStateSpace>()->getBounds()))
    {
    }
    static ompl::base::RealVectorBounds bounds3(const ompl::base::RealVectorBounds &bounds)
    {
        ompl::base::RealVectorBounds b(3);
        std::copy(bounds.low.begin(), bounds.low.begin() + 3, b.low.begin());
        std::copy(bounds.high.begin(), bounds.high.begin() + 3, b.high.begin());
        return b;
    }
    void project(const ompl::base::State *s, std::vector<double> &coord) const override
    {
        const double* pos = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        coord.resize(3);
        coord[0] = pos[0];
        coord[1] = pos[1];
        coord[2] = pos[2];
    }

    void sampleFullState(const ompl::base::StateSamplerPtr &sampler,
    const std::vector<double>& coord, ompl::base::State *s) const override
    {
        double* pos = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        sampler->sampleUniform(s);
        pos[0] = coord[0];
        pos[1] = coord[1];
        pos[2] = coord[2];
    }
};

#endif
