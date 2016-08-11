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

#include "ompl/control/planners/ltl/PropositionalDecomposition.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/base/StateSampler.h"
#include <vector>

ompl::control::PropositionalDecomposition::PropositionalDecomposition(const DecompositionPtr &decomp)
  : Decomposition(decomp->getDimension(), decomp->getBounds()), decomp_(decomp)
{
}

ompl::control::PropositionalDecomposition::~PropositionalDecomposition() = default;

int ompl::control::PropositionalDecomposition::getNumRegions() const
{
    return decomp_->getNumRegions();
}

double ompl::control::PropositionalDecomposition::getRegionVolume(int rid)
{
    return decomp_->getRegionVolume(rid);
}

int ompl::control::PropositionalDecomposition::locateRegion(const base::State *s) const
{
    return decomp_->locateRegion(s);
}

void ompl::control::PropositionalDecomposition::project(const base::State *s, std::vector<double> &coord) const
{
    return decomp_->project(s, coord);
}

void ompl::control::PropositionalDecomposition::getNeighbors(int rid, std::vector<int> &neighbors) const
{
    decomp_->getNeighbors(rid, neighbors);
}

void ompl::control::PropositionalDecomposition::sampleFromRegion(int rid, RNG &rng, std::vector<double> &coord) const
{
    decomp_->sampleFromRegion(rid, rng, coord);
}

void ompl::control::PropositionalDecomposition::sampleFullState(const base::StateSamplerPtr &sampler,
                                                                const std::vector<double> &coord, base::State *s) const
{
    decomp_->sampleFullState(sampler, coord, s);
}
