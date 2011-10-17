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

#include "ompl/control/planners/syclop/GridDecomposition.h"

ompl::control::GridDecomposition::GridDecomposition(const int len, const std::size_t dim, const base::RealVectorBounds& b) :
    Decomposition(len*len, dim, b), length_(len), cellVolume_(1.0)
{
    for (std::size_t i = 0; i < dim; ++i)
        cellVolume_ *= (b.high[i] - b.low[i]) / len;
}

/* This implementation requires time linear with the number of regions.
 * We can do constant time if we know the dimension offline (oopsmp-syclop has cases for 2 and 3),
 * but can we beat linear time with arbitrary dimension? */
void ompl::control::GridDecomposition::getNeighbors(const int rid, std::vector<int>& neighbors) const
{
    if (dimension_ == 1)
    {
        if (rid > 0)
            neighbors.push_back(rid-1);
        if (rid < length_-1)
            neighbors.push_back(rid+1);
    }
    else if (dimension_ == 2)
    {
        static const int offset[] = {
            -1, -1,
             0, -1,
            +1, -1,
            -1,  0,
            +1,  0,
            -1, +1,
             0, +1,
            +1, +1
        };
        std::vector<int> coord(2);
        regionToCoord(rid, coord);
        std::vector<int> nc(2);
        for (std::size_t i = 0; i < 16; i += 2)
        {
            nc[0] = coord[0] + offset[i];
            nc[1] = coord[1] + offset[i+1];
            if (nc[0] >= 0 && nc[0] < length_ && nc[1] >= 0 && nc[1] < length_)
                neighbors.push_back(nc[0]*length_ + nc[1]);
        }
    }
    else if (dimension_ == 3)
    {
        //TODO FINISH (below is copied from oopsmp)
        static const int offset[] = {
            -1,  0, 0,
        	+1,  0, 0,
        	 0, -1, 0,
        	 0, +1, 0,
        	-1, -1, 0,
        	-1, +1, 0,
        	+1, -1, 0,
        	+1, +1, 0,
        	-1,  0, -1,
        	+1,  0, -1,
        	 0, -1, -1,
        	 0, +1, -1,
        	-1, -1, -1,
        	-1, +1, -1,
        	+1, -1, -1,
        	+1, +1, -1,
        	-1,  0, +1,
        	+1,  0, +1,
        	 0, -1, +1,
        	 0, +1, +1,
        	-1, -1, +1,
        	-1, +1, +1,
        	+1, -1, +1,
        	+1, +1, +1,
        	0, 0, -1,
        	0, 0, +1
        };
    }
    else
    {
        for (int s = 0; s < getNumRegions(); ++s)
        {
             if (areNeighbors(rid, s))
                 neighbors.push_back(s);
        }
    }
}

int ompl::control::GridDecomposition::locateRegion(const base::State* s) const
{
    std::valarray<double> coord(dimension_);
    project(s, coord);
    int region = 0;
    int factor = 1;
    for (int i = dimension_-1; i >= 0; --i)
    {
        const int index = (int) (length_*(coord[i]-bounds_.low[i])/(bounds_.high[i]-bounds_.low[i]));
        region += factor*index;
        factor *= length_;
    }
    return region;
}

bool ompl::control::GridDecomposition::areNeighbors(const int r, const int s) const
{
    if (r == s)
        return false;
    std::vector<int> rc(dimension_);
    std::vector<int> sc(dimension_);
    regionToCoord(r, rc);
    regionToCoord(s, sc);
    for (std::size_t i = 0; i < dimension_; ++i)
    {
        if (abs(rc[i]-sc[i]) > 1)
            return false;
    }
    return true;
}

void ompl::control::GridDecomposition::regionToCoord(int rid, std::vector<int>& coord) const
{
    coord.resize(dimension_);
    for (int i = dimension_-1; i >= 0; --i)
    {
        int remainder = rid % length_;
        coord[i] = remainder;
        rid /= length_;
    }
}

ompl::base::RealVectorBounds ompl::control::GridDecomposition::getRegionBounds(const int rid) const
{
    //todo store this in region obj so it's only computed once (computed lazily)
    ompl::base::RealVectorBounds regionBounds(dimension_);
    std::vector<int> rc(dimension_);
    regionToCoord(rid, rc);
    for (std::size_t i = 0; i < dimension_; ++i)
    {
        const double length = (bounds_.high[i] - bounds_.low[i]) / length_;
        regionBounds.low[i] = bounds_.low[i] + length*rc[i];
        regionBounds.high[i] = regionBounds.low[i] + length;
    }
    return regionBounds;
}
