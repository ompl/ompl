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

ompl::control::GridDecomposition::GridDecomposition(const int len, const int dim, const base::RealVectorBounds& b) :
    Decomposition(len*len, b), length(len), dimension(dim), cellVolume(1.0)
{
    for (int i = 0; i < dim; ++i)
        cellVolume *= (b.high[i] - b.low[i]) / len;
}

/* This implementation requires time linear with the number of regions.
 * We can do constant time if we know the dimension offline (oopsmp-syclop has cases for 2 and 3),
 * but can we beat linear time with arbitrary dimension? */
void ompl::control::GridDecomposition::getNeighbors(const int rid, std::vector<int>& neighbors) const
{
    if (dimension == 1)
    {
        if (rid > 0)
            neighbors.push_back(rid-1);
        if (rid < length-1)
            neighbors.push_back(rid+1);
    }
    else if (dimension == 2)
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
            if (nc[0] >= 0 && nc[0] < length && nc[1] >= 0 && nc[1] < length)
                neighbors.push_back(nc[0]*length + nc[1]);
        }
    }
    else if (dimension == 3)
    {
        //TODO below is copied from oopsmp
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
    std::valarray<double> coord(dimension);
    project(s, coord);
    int region = 0;
    int factor = 1;
    for (int i = dimension-1; i >= 0; --i)
    {
        const int index = (int) (length*(coord[i]-bounds.low[i])/(bounds.high[i]-bounds.low[i]));
        region += factor*index;
        factor *= length;
    }
    return region;
}

bool ompl::control::GridDecomposition::areNeighbors(int r, int s) const
{
    if (r == s)
        return false;
    std::vector<int> rc(dimension);
    std::vector<int> sc(dimension);
    regionToCoord(r, rc);
    regionToCoord(s, sc);
    for (int i = 0; i < dimension; ++i)
    {
        if (abs(rc[i]-sc[i]) > 1)
            return false;
    }
    return true;
}

void ompl::control::GridDecomposition::regionToCoord(int rid, std::vector<int>& coord) const
{
    coord.resize(dimension);
    for (int i = dimension-1; i >= 0; --i)
    {
        int remainder = rid % length;
        coord[i] = remainder;
        rid /= length;
    }
}
