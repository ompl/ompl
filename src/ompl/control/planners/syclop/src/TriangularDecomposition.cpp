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

#include "ompl/control/planners/syclop/TriangularDecomposition.h"
#include <boost/lexical_cast.hpp>

ompl::control::TriangularDecomposition::TriangularDecomposition(const std::size_t dim, const base::RealVectorBounds& b) :
    Decomposition(dim, b),
    locator(64, this)
{
    unsigned int numTriangles = createTriangles();
    msg_.inform("Created %u triangles", numTriangles);
    setNumRegions(numTriangles);
    buildLocatorGrid();
}

double ompl::control::TriangularDecomposition::getRegionVolume(const int triID)
{
    Triangle& tri = triangles_[triID];
    if (tri.volume < 0)
    {
        /* This triangle area formula relies on the vertices being
         * stored in counter-clockwise order. */
        tri.volume = 0.5*(
            (tri.pts[0].x-tri.pts[2].x)*(tri.pts[1].y-tri.pts[0].y)
          - (tri.pts[0].x-tri.pts[1].x)*(tri.pts[2].y-tri.pts[0].y)
        );
    }
    return tri.volume;
}

void ompl::control::TriangularDecomposition::sampleFromRegion(const int triID, RNG& rng, std::vector<double>& coord) const
{
    /* Uniformly sample a point from within a triangle, using the approach discussed in
     * http://math.stackexchange.com/questions/18686/uniform-random-point-in-triangle */
    const Triangle& tri = triangles_[triID];
    coord.resize(2);
    const double r1 = sqrt(rng.uniform01());
    const double r2 = rng.uniform01();
    coord[0] = (1-r1)*tri.pts[0].x + r1*(1-r2)*tri.pts[1].x + r1*r2*tri.pts[2].x;
    coord[1] = (1-r1)*tri.pts[0].y + r1*(1-r2)*tri.pts[1].y + r1*r2*tri.pts[2].y;
}

void ompl::control::TriangularDecomposition::getNeighbors(const int triID, std::vector<int>& neighbors) const
{
    neighbors = triangles_[triID].neighbors;
}

int ompl::control::TriangularDecomposition::locateRegion(const base::State* s) const
{
    std::vector<double> coord(2);
    project(s, coord);
    const std::vector<int>& gridTriangles = locator.locateTriangles(s);
    int triangle = -1;
    for (std::vector<int>::const_iterator i = gridTriangles.begin(); i != gridTriangles.end(); ++i)
    {
        const int triID = *i;
        if (triContains(triangles_[triID], coord))
        {
            if (triangle >= 0)
                msg_.error("Decomposition coordinate (%f,%f) is somehow contained by multiple triangles.\n", coord[0], coord[1]);
            triangle = triID;
        }
    }
    return triangle;
}

unsigned int ompl::control::TriangularDecomposition::createTriangles()
{
    /* create a conforming Delaunay triangulation
       where each triangle takes up no more than 0.03% of
       the total area of the decomposition space */
    const base::RealVectorBounds& bounds = getBounds();
    const double maxTriangleArea = bounds.getVolume()*0.0003;
    std::string triswitches = "pDznQ -a" + boost::lexical_cast<std::string>(maxTriangleArea);
    struct triangulateio in;
    in.numberofpoints = 4;
    in.numberofpointattributes = 0;
    in.pointlist = (REAL*) malloc(2*in.numberofpoints*sizeof(REAL));
    in.pointlist[0] = bounds.low[0];
    in.pointlist[1] = bounds.low[1];
    in.pointlist[2] = bounds.high[0];
    in.pointlist[3] = bounds.low[1];
    in.pointlist[4] = bounds.high[0];
    in.pointlist[5] = bounds.high[1];
    in.pointlist[6] = bounds.low[0];
    in.pointlist[7] = bounds.high[1];

    in.pointattributelist = NULL;
    in.pointmarkerlist = NULL;

    in.numberofsegments = 4;
    in.segmentlist = (int*) malloc(2*in.numberofpoints*sizeof(int));
    for (int i = 0; i < in.numberofpoints; ++i)
    {
        in.segmentlist[2*i] = i;
        in.segmentlist[2*i+1] = (i+1) % in.numberofpoints;
    }
    in.segmentmarkerlist = (int*) NULL;

    in.numberofholes = 0;
    in.numberofregions = 0;
    in.regionlist = NULL;

    struct triangulateio out;
    out.pointlist = (REAL*) NULL;
    out.pointattributelist = (REAL*) NULL;
    out.pointmarkerlist = (int*) NULL;
    out.trianglelist = (int*) NULL;
    out.triangleattributelist = (REAL*) NULL;
    out.neighborlist = (int*) NULL;
    out.segmentlist = (int*) NULL;
    out.segmentmarkerlist = (int*) NULL;
    out.edgelist = (int*) NULL;
    out.edgemarkerlist = (int*) NULL;
    out.pointlist = (REAL*) NULL;
    out.pointattributelist = (REAL*) NULL;
    out.trianglelist = (int*) NULL;
    out.triangleattributelist = (REAL*) NULL;

    struct triangulateio *vorout = NULL;
    triangulate(const_cast<char*>(triswitches.c_str()), &in, &out, vorout);

    triangles_.resize(out.numberoftriangles);
    for (int i = 0; i < out.numberoftriangles; ++i)
    {
        Triangle& t = triangles_[i];
        for (int j = 0; j < 3; ++j)
        {
            t.pts[j].x = out.pointlist[2*out.trianglelist[3*i+j]];
            t.pts[j].y = out.pointlist[2*out.trianglelist[3*i+j]+1];
            if (out.neighborlist[3*i+j] >= 0)
                t.neighbors.push_back(out.neighborlist[3*i+j]);
        }
        t.volume = -1.;
    }

    trifree(in.pointlist);
    trifree(in.segmentlist);
    trifree(out.pointlist);
    trifree(out.pointmarkerlist);
    trifree(out.trianglelist);
    trifree(out.neighborlist);
    trifree(out.edgelist);
    trifree(out.edgemarkerlist);
    trifree(out.segmentlist);
    trifree(out.segmentmarkerlist);

    return out.numberoftriangles;
}

void ompl::control::TriangularDecomposition::buildLocatorGrid()
{
    locator.buildTriangleMap(triangles_);
}

bool ompl::control::TriangularDecomposition::triContains(const Triangle& tri, const std::vector<double>& coord) const
{
    for (int i = 0; i < 3; ++i)
    {
        /* point (coord[0],coord[1]) needs to be to the left of
           the vector from (ax,ay) to (bx,by) */
        const double ax = tri.pts[i].x;
        const double ay = tri.pts[i].y;
        const double bx = tri.pts[(i+1)%3].x;
        const double by = tri.pts[(i+1)%3].y;

        // return false if the point is instead to the right of the vector
        if ((coord[0]-ax)*(by-ay) - (bx-ax)*(coord[1]-ay) > 0.)
            return false;
    }
    return true;
}

void ompl::control::TriangularDecomposition::LocatorGrid::buildTriangleMap(const std::vector<Triangle>& triangles)
{
    regToTriangles_.resize(getNumRegions());
    std::vector<double> bboxLow(2);
    std::vector<double> bboxHigh(2);
    std::vector<int> gridCoord[2];
    for (int i = 0; i < triangles.size(); ++i)
    {
        const Triangle& tri = triangles[i];
        bboxLow[0] = tri.pts[0].x;
        bboxLow[1] = tri.pts[0].y;
        bboxHigh[0] = bboxLow[0];
        bboxHigh[1] = bboxLow[1];

        for (int j = 1; j < 3; ++j)
        {
            if (tri.pts[j].x < bboxLow[0])
                bboxLow[0] = tri.pts[j].x;
            else if (tri.pts[j].x > bboxHigh[0])
                bboxHigh[0] = tri.pts[j].x;
            if (tri.pts[j].y < bboxLow[1])
                bboxLow[1] = tri.pts[j].y;
            else if (tri.pts[j].y > bboxHigh[1])
                bboxHigh[1] = tri.pts[j].y;
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
    }
}
