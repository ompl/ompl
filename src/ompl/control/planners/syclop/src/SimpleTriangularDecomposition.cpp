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

#include "ompl/control/planners/syclop/SimpleTriangularDecomposition.h"
#include <boost/lexical_cast.hpp>

ompl::control::SimpleTriangularDecomposition::SimpleTriangularDecomposition(const std::size_t dim, const base::RealVectorBounds& b) :
    Decomposition(dim, b),
    locator(64, this)
{
    int numTriangles = createTriangles();
    msg_.inform("Created %d triangles", numTriangles);
    setNumRegions(numTriangles);
    buildLocatorGrid();
    triVolume_.resize(getNumRegions(), -1);
}

double ompl::control::SimpleTriangularDecomposition::getRegionVolume(const int rid) const
{
    //TODO: notice that triVolume_ is mutable since this function is const
    if (triVolume_[rid] < 0)
    {
        const std::vector<std::pair<double,double> >& pts = triVertices_[rid];
        //this formula relies on the vertices being stored in counter-clockwise order
        triVolume_[rid] = 0.5*(
            (pts[0].first-pts[2].first)*(pts[1].second-pts[0].second)
          - (pts[0].first-pts[1].first)*(pts[2].second-pts[0].second)
        );
    }
    return triVolume_[rid];
}

void ompl::control::SimpleTriangularDecomposition::sampleFromRegion(const int triID, RNG& rng, std::vector<double>& coord) const
{
    /* Uniformly sample a point from within a triangle, using the approach discussed in
     * http://math.stackexchange.com/questions/18686/uniform-random-point-in-triangle */
    const std::vector<std::pair<double,double> >& pts = triVertices_[triID];
    coord.resize(2);
    const double r1 = sqrt(rng.uniform01());
    const double r2 = rng.uniform01();
    coord[0] = (1-r1)*pts[0].first + r1*(1-r2)*pts[1].first + r1*r2*pts[2].first;
    coord[1] = (1-r1)*pts[0].second + r1*(1-r2)*pts[1].second + r1*r2*pts[2].second;
}

void ompl::control::SimpleTriangularDecomposition::getNeighbors(const int rid, std::vector<int>& neighbors) const
{
    neighbors = triNeighbors_[rid];
}

int ompl::control::SimpleTriangularDecomposition::locateRegion(const base::State* s) const
{
    //TODO: on initialization, use locator grid to intersect with triangles
    //then locateRegion first maps to grid region
    //grid region corresponds to a small set of triangles with which it intersects
    //then linear search the set of triangles to find the one that contains this point
    std::vector<double> coord(2);
    project(s, coord);
    const std::vector<int>& triangles = locator.locateTriangles(s);
    int triangle = -1;
    for (std::vector<int>::const_iterator i = triangles.begin(); i != triangles.end(); ++i)
    {
        const int triID = *i;
        if (triContains(triID, coord))
        {
            if (triangle >= 0)
                printf("error: (%f,%f) is contained by triangles %d and %d\n",
                    coord[0], coord[1], triangle, triID);
            triangle = triID;
        }
    }
    //printf("state projected to triangle %d\n", triangle);
    return triangle;
}

int ompl::control::SimpleTriangularDecomposition::createTriangles()
{
    //use triangle functions to generate triangulation so that no triangle is more than 0.05% of total area
    const base::RealVectorBounds& bounds = getBounds();
    const double maxTriangleArea = bounds.getVolume()*0.0005;
    /* create a conforming Delaunay triangulation
       where each triangle takes up no more than 0.05% of
       the total area of the decomposition space */
    std::string triswitches = "pDznQ -a" + boost::lexical_cast<std::string>(maxTriangleArea);
    struct triangulateio in;
    in.numberofpoints = 4;
    in.numberofpointattributes = 0;
    in.pointlist = (REAL *) malloc(2*in.numberofpoints*sizeof(REAL));
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

    triVertices_.resize(out.numberoftriangles);
    triNeighbors_.resize(out.numberoftriangles);
    for (int i = 0; i < out.numberoftriangles; ++i)
    {
        std::vector<std::pair<double,double> >& pts = triVertices_[i];
        pts.resize(3);
        for (int j = 0; j < 3; ++j)
        {
            pts[j].first = out.pointlist[2*out.trianglelist[3*i+j]];
            pts[j].second = out.pointlist[2*out.trianglelist[3*i+j]+1];
        }
        for (int j = 0; j < 3; ++j)
        {
            if (out.neighborlist[3*i+j] >= 0)
                triNeighbors_[i].push_back(out.neighborlist[3*i+j]);
        }
    }

    //#define PRINTTRIANGLES

    #ifdef PRINTTRIANGLES
    /* write .node file:
            numpoints dimension numattributes numboundarymarkers
            pointID x y [attributes] [boundarymarker]
            ...
    */
    printf("--- node file ---\n");
    printf("%d 2 0 0\n", out.numberofpoints);
    for (int i = 0; i < out.numberofpoints; ++i)
        printf("%d %f %f\n", i, out.pointlist[2*i], out.pointlist[2*i+1]);

    /* write .ele file:
            numtriangles pointsPerTriangle numAttributes
            triangleID pointID pointID pointId [attributes]
            ...
    */
    printf("--- ele file ---\n");
    printf("%d 3 0\n", out.numberoftriangles);
    for (int i = 0; i < out.numberoftriangles; ++i)
        printf("%d %d %d %d\n", i, out.trianglelist[3*i], out.trianglelist[3*i+1], out.trianglelist[3*i+2]);

    printf("neighbors\n");
    for (int i = 0; i < out.numberoftriangles; ++i)
    {
        printf("%d", i);
        for (int j = 0; j < triNeighbors_[i].size(); ++j)
            printf(" %d", triNeighbors_[i][j]);
        printf("\n");
    }
    #endif

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

void ompl::control::SimpleTriangularDecomposition::buildLocatorGrid()
{
    locator.buildTriangleMap(triVertices_);
}

bool ompl::control::SimpleTriangularDecomposition::triContains(const int triID, const std::vector<double>& coord) const
{
    const std::vector<std::pair<double,double> >& pts = triVertices_[triID];
    for (int i = 0; i < 3; ++i)
    {
        /* point (coord[0],coord[1]) needs to be to the left of
           the vector from (ax,ay) to (bx,by) */
        const double ax = pts[i].first;
        const double ay = pts[i].second;
        const double bx = pts[(i+1)%3].first;
        const double by = pts[(i+1)%3].second;

        if ((coord[0]-ax)*(by-ay) - (bx-ax)*(coord[1]-ay) > 0.)
            return false;
    }

/*    printf("point (%f,%f) is inside of triangle", coord[0], coord[1]);
    for (int j = 0; j < 3; ++j)
        printf(" (%f, %f)", pts[j].first, pts[j].second);
    printf("\n");*/
    return true;
}
