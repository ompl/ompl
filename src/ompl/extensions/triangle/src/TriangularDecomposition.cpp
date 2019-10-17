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

#include "ompl/extensions/triangle/TriangularDecomposition.h"
#include "ompl/base/State.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/util/Hash.h"
#include "ompl/util/String.h"
#include <ostream>
#include <utility>
#include <vector>
#include <set>
#include <string>
#include <unordered_map>
#include <cstdlib>

extern "C" {
#define REAL double
#define VOID void
#define ANSI_DECLARATORS
#include <triangle.h>
}

namespace std
{
    template <>
    struct hash<ompl::control::TriangularDecomposition::Vertex>
    {
        size_t operator()(const ompl::control::TriangularDecomposition::Vertex &v) const
        {
            std::size_t hash = std::hash<double>()(v.x);
            ompl::hash_combine(hash, v.y);
            return hash;
        }
    };
}

ompl::control::TriangularDecomposition::TriangularDecomposition(const base::RealVectorBounds &bounds,
                                                                std::vector<Polygon> holes,
                                                                std::vector<Polygon> intRegs)
  : Decomposition(2, bounds)
  , holes_(std::move(holes))
  , intRegs_(std::move(intRegs))
  , triAreaPct_(0.005)
  , locator(64, this)
{
    // \todo: Ensure that no two holes overlap and no two regions of interest overlap.
    // Report an error otherwise.
}

ompl::control::TriangularDecomposition::~TriangularDecomposition() = default;

void ompl::control::TriangularDecomposition::setup()
{
    int numTriangles = createTriangles();
    OMPL_INFORM("Created %u triangles", numTriangles);
    buildLocatorGrid();
}

void ompl::control::TriangularDecomposition::addHole(const Polygon &hole)
{
    holes_.push_back(hole);
}

void ompl::control::TriangularDecomposition::addRegionOfInterest(const Polygon &region)
{
    intRegs_.push_back(region);
}

int ompl::control::TriangularDecomposition::getNumHoles() const
{
    return holes_.size();
}

int ompl::control::TriangularDecomposition::getNumRegionsOfInterest() const
{
    return intRegs_.size();
}

const std::vector<ompl::control::TriangularDecomposition::Polygon> &
ompl::control::TriangularDecomposition::getHoles() const
{
    return holes_;
}

const std::vector<ompl::control::TriangularDecomposition::Polygon> &
ompl::control::TriangularDecomposition::getAreasOfInterest() const
{
    return intRegs_;
}

int ompl::control::TriangularDecomposition::getRegionOfInterestAt(int triID) const
{
    return intRegInfo_[triID];
}

double ompl::control::TriangularDecomposition::getRegionVolume(int triID)
{
    Triangle &tri = triangles_[triID];
    if (tri.volume < 0)
    {
        /* This triangle area formula relies on the vertices being
         * stored in counter-clockwise order. */
        tri.volume = 0.5 * ((tri.pts[0].x - tri.pts[2].x) * (tri.pts[1].y - tri.pts[0].y) -
                            (tri.pts[0].x - tri.pts[1].x) * (tri.pts[2].y - tri.pts[0].y));
    }
    return tri.volume;
}

void ompl::control::TriangularDecomposition::getNeighbors(int triID, std::vector<int> &neighbors) const
{
    neighbors = triangles_[triID].neighbors;
}

int ompl::control::TriangularDecomposition::locateRegion(const base::State *s) const
{
    std::vector<double> coord(2);
    project(s, coord);
    const std::vector<int> &gridTriangles = locator.locateTriangles(s);
    int triangle = -1;
    for (int triID : gridTriangles)
    {
        if (triContains(triangles_[triID], coord))
        {
            if (triangle >= 0)
                OMPL_WARN("Decomposition space coordinate (%f,%f) is somehow contained by multiple triangles. \
                    This can happen if the coordinate is located exactly on a triangle segment.\n",
                          coord[0], coord[1]);
            triangle = triID;
        }
    }
    return triangle;
}

void ompl::control::TriangularDecomposition::sampleFromRegion(int triID, RNG &rng, std::vector<double> &coord) const
{
    /* Uniformly sample a point from within a triangle, using the approach discussed in
     * http://math.stackexchange.com/questions/18686/uniform-random-point-in-triangle */
    const Triangle &tri = triangles_[triID];
    coord.resize(2);
    const double r1 = sqrt(rng.uniform01());
    const double r2 = rng.uniform01();
    coord[0] = (1 - r1) * tri.pts[0].x + r1 * (1 - r2) * tri.pts[1].x + r1 * r2 * tri.pts[2].x;
    coord[1] = (1 - r1) * tri.pts[0].y + r1 * (1 - r2) * tri.pts[1].y + r1 * r2 * tri.pts[2].y;
}

void ompl::control::TriangularDecomposition::print(std::ostream &out) const
{
    /* For each triangle, print a line of the form
       N x1 y1 x2 y2 x3 y3 L1 L2 ... -1
       N is the ID of the triangle
       L1 L2 ... is the sequence of all regions of interest to which
       this triangle belongs. */
    for (unsigned int i = 0; i < triangles_.size(); ++i)
    {
        out << i << " ";
        const Triangle &tri = triangles_[i];
        for (int v = 0; v < 3; ++v)
            out << tri.pts[v].x << " " << tri.pts[v].y << " ";
        if (intRegInfo_[i] > -1)
            out << intRegInfo_[i] << " ";
        out << "-1" << std::endl;
    }
}

ompl::control::TriangularDecomposition::Vertex::Vertex(double vx, double vy) : x(vx), y(vy)
{
}

bool ompl::control::TriangularDecomposition::Vertex::operator==(const Vertex &v) const
{
    return x == v.x && y == v.y;
}

int ompl::control::TriangularDecomposition::createTriangles()
{
    /* create a conforming Delaunay triangulation
       where each triangle takes up no more than triAreaPct_ percentage of
       the total area of the decomposition space */
    const base::RealVectorBounds &bounds = getBounds();
    const double maxTriangleArea = bounds.getVolume() * triAreaPct_;
    std::string triswitches = "pDznQA -a" + ompl::toString(maxTriangleArea);
    struct triangulateio in;

    /* Some vertices may be duplicates, such as when an obstacle has a vertex equivalent
       to one at the corner of the bounding box of the decomposition.
       libtriangle does not perform correctly if points are duplicated in the pointlist;
       so, to prevent duplicate vertices, we use a hashmap from Vertex to the index for
       that Vertex in the pointlist. We'll fill the map with Vertex objects,
       and then we'll actually add them to the pointlist. */
    std::unordered_map<Vertex, int> pointIndex;

    // First, add the points from the bounding box
    pointIndex[Vertex(bounds.low[0], bounds.low[1])] = 0;
    pointIndex[Vertex(bounds.high[0], bounds.low[1])] = 1;
    pointIndex[Vertex(bounds.high[0], bounds.high[1])] = 2;
    pointIndex[Vertex(bounds.low[0], bounds.high[1])] = 3;

    /* in.numberofpoints equals the total number of unique vertices.
       in.numberofsegments is slightly different: it equals the total number of given vertices.
       They will both be at least 4, due to the bounding box. */
    in.numberofpoints = 4;
    in.numberofsegments = 4;

    // Run through obstacle vertices in holes_, and tally point and segment counters
    for (auto &p : holes_)
    {
        for (auto &pt : p.pts)
        {
            ++in.numberofsegments;
            /* Only assign an index to this vertex (and tally the point counter)
               if this is a newly discovered vertex. */
            if (pointIndex.find(pt) == pointIndex.end())
                pointIndex[pt] = in.numberofpoints++;
        }
    }

    /* Run through region-of-interest vertices in intRegs_, and tally point and segment counters.
       Here we're following the same logic as above with holes_. */
    for (auto &p : intRegs_)
    {
        for (auto &pt : p.pts)
        {
            ++in.numberofsegments;
            if (pointIndex.find(pt) == pointIndex.end())
                pointIndex[pt] = in.numberofpoints++;
        }
    }

    // in.pointlist is a sequence (x1 y1 x2 y2 ...) of ordered pairs of points
    in.pointlist = (REAL *)malloc(2 * in.numberofpoints * sizeof(REAL));

    // add unique vertices from our map, using their assigned indices
    for (const auto &i : pointIndex)
    {
        const Vertex &v = i.first;
        int index = i.second;
        in.pointlist[2 * index] = v.x;
        in.pointlist[2 * index + 1] = v.y;
    }

    /* in.segmentlist is a sequence (a1 b1 a2 b2 ...) of pairs of indices into
       in.pointlist to designate a segment between the respective points. */
    in.segmentlist = (int *)malloc(2 * in.numberofsegments * sizeof(int));

    // First, add segments for the bounding box
    for (int i = 0; i < 4; ++i)
    {
        in.segmentlist[2 * i] = i;
        in.segmentlist[2 * i + 1] = (i + 1) % 4;
    }

    /* segIndex keeps track of where we are in in.segmentlist,
       as we fill it from multiple sources of data. */
    int segIndex = 4;

    /* Now, add segments for each obstacle in holes_, using our index map
       from before to get the pointlist index for each vertex */
    for (auto &p : holes_)
    {
        for (unsigned int j = 0; j < p.pts.size(); ++j)
        {
            in.segmentlist[2 * segIndex] = pointIndex[p.pts[j]];
            in.segmentlist[2 * segIndex + 1] = pointIndex[p.pts[(j + 1) % p.pts.size()]];
            ++segIndex;
        }
    }

    /* Now, add segments for each region-of-interest in intRegs_,
       using the same logic as before. */
    for (auto &p : intRegs_)
    {
        for (unsigned int j = 0; j < p.pts.size(); ++j)
        {
            in.segmentlist[2 * segIndex] = pointIndex[p.pts[j]];
            in.segmentlist[2 * segIndex + 1] = pointIndex[p.pts[(j + 1) % p.pts.size()]];
            ++segIndex;
        }
    }

    /* libtriangle needs an interior point for each obstacle in holes_.
       For now, we'll assume that each obstacle is convex, and we'll
       generate the interior points ourselves using getPointInPoly. */
    in.numberofholes = holes_.size();
    in.holelist = nullptr;
    if (in.numberofholes > 0)
    {
        /* holelist is a sequence (x1 y1 x2 y2 ...) of ordered pairs of interior points.
           The i^th ordered pair is an interior point of the i^th obstacle in holes_. */
        in.holelist = (REAL *)malloc(2 * in.numberofholes * sizeof(REAL));
        for (int i = 0; i < in.numberofholes; ++i)
        {
            Vertex v = getPointInPoly(holes_[i]);
            in.holelist[2 * i] = v.x;
            in.holelist[2 * i + 1] = v.y;
        }
    }

    /* Similar to above, libtriangle needs an interior point for each
       region-of-interest in intRegs_. We follow the same assumption as before
       that each region-of-interest is convex. */
    in.numberofregions = intRegs_.size();
    in.regionlist = nullptr;
    if (in.numberofregions > 0)
    {
        /* regionlist is a sequence (x1 y1 L1 -1 x2 y2 L2 -1 ...) of ordered triples,
           each ended with -1. The i^th ordered pair (xi,yi,Li) is an interior point
           of the i^th region-of-interest in intRegs_, which is assigned the integer
           label Li. */
        in.regionlist = (REAL *)malloc(4 * in.numberofregions * sizeof(REAL));
        for (unsigned int i = 0; i < intRegs_.size(); ++i)
        {
            Vertex v = getPointInPoly(intRegs_[i]);
            in.regionlist[4 * i] = v.x;
            in.regionlist[4 * i + 1] = v.y;
            // triangles outside of interesting regions get assigned an attribute of zero by default
            // so let's number our attributes from 1 to numProps, then shift it down by 1 when we're done
            in.regionlist[4 * i + 2] = (REAL)(i + 1);
            in.regionlist[4 * i + 3] = -1.;
        }
    }

    // mark remaining input fields as unused
    in.segmentmarkerlist = (int *)nullptr;
    in.numberofpointattributes = 0;
    in.pointattributelist = nullptr;
    in.pointmarkerlist = nullptr;

    // initialize output libtriangle structure, which will hold the results of the triangulation
    struct triangulateio out;
    out.pointlist = (REAL *)nullptr;
    out.pointattributelist = (REAL *)nullptr;
    out.pointmarkerlist = (int *)nullptr;
    out.trianglelist = (int *)nullptr;
    out.triangleattributelist = (REAL *)nullptr;
    out.neighborlist = (int *)nullptr;
    out.segmentlist = (int *)nullptr;
    out.segmentmarkerlist = (int *)nullptr;
    out.edgelist = (int *)nullptr;
    out.edgemarkerlist = (int *)nullptr;
    out.pointlist = (REAL *)nullptr;
    out.pointattributelist = (REAL *)nullptr;
    out.trianglelist = (int *)nullptr;
    out.triangleattributelist = (REAL *)nullptr;

    // call the triangulation routine
    triangulate(const_cast<char *>(triswitches.c_str()), &in, &out, nullptr);

    triangles_.resize(out.numberoftriangles);
    intRegInfo_.resize(out.numberoftriangles);
    for (int i = 0; i < out.numberoftriangles; ++i)
    {
        Triangle &t = triangles_[i];
        for (int j = 0; j < 3; ++j)
        {
            t.pts[j].x = out.pointlist[2 * out.trianglelist[3 * i + j]];
            t.pts[j].y = out.pointlist[2 * out.trianglelist[3 * i + j] + 1];
            if (out.neighborlist[3 * i + j] >= 0)
                t.neighbors.push_back(out.neighborlist[3 * i + j]);
        }
        t.volume = -1.;

        if (in.numberofregions > 0)
        {
            auto attribute = (int)out.triangleattributelist[i];
            /* Shift the region-of-interest ID's down to start from zero. */
            intRegInfo_[i] = (attribute > 0 ? attribute - 1 : -1);
        }
    }

    trifree(in.pointlist);
    trifree(in.segmentlist);
    if (in.numberofholes > 0)
        trifree(in.holelist);
    if (in.numberofregions > 0)
        trifree(in.regionlist);
    trifree(out.pointlist);
    trifree(out.pointattributelist);
    trifree(out.pointmarkerlist);
    trifree(out.trianglelist);
    trifree(out.triangleattributelist);
    trifree(out.neighborlist);
    trifree(out.edgelist);
    trifree(out.edgemarkerlist);
    trifree(out.segmentlist);
    trifree(out.segmentmarkerlist);

    return out.numberoftriangles;
}

void ompl::control::TriangularDecomposition::LocatorGrid::buildTriangleMap(const std::vector<Triangle> &triangles)
{
    regToTriangles_.resize(getNumRegions());
    std::vector<double> bboxLow(2);
    std::vector<double> bboxHigh(2);
    std::vector<int> gridCoord[2];
    for (unsigned int i = 0; i < triangles.size(); ++i)
    {
        /* for Triangle tri, compute the smallest rectangular
         * bounding box that contains tri. */
        const Triangle &tri = triangles[i];
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

        /* Convert the bounding box into grid cell coordinates */

        coordToGridCoord(bboxLow, gridCoord[0]);
        coordToGridCoord(bboxHigh, gridCoord[1]);

        /* Every grid cell within bounding box gets
           tri added to its map entry */
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

void ompl::control::TriangularDecomposition::buildLocatorGrid()
{
    locator.buildTriangleMap(triangles_);
}

bool ompl::control::TriangularDecomposition::triContains(const Triangle &tri, const std::vector<double> &coord)
{
    for (int i = 0; i < 3; ++i)
    {
        /* point (coord[0],coord[1]) needs to be to the left of
           the vector from (ax,ay) to (bx,by) */
        const double ax = tri.pts[i].x;
        const double ay = tri.pts[i].y;
        const double bx = tri.pts[(i + 1) % 3].x;
        const double by = tri.pts[(i + 1) % 3].y;

        // return false if the point is instead to the right of the vector
        if ((coord[0] - ax) * (by - ay) - (bx - ax) * (coord[1] - ay) > 0.)
            return false;
    }
    return true;
}

ompl::control::TriangularDecomposition::Vertex
ompl::control::TriangularDecomposition::getPointInPoly(const Polygon &poly)
{
    Vertex p;
    p.x = 0.;
    p.y = 0.;
    for (auto pt : poly.pts)
    {
        p.x += pt.x;
        p.y += pt.y;
    }
    p.x /= poly.pts.size();
    p.y /= poly.pts.size();
    return p;
}
