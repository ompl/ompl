/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

/* Author: Ryan Luna */

#include "PolyWorld.h"

#include <cassert>
#include <fstream>
#include <ompl/util/Console.h>
#include <yaml-cpp/yaml.h>

namespace
{
    // Given a triple of points p,q, and r, classifies the two line segments (pq)
    // and (qr) as a 'left turn', 'right turn' or collinear.
    // Return value:
    // 1 indicates a "left" turn
    // -1 indicates a "right" turn
    // 0 indicates no turn (collinear)
    int turn(Point p, Point q, Point r)
    {
        const double orn = (q.first - p.first) * (r.second - p.second) - (r.first - p.first) * (q.second - p.second);

        if (orn < 0.0)
            return -1;
        if (orn > 0.0)
            return 1;
        return 0;
    }

    // Returns the squared distance between p1 and p2.
    double sqrDistance(Point p1, Point p2)
    {
        const double dx = p2.first - p1.first;
        const double dy = p2.second - p1.second;
        return dx * dx + dy * dy;
    }

}  // namespace

bool cmpDouble(double a, double b, const double eps)
{
    return fabs(a - b) < eps;
}

bool equalPoints(Point p0, Point p1)
{
    return cmpDouble(p0.first, p1.first) && cmpDouble(p0.second, p1.second);
}

ConvexPolygon::ConvexPolygon(const std::vector<Point> &coords)
{
    assert(coords.size() >= 3);

    // Compute convex hull of points
    // Find the left-most point.
    size_t left_idx = 0;
    for (size_t i = 1; i < coords.size(); ++i)
    {
        if (coords[i].first < coords[left_idx].first)
            left_idx = i;
        else if (coords[i].first == coords[left_idx].first)
        {
            if (coords[i].second < coords[left_idx].second)
                left_idx = i;
        }
    }

    // Giftwrap algorithm.
    coordinates_.push_back(coords[left_idx]);
    for (size_t i = 0; i < coordinates_.size(); ++i)
    {
        const Point p = coordinates_[i];
        Point q(p);
        // Find the furthest, 'right-most' vertex and store this in q.
        for (size_t j = 0; j < coords.size(); ++j)
        {
            const int t = turn(p, q, coords[j]);
            if (t == -1 || (t == 0 && sqrDistance(p, coords[j]) > sqrDistance(p, q)))
            {
                q = coords[j];
            }
        }

        if (!equalPoints(q, coordinates_[0]))
            coordinates_.push_back(q);
    }
}

// This algorithm originally came from PNPOLY:
//   http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
// Count the number of times a ray extended horizontally from the y coordinate
// of the point intersects the polygon.
// If it intersects an odd number of times, the point is inside the polygon.
bool ConvexPolygon::inside(Point point) const
{
    size_t i, j;
    bool c = false;
    for (i = 0, j = coordinates_.size() - 1; i < coordinates_.size(); j = i++)
    {
        if (((coordinates_[i].second > point.second) != (coordinates_[j].second > point.second)) &&
            (point.first < (coordinates_[j].first - coordinates_[i].first) * (point.second - coordinates_[i].second) /
                                   (coordinates_[j].second - coordinates_[i].second) +
                               coordinates_[i].first))
        {
            c = !c;
        }
    }
    return c;
}

PolyWorld::PolyWorld(const std::string &worldName, const std::pair<double, double> &xBounds,
                     const std::pair<double, double> &yBounds)
  : worldName_(worldName)
{
    assert(xBounds.first < xBounds.second);
    assert(yBounds.first < yBounds.second);
    bounds_.resize(2);
    bounds_[0] = xBounds;
    bounds_[1] = yBounds;
}

const std::string &PolyWorld::worldName() const
{
    return worldName_;
}

std::pair<double, double> PolyWorld::xBounds() const
{
    assert(bounds_.size() > 0);
    return bounds_[0];
}

std::pair<double, double> PolyWorld::yBounds() const
{
    assert(bounds_.size() > 1);
    return bounds_[1];
}

size_t PolyWorld::numObstacles() const
{
    return obstacles_.size();
}

const std::vector<ConvexPolygon> &PolyWorld::obstacles() const
{
    return obstacles_;
}

const ConvexPolygon &PolyWorld::obstacle(size_t i) const
{
    assert(i < obstacles_.size());
    return obstacles_[i];
}

void PolyWorld::addObstacle(const ConvexPolygon &polygon)
{
    obstacles_.push_back(polygon);
}

bool PolyWorld::outOfBounds(const Point p) const
{
    return p.first <= bounds_[0].first || p.first >= bounds_[0].second || p.second <= bounds_[1].first ||
           p.second >= bounds_[1].second;
}

bool PolyWorld::pointCollisionFree(const Point p) const
{
    bool valid = true;
    for (size_t i = 0; i < obstacles_.size() && valid; ++i)
    {
        valid = !(obstacles_[i].inside(p));
    }
    return valid;
}

void PolyWorld::writeWorld(const char *filename) const
{
    std::ofstream fout;
    fout.open(filename);
    if (!fout)
    {
        OMPL_ERROR("Failed to open %s for writing", filename);
        return;
    }

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << worldName_;

    out << YAML::Key << "bounds";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::BeginMap;
    out << YAML::Key << "lower";
    out << YAML::Value << bounds_[0].first;
    out << YAML::Key << "upper";
    out << YAML::Value << bounds_[0].second;
    out << YAML::EndMap;
    out << YAML::Key << "y";
    out << YAML::BeginMap;
    out << YAML::Key << "lower";
    out << YAML::Value << bounds_[1].first;
    out << YAML::Key << "upper";
    out << YAML::Value << bounds_[1].second;
    out << YAML::EndMap;
    out << YAML::EndMap;

    out << YAML::Key << "obstacles";
    out << YAML::BeginSeq;

    for (size_t i = 0; i < obstacles_.size(); ++i)
    {
        const ConvexPolygon &polygon = obstacles_[i];

        out << YAML::BeginMap;
        out << "polygon";
        out << YAML::BeginSeq;
        for (size_t j = 0; j < polygon.numPoints(); ++j)
        {
            Point p = polygon[j];

            out << YAML::BeginMap;
            out << YAML::Key << "vertex";

            std::stringstream str;
            str << "(" << p.first << "," << p.second << ")";
            out << YAML::Value << str.str();
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    // writing file
    fout << out.c_str();
    fout.close();
}

// // Read the YAML world file
// void PolyWorld::readWorld(const char *worldFile)
// {
//     YAML::Node worldConfig = YAML::LoadFile(worldFile);

//     if (worldConfig["name"])
//         worldName_ = worldConfig["name"].as<std::string>();
//     else
//     {
//         std::cerr << "[Warning] Name not specified in world file.  Giving default name." << std::endl;
//         worldName_ = "MyWorld";
//     }

//     // Reading in bounds
//     bounds_.clear();
//     if (!worldConfig["bounds"])
//     {
//         std::cerr << "[Warning] Bounds for world not specified.  Bounds will be inferred as the minimum bounding
//         box."
//                   << std::endl;
//     }
//     else
//     {
//         YAML::Node boundsConfig = worldConfig["bounds"];
//         YAML::Node xBoundConfig = boundsConfig["x"];
//         YAML::Node yBoundConfig = boundsConfig["y"];

//         if (!xBoundConfig || !yBoundConfig)
//             std::cerr << "[Warning] Malformed bounds entry.  Bounds will be inferred as the minimum bounding box."
//                       << std::endl;
//         else
//         {
//             YAML::Node xLowerConfig = xBoundConfig["lower"];
//             YAML::Node xUpperConfig = xBoundConfig["upper"];

//             if (!xLowerConfig || !xUpperConfig)
//                 std::cerr << "[Warning] Malformed X bounds entry.  Bounds will be inferred as the minimum bounding
//                 box."
//                           << std::endl;
//             else
//             {
//                 double lower = xLowerConfig.as<double>();
//                 double upper = xUpperConfig.as<double>();
//                 bounds_.push_back(std::make_pair(lower, upper));
//             }

//             YAML::Node yLowerConfig = yBoundConfig["lower"];
//             YAML::Node yUpperConfig = yBoundConfig["upper"];

//             if (!yLowerConfig || !yUpperConfig)
//             {
//                 std::cerr << "[Warning] Malformed Y bounds entry.  Bounds will be inferred as the minimum bounding
//                 box."
//                           << std::endl;
//                 bounds_.clear();
//             }
//             else
//             {
//                 double lower = yLowerConfig.as<double>();
//                 double upper = yUpperConfig.as<double>();
//                 bounds_.push_back(std::make_pair(lower, upper));
//             }
//         }
//     }

//     // Reading in obstacles
//     YAML::Node obstacleConfig = worldConfig["obstacles"];
//     if (!obstacleConfig)
//         std::cerr << "[Warning] No obstacles specified!" << std::endl;
//     else
//     {
//         if (!obstacleConfig.IsSequence())
//             throw std::runtime_error("Expected a sequence of geometries for obstacle tag");

//         for (size_t i = 0; i < obstacleConfig.size(); i++)
//         {
//             YAML::Node obstacleNode = obstacleConfig[i];
//             Geometry::Geometry *obs = readGeometry(obstacleNode);

//             if (obs)
//                 obstacles_.push_back(obs);
//         }
//     }
// }

// Geometry::Geometry *PolyWorld::readGeometry(const YAML::Node &node) const
// {
//     if (node["polygon"])
//     {
//         return readPolygon(node["polygon"]);
//     }
//     else if (node["rectangle"])
//     {
//         return readRectangle(node["rectangle"]);
//     }

//     std::cerr << "[ERROR] Unknown geometry node specified.  Skipping" << std::endl;
//     return nullptr;
// }

// Geometry::ConvexPolygon *PolyWorld::readPolygon(const YAML::Node &polygonNode) const
// {
//     if (!polygonNode.IsSequence())
//     {
//         std::cerr << "[ERROR] Expected a sequence of vertices for the polygon" << std::endl;
//         return nullptr;
//     }
//     if (polygonNode.size() < 3)
//     {
//         std::cerr << "[ERROR] Polygons must have at least three vertices" << std::endl;
//         return nullptr;
//     }

//     std::vector<Geometry::Point> points;
//     for (size_t i = 0; i < polygonNode.size(); i++)
//     {
//         YAML::Node vertexNode = polygonNode[i]["vertex"];

//         Geometry::Point p = readCoordinate(vertexNode.as<std::string>());
//         points.push_back(p);
//     }

//     Geometry::ConvexPolygon *polygon = new Geometry::ConvexPolygon();
//     polygon->initialize(points);
//     return polygon;
// }

// Geometry::ConvexPolygon *PolyWorld::readRectangle(const YAML::Node &rectNode) const
// {
//     std::string upperleft = rectNode["upperleft"].as<std::string>();
//     double width = rectNode["width"].as<double>();
//     double height = rectNode["height"].as<double>();

//     Geometry::Point coord = readCoordinate(upperleft);

//     std::vector<Geometry::Point> points;
//     points.push_back(coord);
//     points.push_back(Geometry::Point(coord.first, coord.second - height));
//     points.push_back(Geometry::Point(coord.first + width, coord.second - height));
//     points.push_back(Geometry::Point(coord.first + width, coord.second));

//     Geometry::ConvexPolygon *polygon = new Geometry::ConvexPolygon();
//     polygon->initialize(points);
//     return polygon;
// }

// Geometry::Point PolyWorld::readCoordinate(const std::string &str) const
// {
//     std::string coord = str;
//     // Remove all spaces
//     boost::erase_all(coord, " ");

//     // Split at comma
//     std::vector<std::string> strs;
//     boost::algorithm::split(strs, coord, boost::is_any_of(","));

//     Geometry::Point p;

//     // Only expecting two coordinates
//     if (strs.size() != 2)
//     {
//         std::cerr << "[Warning] Expected 2D coordinate, got " << str << std::endl;
//         return p;
//     }

//     // Casting coordinates to doubles.  Parenthesis are stripped off, if they are there
//     p.first = ompl::stod(strs[0][0] == '(' ? strs[0].substr(1) : strs[0]);
//     p.second = ompl::stod(strs[1][strs[1].size() - 1] == ')' ? strs[1].substr(0, strs[1].size() - 1) : strs[1]);

//     return p;
// }
