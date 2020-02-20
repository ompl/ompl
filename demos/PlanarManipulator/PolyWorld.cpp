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

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <ompl/util/String.h>
#include "PolyWorld.h"

PolyWorld::PolyWorld(const std::string &worldName, const std::pair<double, double> &xBounds,
                     const std::pair<double, double> &yBounds)
  : worldName_(worldName)
{
    bounds_.resize(2);
    bounds_[0] = xBounds;
    bounds_[1] = yBounds;
}

PolyWorld::PolyWorld(const char *worldFile)
{
    readWorld(worldFile);
}

PolyWorld::~PolyWorld()
{
    clear();
}

void PolyWorld::clear()
{
    for (auto &obstacle : obstacles_)
        delete obstacle;
    obstacles_.clear();
    bounds_.clear();
    worldName_.clear();
}

void PolyWorld::initialize(const std::string &worldName, const std::pair<double, double> &xBounds,
                           const std::pair<double, double> &yBounds)
{
    bounds_.resize(2);
    bounds_[0] = xBounds;
    bounds_[1] = yBounds;
    worldName_ = worldName;
}

const std::string &PolyWorld::getWorldName() const
{
    return worldName_;
}

std::pair<double, double> PolyWorld::getXBounds() const
{
    assert(bounds_.size() > 0);
    return bounds_[0];
}

std::pair<double, double> PolyWorld::getYBounds() const
{
    assert(bounds_.size() > 1);
    return bounds_[1];
}

unsigned int PolyWorld::getNumObstacles() const
{
    return obstacles_.size();
}

const std::vector<Geometry::Geometry *> &PolyWorld::getObstacles() const
{
    return obstacles_;
}

const Geometry::Geometry *PolyWorld::getObstacle(unsigned int i) const
{
    assert(i < obstacles_.size());
    return obstacles_[i];
}

void PolyWorld::addObstacle(Geometry::Geometry *geom)
{
    obstacles_.push_back(geom);
}

bool PolyWorld::outOfBounds(const Geometry::Point &p) const
{
    if (p.first <= bounds_[0].first || p.first >= bounds_[0].second || p.second <= bounds_[1].first ||
        p.second >= bounds_[1].second)
        return true;
    return false;
}

bool PolyWorld::pointCollisionFree(const Geometry::Point &p) const
{
    bool valid = true;
    for (size_t i = 0; i < obstacles_.size() && valid; ++i)
        valid = !(obstacles_[i]->inside(p));

    return valid;
}

void PolyWorld::writeWorld(const char *filename) const
{
    std::ofstream fout;
    fout.open(filename);
    if (!fout)
    {
        std::cerr << "Failed to open " << filename << " for writing" << std::endl;
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
        const Geometry::ConvexPolygon *obs = dynamic_cast<const Geometry::ConvexPolygon *>(obstacles_[i]);
        if (!obs)
        {
            std::cerr << "PolyWorld::writeWorld - I only know how to write convex polygons" << std::endl;
            continue;
        }

        out << YAML::BeginMap;
        out << "polygon";
        out << YAML::BeginSeq;
        for (size_t j = 0; j < obs->numVertices(); ++j)
        {
            Geometry::Point p = (*obs)[j];

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

// Read the YAML world file
void PolyWorld::readWorld(const char *worldFile)
{
    YAML::Node worldConfig = YAML::LoadFile(worldFile);

    if (worldConfig["name"])
        worldName_ = worldConfig["name"].as<std::string>();
    else
    {
        std::cerr << "[Warning] Name not specified in world file.  Giving default name." << std::endl;
        worldName_ = "MyWorld";
    }

    // Reading in bounds
    bounds_.clear();
    if (!worldConfig["bounds"])
    {
        std::cerr << "[Warning] Bounds for world not specified.  Bounds will be inferred as the minimum bounding box."
                  << std::endl;
    }
    else
    {
        YAML::Node boundsConfig = worldConfig["bounds"];
        YAML::Node xBoundConfig = boundsConfig["x"];
        YAML::Node yBoundConfig = boundsConfig["y"];

        if (!xBoundConfig || !yBoundConfig)
            std::cerr << "[Warning] Malformed bounds entry.  Bounds will be inferred as the minimum bounding box."
                      << std::endl;
        else
        {
            YAML::Node xLowerConfig = xBoundConfig["lower"];
            YAML::Node xUpperConfig = xBoundConfig["upper"];

            if (!xLowerConfig || !xUpperConfig)
                std::cerr << "[Warning] Malformed X bounds entry.  Bounds will be inferred as the minimum bounding box."
                          << std::endl;
            else
            {
                double lower = xLowerConfig.as<double>();
                double upper = xUpperConfig.as<double>();
                bounds_.push_back(std::make_pair(lower, upper));
            }

            YAML::Node yLowerConfig = yBoundConfig["lower"];
            YAML::Node yUpperConfig = yBoundConfig["upper"];

            if (!yLowerConfig || !yUpperConfig)
            {
                std::cerr << "[Warning] Malformed Y bounds entry.  Bounds will be inferred as the minimum bounding box."
                          << std::endl;
                bounds_.clear();
            }
            else
            {
                double lower = yLowerConfig.as<double>();
                double upper = yUpperConfig.as<double>();
                bounds_.push_back(std::make_pair(lower, upper));
            }
        }
    }

    // Reading in obstacles
    YAML::Node obstacleConfig = worldConfig["obstacles"];
    if (!obstacleConfig)
        std::cerr << "[Warning] No obstacles specified!" << std::endl;
    else
    {
        if (!obstacleConfig.IsSequence())
            throw std::runtime_error("Expected a sequence of geometries for obstacle tag");

        for (size_t i = 0; i < obstacleConfig.size(); i++)
        {
            YAML::Node obstacleNode = obstacleConfig[i];
            Geometry::Geometry *obs = readGeometry(obstacleNode);

            if (obs)
                obstacles_.push_back(obs);
        }
    }
}

Geometry::Geometry *PolyWorld::readGeometry(const YAML::Node &node) const
{
    if (node["polygon"])
    {
        return readPolygon(node["polygon"]);
    }
    else if (node["rectangle"])
    {
        return readRectangle(node["rectangle"]);
    }

    std::cerr << "[ERROR] Unknown geometry node specified.  Skipping" << std::endl;
    return nullptr;
}

Geometry::ConvexPolygon *PolyWorld::readPolygon(const YAML::Node &polygonNode) const
{
    if (!polygonNode.IsSequence())
    {
        std::cerr << "[ERROR] Expected a sequence of vertices for the polygon" << std::endl;
        return nullptr;
    }
    if (polygonNode.size() < 3)
    {
        std::cerr << "[ERROR] Polygons must have at least three vertices" << std::endl;
        return nullptr;
    }

    std::vector<Geometry::Point> points;
    for (size_t i = 0; i < polygonNode.size(); i++)
    {
        YAML::Node vertexNode = polygonNode[i]["vertex"];

        Geometry::Point p = readCoordinate(vertexNode.as<std::string>());
        points.push_back(p);
    }

    Geometry::ConvexPolygon *polygon = new Geometry::ConvexPolygon();
    polygon->initialize(points);
    return polygon;
}

Geometry::ConvexPolygon *PolyWorld::readRectangle(const YAML::Node &rectNode) const
{
    std::string upperleft = rectNode["upperleft"].as<std::string>();
    double width = rectNode["width"].as<double>();
    double height = rectNode["height"].as<double>();

    Geometry::Point coord = readCoordinate(upperleft);

    std::vector<Geometry::Point> points;
    points.push_back(coord);
    points.push_back(Geometry::Point(coord.first, coord.second - height));
    points.push_back(Geometry::Point(coord.first + width, coord.second - height));
    points.push_back(Geometry::Point(coord.first + width, coord.second));

    Geometry::ConvexPolygon *polygon = new Geometry::ConvexPolygon();
    polygon->initialize(points);
    return polygon;
}

Geometry::Point PolyWorld::readCoordinate(const std::string &str) const
{
    std::string coord = str;
    // Remove all spaces
    boost::erase_all(coord, " ");

    // Split at comma
    std::vector<std::string> strs;
    boost::algorithm::split(strs, coord, boost::is_any_of(","));

    Geometry::Point p;

    // Only expecting two coordinates
    if (strs.size() != 2)
    {
        std::cerr << "[Warning] Expected 2D coordinate, got " << str << std::endl;
        return p;
    }

    // Casting coordinates to doubles.  Parenthesis are stripped off, if they are there
    p.first = ompl::stod(strs[0][0] == '(' ? strs[0].substr(1) : strs[0]);
    p.second = ompl::stod(strs[1][strs[1].size() - 1] == ')' ? strs[1].substr(0, strs[1].size() - 1) : strs[1]);

    return p;
}
