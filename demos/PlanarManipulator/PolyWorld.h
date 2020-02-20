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

#ifndef POLYWORLD_H
#define POLYWORLD_H

#include <string>
#include <yaml-cpp/yaml.h>
#include "Geometry.h"

// A representation of a world composed of polygonal obstacles
// The world is read in using a proprietary YAML format
class PolyWorld
{
public:
    // Empty world
    PolyWorld(const std::string &worldName, const std::pair<double, double> &xBounds,
              const std::pair<double, double> &yBounds);
    // Read world from file
    PolyWorld(const char *worldFile);
    PolyWorld()
    {
    }
    virtual ~PolyWorld();

    void clear();

    void initialize(const std::string &worldName, const std::pair<double, double> &xBounds,
                    const std::pair<double, double> &yBounds);

    const std::string &getWorldName() const;
    std::pair<double, double> getXBounds() const;
    std::pair<double, double> getYBounds() const;

    unsigned int getNumObstacles() const;
    const std::vector<Geometry::Geometry *> &getObstacles() const;
    const Geometry::Geometry *getObstacle(unsigned int i) const;

    void addObstacle(Geometry::Geometry *geom);

    bool outOfBounds(const Geometry::Point &p) const;

    // Check the point for collision against all obstacles, including "unknown" obstacles
    bool pointCollisionFree(const Geometry::Point &p) const;

    // Write the world to the given filename in YAML format
    void writeWorld(const char *filename) const;

protected:
    void readWorld(const char *worldFile);
    Geometry::Geometry *readGeometry(const YAML::Node &node) const;
    Geometry::ConvexPolygon *readPolygon(const YAML::Node &polygonNode) const;
    Geometry::ConvexPolygon *readRectangle(const YAML::Node &rectNode) const;
    Geometry::Point readCoordinate(const std::string &string) const;

    std::string worldName_;
    std::vector<std::pair<double, double>> bounds_;

    std::vector<Geometry::Geometry *> obstacles_;
};

#endif
