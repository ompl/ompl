/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

// This is basically just a simplified version of Ryan Luna's Demo, used for
// testing purposes of the multilevel planning framework

#include <fstream>

#include "boost/program_options.hpp"
#include "../PlanarManipulator/PolyWorld.h"
#include "../PlanarManipulator/PlanarManipulatorPolyWorld.h"
#include "../PlanarManipulator/PlanarManipulator.h"
#include "../PlanarManipulator/PlanarManipulatorStateSpace.h"
#include "../PlanarManipulator/PlanarManipulatorStateValidityChecker.h"
#include "../PlanarManipulator/PlanarManipulatorIKGoal.h"

#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/SE2_R2.h>

using namespace ompl::base;
using namespace ompl::geometric;

const int numLinks = 9;
const double timeout = 10;
const int xySlices = std::max(2, numLinks / 3);
const std::string problemName = "corridor";
const bool viz = false;

void WriteVisualization(const PlanarManipulator &manipulator, const PolyWorld *world, const PathGeometric &path)
{
    const int numLinks = manipulator.getNumLinks();

    const char *world_file = "world.yaml";
    OMPL_INFORM("Writing world to %s", world_file);
    world->writeWorld(world_file);

    const double linkLength = 1.0 / numLinks;
    const Eigen::Affine2d &basePose = manipulator.getBaseFrame();

    const char *path_file = "manipulator_path.txt";
    OMPL_INFORM("Writing path to %s", path_file);
    std::ofstream fout;

    fout.open(path_file);
    fout << numLinks << " " << linkLength << " " << basePose.translation()(0) << " " << basePose.translation()(1) << " "
         << xySlices << std::endl;

    // Write each state on the interpolated path.
    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const double *angles = path.getState(i)->as<PlanarManipulatorStateSpace::StateType>()->values;
        for (size_t j = 0; j < manipulator.getNumLinks(); ++j)
            fout << angles[j] << " ";
        fout << std::endl;
    }
    fout.close();
}

class R2CollisionChecker : public ompl::base::StateValidityChecker
{
public:
    R2CollisionChecker(const ompl::base::SpaceInformationPtr &si, const PolyWorld *world)
      : ompl::base::StateValidityChecker(si), world_(world)
    {
    }

    ~R2CollisionChecker() = default;

    virtual bool isValid(const ompl::base::State *state) const
    {
        const double *angles = state->as<RealVectorStateSpace::StateType>()->values;
        std::vector<Point> coordinates;
        coordinates.push_back({angles[0], angles[1]});

        //(1) check out of bounds
        if (world_->outOfBounds(coordinates[0]))
        {
            return false;
        }

        // Check each coordinate for obstacle intersection.
        for (size_t j = 0; j < world_->numObstacles(); ++j)
            if (world_->obstacle(j).inside(coordinates[0]))
                return false;

        return true;
    }

private:
    const PolyWorld *world_;
};
class SE2CollisionChecker : public ompl::base::StateValidityChecker
{
public:
    SE2CollisionChecker(const ompl::base::SpaceInformationPtr &si, const PolyWorld *world)
      : ompl::base::StateValidityChecker(si), world_(world)
    {
    }

    ~SE2CollisionChecker() = default;

    virtual bool isValid(const ompl::base::State *state) const
    {
        const double x = state->as<SE2StateSpace::StateType>()->getX();
        const double y = state->as<SE2StateSpace::StateType>()->getY();

        std::vector<Point> coordinates;
        coordinates.push_back({x, y});

        //(1) check out of bounds
        if (world_->outOfBounds(coordinates[0]))
            return false;

        // Check each coordinate for obstacle intersection.
        for (size_t j = 0; j < world_->numObstacles(); ++j)
            if (world_->obstacle(j).inside(coordinates[0]))
                return false;

        return true;
    }

private:
    const PolyWorld *world_;
};
