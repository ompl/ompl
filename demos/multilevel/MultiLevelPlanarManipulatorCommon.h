//This is basically just a simplified version of Ryan Luna's Demo, used for
//testing purposes of the multilevel planning framework

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

const int numLinks = 10;
const double timeout = 10;
const int xySlices = std::max(2, numLinks / 3);
const std::string problemName = "corridor";
const bool viz = false;

void WriteVisualization(
    const PlanarManipulator &manipulator, 
    const PolyWorld &world, 
    const ompl::geometric::PathGeometric &path)
{
    const int numLinks = manipulator.getNumLinks();

    const char *world_file = "world.yaml";
    OMPL_INFORM("Writing world to %s", world_file);
    world.writeWorld(world_file);

    const double linkLength = 1.0 / numLinks;
    const Eigen::Affine2d &basePose = manipulator.getBaseFrame();

    const char *path_file = "manipulator_path.txt";
    OMPL_INFORM("Writing path to %s", path_file);
    std::ofstream fout;

    fout.open(path_file);
    fout << numLinks << " " << linkLength 
      << " " << basePose.translation()(0) 
      << " " << basePose.translation()(1) << " " << xySlices << std::endl;

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
    R2CollisionChecker(const ompl::base::SpaceInformationPtr si, const PolyWorld &world)
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
        if (world_.outOfBounds(coordinates[0])) return false;

        // Check each coordinate for obstacle intersection.
        for (size_t j = 0; j < world_.numObstacles(); ++j)
            if (world_.obstacle(j).inside(coordinates[0]))
                return false;

        return true;
    }
private:
    const PolyWorld &world_;
};
class SE2CollisionChecker : public ompl::base::StateValidityChecker
{
public:
    SE2CollisionChecker(const ompl::base::SpaceInformationPtr si, const PolyWorld &world)
      : ompl::base::StateValidityChecker(si), world_(world)
    {
    }

    ~SE2CollisionChecker() = default;

    virtual bool isValid(const ompl::base::State *state) const
    {
        const double x = state->as<SE2StateSpace::StateType>()->getX();
        const double y = state->as<SE2StateSpace::StateType>()->getY();

        std::vector<Point> coordinates;
        coordinates.push_back({x,y});

        //(1) check out of bounds
        if (world_.outOfBounds(coordinates[0])) return false;

        // Check each coordinate for obstacle intersection.
        for (size_t j = 0; j < world_.numObstacles(); ++j)
            if (world_.obstacle(j).inside(coordinates[0]))
                return false;

        return true;
    }
private:
    const PolyWorld &world_;
};
