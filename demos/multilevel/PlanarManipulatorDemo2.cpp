//This is basically just a simplified version of Ryan Luna's Demo, used for
//testing purposes of the multilevel planning framework
//
// TODO
//  [ ] How do the decompositions here by Ryan differ from multilevel? Can we
//  reuse anything? 
//  [ ] How does it all compare to syclop decompositions?

#include <fstream>

#include "boost/program_options.hpp"
#include "../PlanarManipulator/PolyWorld.h"
#include "../PlanarManipulator/PlanarManipulatorPolyWorld.h"
#include "../PlanarManipulator/PlanarManipulator.h"
#include "../PlanarManipulator/PlanarManipulatorStateSpace.h"
#include "../PlanarManipulator/PlanarManipulatorStateValidityChecker.h"
#include "../PlanarManipulator/PlanarManipulatorIKGoal.h"

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>

using namespace ompl::base;
using namespace ompl::geometric;

// Input arguments to this binary.
int numLinks = 10;
double timeout = 60;
std::string problemName = "corridor";
bool viz = false;

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
      << " " << basePose.translation()(1) << std::endl;

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

class MyProjectionOperator: public BundleSpaceProjection
{
  MyProjectionOperator(StateSpacePtr bundle, StateSpacePtr base):
    BundleSpaceProjection(bundle, base)
  {
  }
  void project( const State *xBundle, State *xBase) const
  {
    //Take xBundle and return xBase (e.g. Forward Kinematics)
  }
  void lift( const State *xBase, State *xBundle) const
  {
    //Take xBase and return xBundle (e.g. Inverse Kinematics)
  }
}



int main()
{
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

    Eigen::Affine2d baseFrame;
    Eigen::Affine2d goalFrame;

    PlanarManipulator manipulator = PlanarManipulator(numLinks, 1.0/numLinks);
    PolyWorld world = createCorridorProblem(numLinks, baseFrame, goalFrame);
    // PolyWorld world = createConstrictedProblem(numLinks, baseFrame, goalFrame);

    //#########################################################################
    //## Create robot joint configuration space [TOTAL SPACE]
    //#########################################################################
    ompl::base::StateSpacePtr space(new PlanarManipulatorStateSpace(numLinks));
    ompl::base::RealVectorBounds bounds(numLinks);
    bounds.setLow(-M_PI);
    bounds.setHigh(M_PI);
    space->as<PlanarManipulatorStateSpace>()->setBounds(bounds);
    manipulator.setBounds(bounds.low, bounds.high);

    SpaceInformationPtr siJointSpace = std::make_shared<SpaceInformation>(space);
    siJointSpace->setStateValidityChecker( 
        std::make_shared<PlanarManipulatorCollisionChecker>( 
          siJointSpace, manipulator, world));
    siJointSpace->setStateValidityCheckingResolution(0.001);

    //#########################################################################
    //## Create task space [BASE SPACE]
    //#########################################################################
    // ompl::base::StateSpacePtr workspace(new RealVectorStateSpace(3));
    // ompl::base::RealVectorBounds bounds(3);
    // bounds.setLow(-2);
    // bounds.setHigh(+2);
    // workspace->as<RealVectorStateSpace>()->setBounds(bounds);

    // SpaceInformationPtr siTask = std::make_shared<SpaceInformation>(workspace);
    // siTask->setStateValidityChecker( 
    //     std::make_shared<TaskSpaceCollisionChecker>( siTask));
    // siTask->setStateValidityCheckingResolution(0.001);
    
    //#########################################################################
    //## Create mapping total to base space [PROJECTION]
    //#########################################################################

    Projection proj = new MyProjectionOperator(totalSpace, baseSpace);

    std::vector<SpaceInformationPtr> siVec;
    siVec.push_back(siTask);
    siVec.push_back(siJointSpace);

    std::vector<BundleSpaceProjectionPtr> projVec;
    projVec.push_back(proj);

    auto planner = std::make_shared<ompl::multilevel::QRRT>(siVec, projVec);

    // FiberBundle bundle(totalSpace, baseSpace);
    // FiberBundle bundle(totalSpace, baseSpace, projection);
    // FiberBundle bundle(std::vector<spaces>, std::vector<projections>);

    //#########################################################################
    //## Create robot joint configuration space
    //#########################################################################
    // Set the start and goal.
    ompl::base::State *start = si->allocState();
    double *start_angles = start->as<PlanarManipulatorStateSpace::StateType>()->values;

    for (int i = 0; i < numLinks; ++i)
    {
        start_angles[i] = 1e-7;
    }

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(si);
    pdef->addStartState(start);

    si->freeState(start);

    ompl::base::GoalPtr goal(new PlanarManipulatorIKGoal(si, goalFrame, &manipulator, false));
    goal->as<PlanarManipulatorIKGoal>()->setThreshold(1e-3);
    pdef->setGoal(goal);

    // auto planner = std::make_shared<ompl::geometric::RRTConnect>(si);
    auto planner = std::make_shared<ompl::multilevel::QRRT>(si);
    // auto planner = std::make_shared<ompl::geometric::RRT>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    PlannerStatus status = planner->Planner::solve(timeout);

    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
        status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        PathPtr path = pdef->getSolutionPath();
        PathGeometric &pgeo = *static_cast<PathGeometric*>(path.get());
        OMPL_INFORM("Solution path has %d states", pgeo.getStateCount());

        pgeo.interpolate(250);
        WriteVisualization(manipulator, world, pgeo);
    }
}
