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
#include <ompl/multilevel/planners/qmp/QMP.h>
#include <ompl/multilevel/datastructures/Projection.h>

using namespace ompl::base;
using namespace ompl::geometric;

// Input arguments to this binary.
const int numLinks = 10;
const double timeout = 60;
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

namespace ompl
{
  namespace multilevel
  {
    OMPL_CLASS_FORWARD(Projection);
  }
}

class MyProjectionOperator: public ompl::multilevel::Projection
{
  public:
    MyProjectionOperator(StateSpacePtr bundle, StateSpacePtr base, PlanarManipulator *manip):
      Projection(), bundle_(bundle), base_(base), manip_(manip)
    {
    }

    void project( const State *xBundle, State *xBase) const
    {
        std::vector<double> reals;
        bundle_->copyToReals(reals, xBundle);

        Eigen::Affine2d eeFrame;
        manip_->FK(reals, eeFrame);

        std::cout << " " << eeFrame.translation()(0) 
                      << " " << eeFrame.translation()(1) << std::endl;


        double x = eeFrame.translation()(0);
        double y = eeFrame.translation()(1);
        xBase->as<RealVectorStateSpace::StateType>()->values[0] = x;
        xBase->as<RealVectorStateSpace::StateType>()->values[1] = y;
    }

    void lift( const State *xBase, State *xBundle) const
    {
      //Take xBase and return xBundle (e.g. Inverse Kinematics)
    }

  private:
    ompl::base::StateSpacePtr bundle_;
    ompl::base::StateSpacePtr base_;
    PlanarManipulator *manip_;
};



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

    SpaceInformationPtr si = std::make_shared<SpaceInformation>(space);
    si->setStateValidityChecker( 
        std::make_shared<PlanarManipulatorCollisionChecker>( 
          si, manipulator, world));
    si->setStateValidityCheckingResolution(0.001);

    //#########################################################################
    //## Create task space [BASE SPACE]
    //#########################################################################
    ompl::base::StateSpacePtr workspace(new RealVectorStateSpace(3));
    ompl::base::RealVectorBounds boundsWorkspace(3);
    boundsWorkspace.setLow(-2);
    boundsWorkspace.setHigh(+2);
    workspace->as<RealVectorStateSpace>()->setBounds(boundsWorkspace);

    SpaceInformationPtr siTask = std::make_shared<SpaceInformation>(workspace);
    // siTask->setStateValidityChecker( 
    //     std::make_shared<TaskSpaceCollisionChecker>( siTask));
    siTask->setStateValidityChecker( 
        std::make_shared<AllValidStateValidityChecker>( siTask));
    siTask->setStateValidityCheckingResolution(0.001);
    
    //#########################################################################
    //## Create mapping total to base space [PROJECTION]
    //#########################################################################

    ompl::multilevel::ProjectionPtr proj = 
      std::make_shared<MyProjectionOperator>(space, workspace, &manipulator);

    std::vector<SpaceInformationPtr> siVec;
    siVec.push_back(siTask);
    siVec.push_back(si);

    std::vector<ompl::multilevel::ProjectionPtr> projVec;
    projVec.push_back(proj);

    auto planner = std::make_shared<ompl::multilevel::QRRT>(siVec, projVec);

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
    // auto planner = std::make_shared<ompl::multilevel::QRRT>(si);
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
