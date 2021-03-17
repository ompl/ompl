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

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>
#include <ompl/multilevel/planners/qmp/QMP.h>

#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/SE2_R2.h>

using namespace ompl::base;
using namespace ompl::geometric;

// Input arguments to this binary.
const int numLinks = 10;
const double timeout = 3;
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

class ProjectionJointSpaceToSE2: public ompl::multilevel::Projection
{
  public:
    ProjectionJointSpaceToSE2(StateSpacePtr bundle, StateSpacePtr base, PlanarManipulator *manip):
      Projection(bundle, base), manip_(manip)
    {
    }

    void project( const State *xBundle, State *xBase) const
    {
        std::vector<double> reals;
        getBundle()->copyToReals(reals, xBundle);

        Eigen::Affine2d eeFrame;
        manip_->FK(reals, eeFrame);

        double x = eeFrame.translation()(0);
        double y = eeFrame.translation()(1);
        double yaw = acos(eeFrame.matrix()(0, 0));

        xBase->as<SE2StateSpace::StateType>()->setXY(x,y);
        xBase->as<SE2StateSpace::StateType>()->setYaw(yaw);

        std::cout << "---Project---" << std::endl;
        getBundle()->printState(xBundle);
        getBase()->printState(xBase);
    }

    void lift( const State *xBase, State *xBundle) const
    {
        std::vector<double> reals;
        getBase()->copyToReals(reals, xBase);

        //to Eigen
        Eigen::Affine2d eeFrame;
        eeFrame.translation()(0) = reals.at(0);
        eeFrame.translation()(1) = reals.at(1);
        eeFrame.rotate(reals.at(2));

        std::vector<double> solution;
        manip_->IK(solution, eeFrame);

        double *angles = xBundle->as<PlanarManipulatorStateSpace::StateType>()->values;
        for(uint k = 0; k < solution.size(); k++)
        {
            angles[k] = solution.at(k);
        }
        // std::cout << "---Lift---" << std::endl;
        // getBase()->printState(xBase);
        // getBundle()->printState(xBundle);
    }

  private:
    PlanarManipulator *manip_;
};

int main()
{
    // ompl::msg::setLogLevel(ompl::msg::LOG_INFO);

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
    //## Create task space [SE2 BASE SPACE]
    //#########################################################################
    ompl::base::StateSpacePtr spaceSE2(new SE2StateSpace());
    ompl::base::RealVectorBounds boundsWorkspace(2);
    boundsWorkspace.setLow(-2);
    boundsWorkspace.setHigh(+2);
    spaceSE2->as<SE2StateSpace>()->setBounds(boundsWorkspace);

    SpaceInformationPtr siSE2 = std::make_shared<SpaceInformation>(spaceSE2);
    siSE2->setStateValidityChecker( 
        std::make_shared<AllValidStateValidityChecker>(siSE2));
    siSE2->setStateValidityCheckingResolution(0.001);

    //#########################################################################
    //## Create task space [R2 BASE SPACE]
    //#########################################################################
    ompl::base::StateSpacePtr spaceR2(new RealVectorStateSpace(2));
    ompl::base::RealVectorBounds boundsR2(2);
    boundsR2.setLow(-2);
    boundsR2.setHigh(+2);
    spaceR2->as<RealVectorStateSpace>()->setBounds(boundsR2);

    SpaceInformationPtr siR2 = std::make_shared<SpaceInformation>(spaceR2);
    siR2->setStateValidityChecker( 
        std::make_shared<AllValidStateValidityChecker>(siR2));
    siR2->setStateValidityCheckingResolution(0.001);
    
    //#########################################################################
    //## Create mapping total to base space [PROJECTION]
    //#########################################################################
    ompl::multilevel::ProjectionPtr projAB = 
      std::make_shared<ProjectionJointSpaceToSE2>(space, spaceSE2, &manipulator);

    ompl::multilevel::ProjectionPtr projBC = 
      std::make_shared<ompl::multilevel::Projection_SE2_R2>(spaceSE2, spaceR2);

    std::static_pointer_cast<ompl::multilevel::FiberedProjection>(projBC)->makeFiberSpace();

    //#########################################################################
    //## Put it all together
    //#########################################################################
    std::vector<SpaceInformationPtr> siVec;
    std::vector<ompl::multilevel::ProjectionPtr> projVec;

    // siVec.push_back(siR2); //Base Space R2
    // projVec.push_back(projBC); //Projection R2 to SE2
    siVec.push_back(siSE2); //Base Space SE2
    projVec.push_back(projAB); //Projection SE2 to X
    siVec.push_back(si); //State Space X

    auto planner = std::make_shared<ompl::multilevel::QRRT>(siVec, projVec);

    //#########################################################################
    //## Set start state
    //#########################################################################
    ompl::base::State *start = si->allocState();
    double *start_angles = start->as<PlanarManipulatorStateSpace::StateType>()->values;

    for (int i = 0; i < numLinks; ++i)
    {
        start_angles[i] = 1e-7;
    }


    //#########################################################################
    //## Set goal state
    //#########################################################################
    // 0.346324 0.0828153 2.96842 -2.17559 -0.718962 0.16532 -0.228314 0.172762 0.0471638 0.341137 
    ompl::base::State *goal = si->allocState();

    std::vector<double> goalJoints;
    manipulator.IK(goalJoints, goalFrame);

    double *goal_angles = goal->as<PlanarManipulatorStateSpace::StateType>()->values;
    goal_angles[0] = 0.346324;
    goal_angles[1] = 0.0828153;
    goal_angles[2] =2.96842;
    goal_angles[3] =-2.17559;
    goal_angles[4] =-0.718962;
    goal_angles[5] =0.16532;
    goal_angles[6] =-0.228314;
    goal_angles[7] =0.172762;
    goal_angles[8] =0.0471638;
    goal_angles[9] =0.341137;
    //0.346324 0.0828153 2.96842 -2.17559 -0.718962 0.16532 -0.228314 0.172762 0.0471638 0.341137 


    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(si);
    pdef->addStartState(start);
    pdef->setGoalState(goal, 1e-3);

    si->freeState(start);
    si->freeState(goal);

    //#########################################################################
    //## Invoke planner
    //#########################################################################
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
