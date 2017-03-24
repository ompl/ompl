#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/extensions/triangle/TriangularDecomposition.h>
#include <iostream>

namespace ob = ompl::base;
namespace oc = ompl::control;

// a decomposition is only needed for SyclopRRT and SyclopEST
class MyTriangularDecomposition : public oc::TriangularDecomposition
{
public:
    MyTriangularDecomposition(const ob::RealVectorBounds& bounds)
        : oc::TriangularDecomposition(bounds, createObstacles())
    {
        setup();
    }
    void project(const ob::State* s, std::vector<double>& coord) const override
    {
        coord.resize(2);
        coord[0] = s->as<ob::SE2StateSpace::StateType>()->getX();
        coord[1] = s->as<ob::SE2StateSpace::StateType>()->getY();
    }

    void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
    {
        sampler->sampleUniform(s);
        s->as<ob::SE2StateSpace::StateType>()->setXY(coord[0], coord[1]);
    }

    std::vector<Polygon> createObstacles()
    {
        std::vector<Polygon> obst;
        Triangle tri;
        tri.pts[0].x = -0.5;
        tri.pts[0].y = 0.75;
        tri.pts[1].x = -0.75;
        tri.pts[1].y = 0.68;
        tri.pts[2].x = -0.5;
        tri.pts[2].y = 0.5;
        obst.push_back(tri);

        Polygon rect(4);
        rect.pts[0].x = 0.;
        rect.pts[0].y = 0.5;
        rect.pts[1].x = -0.3;
        rect.pts[1].y = 0.;
        rect.pts[2].x = 0.;
        rect.pts[2].y = -0.5;
        rect.pts[3].x = 0.6;
        rect.pts[3].y = 0.6;
        obst.push_back(rect);

        return obst;
    }
};

bool triContains(double x, double y, double ax, double ay, double bx, double by, double cx, double cy)
{
    if ((x-ax)*(by-ay) - (bx-ax)*(y-ay) > 0.)
        return false;
    if ((x-bx)*(cy-by) - (cx-bx)*(y-by) > 0.)
        return false;
    if ((x-cx)*(ay-cy) - (ax-cx)*(y-cy) > 0.)
        return false;
    return true;
}


bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    // check validity of state defined by pos & rot
    double x = se2state->getX();
    double y = se2state->getY();
    return si->satisfiesBounds(state) && !triContains(x,y, -0.5,0.75,-0.75,0.68,-0.5,0.5)
        && !triContains(x,y, 0,0.5,-0.3,0,0,-0.5)
        && !triContains(x,y,0,-0.5,0.6,0.6,0,0.5);
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);
    const auto *rctrl = control->as<oc::RealVectorControlSpace::ControlType>();

    result->as<ob::SE2StateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] =
        (*pos)[0] + (*rctrl)[0] * duration * cos(rot->value);

    result->as<ob::SE2StateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] =
        (*pos)[1] + (*rctrl)[0] * duration * sin(rot->value);

    result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1)->value =
        rot->value + (*rctrl)[1];
}


void planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set the state propagation routine
    ss.setStatePropagator(propagate);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State *state) { return isStateValid(si, state); });

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-0.5);
    start->setY(0.0);
    start->setYaw(0.0);

    ob::ScopedState<ob::SE2StateSpace> goal(start);
    goal->setX(0.5);

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);

    auto td(std::make_shared<MyTriangularDecomposition>(bounds));
    // print the triangulation to stdout
    td->print(std::cout);

    // hand the triangulation to SyclopEST
    auto planner(std::make_shared<oc::SyclopEST>(ss.getSpaceInformation(), td));
    // hand the SyclopEST planner to SimpleSetup
    ss.setPlanner(planner);

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        ss.getSolutionPath().asGeometric().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    planWithSimpleSetup();
    std::cout << std::endl;
    return 0;
}
