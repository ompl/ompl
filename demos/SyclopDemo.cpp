#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/syclop/Syclop.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Time.h>
#define BOOST_NO_HASH
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <cmath>
#include <iostream>
#include <vector>

namespace ob = ompl::base;
namespace oc = ompl::control;

class TestDecomposition : public oc::GridDecomposition
{
    public:
    TestDecomposition(const int length, const ob::RealVectorBounds &bounds) : oc::GridDecomposition(length, 2, bounds)
    {
    }
    virtual ~TestDecomposition()
    {
    }

    virtual int locateRegion(const ob::State *s)
    {
        const ob::SE2StateSpace::StateType *ws = s->as<ob::SE2StateSpace::StateType>();
        std::vector<double> coord(2);
        coord[0] = ws->getX();
        coord[1] = ws->getY();
        return oc::GridDecomposition::locateRegion(coord);
    }

    virtual void stateToCoord(const ob::State *s, std::vector<double>& coord)
    {
        const ob::SE2StateSpace::StateType *ws = s->as<ob::SE2StateSpace::StateType>();
        coord.resize(2);
        coord[0] = ws->getX();
        coord[1] = ws->getY();
    }
};

bool isStateValid(const oc::SpaceInformation* si, const ob::State *s)
{
    const ob::SE2StateSpace::StateType *se = s->as<ob::SE2StateSpace::StateType>();
    const double x = se->getX();
    const double y = se->getY();
    if (x < -2.0 && y < -2.0)
        return false;
    if (x > 0 && x < 1 && y > 0)
        return false;
    if (x > 2.5 && fabs(y) < 1.5)
        return false;
    return si->satisfiesBounds(s);
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const ob::SE2StateSpace::StateType* location = start->as<ob::SE2StateSpace::StateType>();
    const oc::RealVectorControlSpace::ControlType* ctrl = control->as<oc::RealVectorControlSpace::ControlType>();

    const double x = location->getX();
    const double y = location->getY();
    const double angle = location->getYaw();
    const double velocity = (*ctrl)[0];
    const double steerVelocity = (*ctrl)[1];

    ob::SE2StateSpace::StateType* newLocation = result->as<ob::SE2StateSpace::StateType>();
    newLocation->setXY(x + velocity*duration*cos(angle), y + velocity*duration*sin(angle));
    newLocation->setYaw(angle + duration*steerVelocity);
}

int main(void)
{
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-3);
    bounds.setHigh(3);
    TestDecomposition grid(3, bounds);
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-2);
    cbounds.setHigh(2);

    ob::StateSpacePtr manifold(new ob::SE2StateSpace());
    manifold->as<ob::SE2StateSpace>()->setBounds(bounds);

    //controlSpace : forward velocity & steer velocity
    oc::ControlSpacePtr controlSpace(new oc::RealVectorControlSpace(manifold, 2));
    controlSpace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);
    controlSpace->setPropagationFunction(boost::bind(&propagate, _1, _2, _3, _4));

    ob::ScopedState<ob::SE2StateSpace> init(manifold);
    init->setX(-2.0);
    init->setY(2.0);
    init->setYaw(0);

    ob::ScopedState<ob::SE2StateSpace> goal(init);
    goal->setX(2.0);
    goal->setY(-2.0);

    oc::SpaceInformationPtr si(new oc::SpaceInformation(manifold, controlSpace));
    si->setStateValidityChecker(boost::bind(&isStateValid, si.get(), _1));
    si->setMinMaxControlDuration(1, 10);
    si->setPropagationStepSize(0.1);
    si->setup();

    ob::PlannerPtr planner(new oc::RRT(si));
    //ob::PlannerPtr planner(new oc::SyclopRRT(si,grid));

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(init, goal, 0.05);
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::time::point startTime = ompl::time::now();
    bool solved = planner->solve(120.0);
    double duration = ompl::time::seconds(ompl::time::now()-startTime);
    ob::PlannerData pdata;
    planner->getPlannerData(pdata);
    std::cerr << planner->getName() << " " << solved << " ";
    std::cerr << duration << " " << pdata.states.size() << std::endl;

    return 0;
}
