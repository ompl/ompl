#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <ompl/control/planners/syclop/Syclop.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/util/RandomNumbers.h>
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
    TestDecomposition(const int length, ob::RealVectorBounds &bounds) : oc::GridDecomposition(length, 2, bounds)
    {
    }
    virtual ~TestDecomposition()
    {
    }

    virtual int locateRegion(const ob::State *s)
    {
        const ob::CompoundState *cs = s->as<ob::CompoundState>();
        const ob::SE2StateSpace::StateType *ws = cs->as<ob::SE2StateSpace::StateType>(0);
        std::vector<double> coord(2);
        coord[0] = ws->getX();
        coord[1] = ws->getY();
        return oc::GridDecomposition::locateRegion(coord);
    }

    virtual void stateToCoord(const ob::State *s, std::vector<double>& coord)
    {
        const ob::CompoundState *cs = s->as<ob::CompoundState>();
        const ob::SE2StateSpace::StateType *ws = cs->as<ob::SE2StateSpace::StateType>(0);
        coord.resize(2);
        coord[0] = ws->getX();
        coord[1] = ws->getY();
    }
};

bool isStateValid(const ob::State *s)
{
    const ob::CompoundStateSpace::StateType *cs = s->as<ob::CompoundStateSpace::StateType>();
    const ob::SE2StateSpace::StateType *se = cs->as<ob::SE2StateSpace::StateType>(0);
    const double x = se->getX();
    const double y = se->getY();
    if (x < -0.5 && y < 0.5)
        return false;
    if (x > 0.5 && fabs(y) < 0.5)
        return false;
    return true;
}

int main(void)
{
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    TestDecomposition grid(2, bounds);
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-1);
    cbounds.setHigh(1);

    ob::StateSpacePtr manifold(new ob::CompoundStateSpace());
    ob::StateSpacePtr locSpace(new ob::SE2StateSpace());
    locSpace->as<ob::SE2StateSpace>()->setBounds(bounds);
    ob::StateSpacePtr velSpace(new ob::RealVectorStateSpace(1));
    velSpace->as<ob::RealVectorStateSpace>()->setBounds(cbounds);
    manifold->as<ob::CompoundStateSpace>()->addSubSpace(locSpace, 0.8);
    manifold->as<ob::CompoundStateSpace>()->addSubSpace(velSpace, 0.2);

    oc::ControlSpacePtr controlSpace(new oc::RealVectorControlSpace(manifold, 1));
    controlSpace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    ob::ScopedState<ob::CompoundStateSpace> init(manifold);
    ob::SE2StateSpace::StateType *se = init->as<ob::SE2StateSpace::StateType>(0);
    se->setX(-0.75);
    se->setY(0.8);
    se->setYaw(0);
    init->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;

    ob::ScopedState<ob::CompoundStateSpace> goal(init);
    se = goal->as<ob::SE2StateSpace::StateType>(0);
    se->setX(0.65);
    se->setY(-0.7);

    //createGraphs();
    oc::SpaceInformationPtr si(new oc::SpaceInformation(manifold, controlSpace));
    si->setStateValidityChecker(boost::bind(&isStateValid, _1));
    si->setup();

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(init, goal, 0.05);
    oc::SyclopRRT planner(si, grid);
    planner.setProblemDefinition(pdef);
    planner.setup();

    return 0;
}
