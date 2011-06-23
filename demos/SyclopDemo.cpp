#include <ompl/base/GoalState.h>
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
        const ob::CompoundState* cs = s->as<ob::CompoundState>();
        const ob::SE2StateSpace::StateType *ws = cs->as<ob::SE2StateSpace::StateType>(0);
        std::vector<double> coord(2);
        coord[0] = ws->getX();
        coord[1] = ws->getY();
        return oc::GridDecomposition::locateRegion(coord);
    }

    virtual void stateToCoord(const ob::State *s, std::vector<double>& coord)
    {
        const ob::CompoundState* cs = s->as<ob::CompoundState>();
        const ob::SE2StateSpace::StateType *ws = cs->as<ob::SE2StateSpace::StateType>(0);
        coord.resize(2);
        coord[0] = ws->getX();
        coord[1] = ws->getY();
    }
};

bool isStateValid(const oc::SpaceInformation* si, const ob::State *s)
{
    const ob::CompoundState *cs = s->as<ob::CompoundState>();
    const ob::SE2StateSpace::StateType *se = cs->as<ob::SE2StateSpace::StateType>(0);
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

class CarModel
{
public:
    CarModel(const ob::StateSpace* space) : space_(space), carLength_(0.30)
    {
    }

    void operator()(const ob::State* state, const oc::Control* control, std::valarray<double>& dstate) const
    {
        const ob::CompoundStateSpace::StateType* cs = state->as<ob::CompoundStateSpace::StateType>();
        const ob::SE2StateSpace::StateType* location = cs->as<ob::SE2StateSpace::StateType>(0);
        const ob::RealVectorStateSpace::StateType* vels = cs->as<ob::RealVectorStateSpace::StateType>(1);
        const oc::RealVectorControlSpace::ControlType* ctrl = control->as<oc::RealVectorControlSpace::ControlType>();

        const double x = location->getX();
        const double y = location->getY();
        const double angle = location->getYaw();
        const double velocity = (*vels)[0];
        const double steerVelocity = (*vels)[1];
        const double accel = (*ctrl)[0];
        const double steerAccel = (*ctrl)[1];

        dstate.resize(5);
        dstate[0] = velocity*cos(angle);
        dstate[1] = velocity*sin(angle);
        dstate[2] = velocity*tan(steerVelocity)/carLength_;
        dstate[3] = accel;
        dstate[4] = steerAccel;
    }

    void update(ob::State* state, const std::valarray<double>& stateChange) const
    {
        ob::CompoundStateSpace::StateType* cs = state->as<ob::CompoundStateSpace::StateType>();
        ob::SE2StateSpace::StateType* location = cs->as<ob::SE2StateSpace::StateType>(0);
        ob::RealVectorStateSpace::StateType* vels = cs->as<ob::RealVectorStateSpace::StateType>(1);
        location->setXY(location->getX()+stateChange[0], location->getY()+stateChange[1]);
        location->setYaw(location->getYaw() + stateChange[2]);
        (*vels)[0] += stateChange[3];
        (*vels)[1] += stateChange[4];
    }

private:
    const ob::StateSpace* space_;
    const double carLength_;
};

template<typename F>
class EulerIntegrator
{
public:
    EulerIntegrator(const ob::StateSpace* space, const double timestep) :
    space_(space), timestep_(timestep), ode_(space)
    {
    }

    void propagate(const ob::State* start, const oc::Control* control, const double duration, ob::State* result) const
    {
        double t = timestep_;
        std::valarray<double> dstate;
        space_->copyState(result, start);
        while (t < duration + std::numeric_limits<double>::epsilon())
        {
            ode_(result, control, dstate);
            ode_.update(result, timestep_ * dstate);
            t += timestep_;
        }
        if (t + std::numeric_limits<double>::epsilon() > duration)
        {
            ode_(result, control, dstate);
            ode_.update(result, (t - duration) * dstate);
        }
    }

    double getTimestep(void) const
    {
        return timestep_;
    }
private:
    const ob::StateSpace* space_;
    const double timestep_;
    F ode_;
};

class CarControlSpace : public oc::RealVectorControlSpace
{
public:
    CarControlSpace(const ob::StateSpacePtr& space, const double timestep) : oc::RealVectorControlSpace(space,2), integrator_(space.get(), timestep)
    {
    }

    virtual void propagate(const ob::State* start, const oc::Control* control, const double duration, ob::State* result) const
    {
        integrator_.propagate(start, control, duration, result);
    }

    double getIntegrationTimestep(void) const
    {
        return integrator_.getTimestep();
    }
private:
    EulerIntegrator<CarModel> integrator_;
};

class ConvexGoalRegion : public ob::GoalSampleableRegion
{
public:
    ConvexGoalRegion(const ob::SpaceInformationPtr& si) : ob::GoalSampleableRegion(si), bounds_(5)
    {
        //x coordinate
        bounds_.setLow(0,1.95);
        bounds_.setHigh(0,2.0);
        //y coordinate
        bounds_.setLow(1, -2.0);
        bounds_.setHigh(1, -2.05);
        //steer angle
        bounds_.setLow(2, -1.57);
        bounds_.setHigh(2, 1.57);
        //forward velocity
        bounds_.setLow(3, -0.05);
        bounds_.setHigh(3, 0.05);
        //steer velocity
	    bounds_.setLow(4, -0.2);
        bounds_.setHigh(4, 0.2);
    }

    virtual bool isSatisfied(const ob::State* state) const
    {
        double distance;
        return isSatisfied(state, &distance);
    }

    virtual bool isSatisfied(const ob::State* state, double* distance) const
    {
        const ob::CompoundStateSpace::StateType* cs = state->as<ob::CompoundStateSpace::StateType>();
        const ob::SE2StateSpace::StateType* location = cs->as<ob::SE2StateSpace::StateType>(0);
        const ob::RealVectorStateSpace::StateType* vels = cs->as<ob::RealVectorStateSpace::StateType>(1);
        std::valarray<double> stateVals(2);
        stateVals[0] = location->getX();
        stateVals[1] = location->getY();
        /*stateVals[2] = location->getYaw();
        stateVals[3] = (*vels)[0];
        stateVals[4] = (*vels)[1];*/

        double sq_distance = 0.0;
        bool satisfied = true;
        for (std::size_t i = 0; i < stateVals.size(); ++i)
        {
            const double diff = stateVals[i] - (bounds_.high[i] + bounds_.low[i]) / 2.0;
            satisfied &= (stateVals[i] >= bounds_.low[i] && stateVals[i] <= bounds_.high[i]);
            sq_distance += diff*diff;
        }
        *distance = sqrt(sq_distance);
        return satisfied;
    }

    virtual double distanceGoal(const ob::State* state) const
    {
        double distance;
        isSatisfied(state, &distance);
        return distance;
    }

    virtual void sampleGoal(ob::State* state) const
    {
        ob::CompoundStateSpace::StateType* cs = state->as<ob::CompoundStateSpace::StateType>();
        ob::SE2StateSpace::StateType* location = cs->as<ob::SE2StateSpace::StateType>(0);
        ob::RealVectorStateSpace::StateType* vels = cs->as<ob::RealVectorStateSpace::StateType>(1);
        ompl::RNG rng;
        location->setXY(rng.uniformReal(bounds_.low[0], bounds_.high[0]),
            rng.uniformReal(bounds_.low[1], bounds_.high[1]));
        location->setYaw(rng.uniformReal(bounds_.low[2], bounds_.high[2]));
        (*vels)[0] = rng.uniformReal(bounds_.low[3], bounds_.high[3]);
        (*vels)[1] = rng.uniformReal(bounds_.low[4], bounds_.high[4]);
    }

    virtual unsigned int maxSampleCount(void) const
    {
        return RAND_MAX;
    }

private:
    ob::RealVectorBounds bounds_;
};

int main(void)
{
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-3);
    bounds.setHigh(3);
    TestDecomposition grid(27, bounds);
    ob::RealVectorBounds vbounds(2);
    vbounds.setLow(-0.1);
    vbounds.setHigh(0.1);
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.05);
    cbounds.setHigh(0.05);

    //stateSpace : location, forward velocity, steer velocity
    ob::StateSpacePtr locationSpace(new ob::SE2StateSpace());
    locationSpace->as<ob::SE2StateSpace>()->setBounds(bounds);
    ob::StateSpacePtr velSpace(new ob::RealVectorStateSpace(2));
    velSpace->as<ob::RealVectorStateSpace>()->setBounds(vbounds);
    ob::StateSpacePtr stateSpace(new ob::CompoundStateSpace());
    stateSpace->as<ob::CompoundStateSpace>()->addSubSpace(locationSpace, 1.0);
    stateSpace->as<ob::CompoundStateSpace>()->addSubSpace(velSpace, 1.0);

    //controlSpace : forward accel & steer accel
    oc::ControlSpacePtr controlSpace(new CarControlSpace(stateSpace, 0.01));
    controlSpace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    ob::ScopedState<> init(stateSpace);
    ob::CompoundState* cs = init->as<ob::CompoundState>();
    ob::SE2StateSpace::StateType* location = cs->as<ob::SE2StateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType* vels = cs->as<ob::RealVectorStateSpace::StateType>(1);
    location->setX(-2.0);
    location->setY(2.0);
    location->setYaw(0.0);
    (*vels)[0] = 0.0;
    (*vels)[1] = 0.0;

    oc::SpaceInformationPtr si(new oc::SpaceInformation(stateSpace, controlSpace));
    si->setStateValidityChecker(boost::bind(&isStateValid, si.get(), _1));
    si->setMinMaxControlDuration(1, 10);
    si->setPropagationStepSize(0.5);
    si->setup();

    //ob::PlannerPtr planner(new oc::RRT(si));
    ob::PlannerPtr planner(new oc::SyclopRRT(si,grid));

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->addStartState(init);
    pdef->setGoal(static_cast<ob::GoalPtr>(new ConvexGoalRegion(si)));
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
