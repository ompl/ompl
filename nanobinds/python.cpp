#include "base/init.h"
#include "control/init.h"
#include "geometric/init.h"
#include "util/init.h"

#include <nanobind/nanobind.h>
namespace nb = nanobind;

NB_MODULE(_ompl, m)
{
    nb::set_leak_warnings(false);
    
    m.doc() = "OMPL Python Bindings";

    nb::module_ base = m.def_submodule("base");

    ompl::binding::base::init_Cost(base);
    ompl::binding::base::init_Goal(base);
    ompl::binding::base::init_GoalTypes(base);
    ompl::binding::base::init_MotionValidator(base);
    ompl::binding::base::init_ValidStateSampler(base);
    ompl::binding::base::init_Constraint(base);
    ompl::binding::base::init_Path(base);
    ompl::binding::base::init_Planner(base);
    ompl::binding::base::init_PlannerData(base);
    ompl::binding::base::init_PlannerStatus(base);
    ompl::binding::base::init_PlannerTerminationCondition(base);
    ompl::binding::base::init_StateSampler(base);
    ompl::binding::base::init_ProblemDefinition(base);
    ompl::binding::base::init_ProjectionEvaluator(base);
    ompl::binding::base::init_SpaceInformation(base);
    ompl::binding::base::init_State(base);
    ompl::binding::base::init_StateSpace(base);
    ompl::binding::base::init_StateValidityChecker(base);
    ompl::binding::base::initGoals_GoalRegion(base);
    ompl::binding::base::initGoals_GoalSampleableRegion(base);
    ompl::binding::base::initGoals_GoalSpace(base);
    ompl::binding::base::initGoals_GoalState(base);
    ompl::binding::base::initGoals_GoalStates(base);
    ompl::binding::base::initGoals_GoalLazySamples(base);
    ompl::binding::base::init_ConstrainedSpaceInformation(base);

    ompl::binding::base::initSamplers_UniformValidStateSampler(base);

    ompl::binding::base::initSpaces_DiscreteStateSpace(base);
    ompl::binding::base::initSpaces_RealVectorBounds(base);
    ompl::binding::base::initSpaces_RealVectorStateProjections(base);
    ompl::binding::base::initSpaces_RealVectorStateSpace(base);
    ompl::binding::base::initSpaces_SE2StateSpace(base);
    ompl::binding::base::initSpaces_SE3StateSpace(base);
    ompl::binding::base::initSpaces_SO2StateSpace(base);
    ompl::binding::base::initSpaces_SO3StateSpace(base);
    ompl::binding::base::initSpaces_SpaceTimeStateSpace(base);
    ompl::binding::base::initSpaces_TimeStateSpace(base);
    ompl::binding::base::initSpaces_WrapperStateSpace(base);
    ompl::binding::base::initSpaces_EmptyStateSpace(base);

    ompl::binding::base::initSpacesConstraint_ConstrainedStateSpace(base);
    ompl::binding::base::initSpacesConstraint_ProjectedStateSpace(base);
    ompl::binding::base::initSpacesConstraint_AtlasChart(base);
    ompl::binding::base::initSpacesConstraint_AtlasStateSpace(base);
    ompl::binding::base::initSpacesConstraint_TangentBundleStateSpace(base);


    nb::module_ geometric = m.def_submodule("geometric");
    ompl::binding::geometric::init_PathGeometric(geometric);
    ompl::binding::geometric::init_PathHybridization(geometric);
    ompl::binding::geometric::init_PathSimplifier(geometric);
    ompl::binding::geometric::init_SimpleSetup(geometric);



    ompl::binding::geometric::initPlannersRrt_RRT(geometric);
    ompl::binding::geometric::initPlannersRrt_RRTConnect(geometric);



    nb::module_ control = m.def_submodule("control");
    ompl::binding::control::init_Control(control);
    ompl::binding::control::init_ControlSampler(control);
    ompl::binding::control::init_ControlSpace(control);
    ompl::binding::control::init_ControlSpaceTypes(control);
    ompl::binding::control::init_DirectedControlSampler(control);
    ompl::binding::control::init_PathControl(control);
    ompl::binding::control::init_PlannerData(control);
    ompl::binding::control::init_SimpleDirectedControlSampler(control);
    ompl::binding::control::init_SimpleSetup(control);
    ompl::binding::control::init_SpaceInformation(control);
    ompl::binding::control::init_StatePropagator(control);
    ompl::binding::control::init_SteeredControlSampler(control);
    ompl::binding::control::initPlannersEst_EST(control);
    ompl::binding::control::initPlannersKpiece_KPIECE1(control);
    ompl::binding::control::initPlannersRrt_RRT(control);
    ompl::binding::control::initPlannersSst_SST(control);
    ompl::binding::control::initSpaces_DiscreteControlSpace(control);
    ompl::binding::control::initSpaces_RealVectorControlSpace(control);

    nb::module_ util = m.def_submodule("util");
    ompl::binding::util::init_Console(util);
    ompl::binding::util::init_PPM(util);
}