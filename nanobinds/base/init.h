#include <nanobind/nanobind.h>

namespace ompl::binding::base
{
    void init_ConstrainedSpaceInformation(nanobind::module_& m);
    void init_Constraint(nanobind::module_& m);
    void init_Cost(nanobind::module_& m);
    void init_Goal(nanobind::module_& m);
    void init_GoalTypes(nanobind::module_& m);
    void init_MotionValidator(nanobind::module_& m);
    void init_Path(nanobind::module_& m);
    void init_Planner(nanobind::module_& m);
    void init_PlannerData(nanobind::module_& m);
    void init_PlannerStatus(nanobind::module_& m);
    void init_PlannerTerminationCondition(nanobind::module_& m);
    void init_ProblemDefinition(nanobind::module_& m);
    void init_ProjectionEvaluator(nanobind::module_& m);
    void init_SpaceInformation(nanobind::module_& m);
    void init_State(nanobind::module_& m);
    void init_StateSampler(nanobind::module_& m);
    void init_StateSpace(nanobind::module_& m);
    void init_StateValidityChecker(nanobind::module_& m);
    void init_ValidStateSampler(nanobind::module_& m);
    void initGoals_GoalLazySamples(nanobind::module_& m);
    void initGoals_GoalRegion(nanobind::module_& m);
    void initGoals_GoalSampleableRegion(nanobind::module_& m);
    void initGoals_GoalSpace(nanobind::module_& m);
    void initGoals_GoalState(nanobind::module_& m);
    void initGoals_GoalStates(nanobind::module_& m);
    void initSamplers_UniformValidStateSampler(nanobind::module_& m);
    void initSpacesConstraint_AtlasChart(nanobind::module_& m);
    void initSpacesConstraint_AtlasStateSpace(nanobind::module_& m);
    void initSpacesConstraint_ConstrainedStateSpace(nanobind::module_& m);
    void initSpacesConstraint_ProjectedStateSpace(nanobind::module_& m);
    void initSpacesConstraint_TangentBundleStateSpace(nanobind::module_& m);
    void initSpaces_DiscreteStateSpace(nanobind::module_& m);
    void initSpaces_EmptyStateSpace(nanobind::module_& m);
    void initSpaces_RealVectorBounds(nanobind::module_& m);
    void initSpaces_RealVectorStateProjections(nanobind::module_& m);
    void initSpaces_RealVectorStateSpace(nanobind::module_& m);
    void initSpaces_SE2StateSpace(nanobind::module_& m);
    void initSpaces_SE3StateSpace(nanobind::module_& m);
    void initSpaces_SO2StateSpace(nanobind::module_& m);
    void initSpaces_SO3StateSpace(nanobind::module_& m);
    void initSpaces_SpaceTimeStateSpace(nanobind::module_& m);
    void initSpaces_TimeStateSpace(nanobind::module_& m);
    void initSpaces_WrapperStateSpace(nanobind::module_& m);
}  // namespace ompl::binding::base
