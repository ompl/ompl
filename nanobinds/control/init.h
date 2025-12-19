#include <nanobind/nanobind.h>

namespace ompl::binding::control
{
    void init_Control(nanobind::module_& m);
    void init_ControlSampler(nanobind::module_& m);
    void init_ControlSpace(nanobind::module_& m);
    void init_ControlSpaceTypes(nanobind::module_& m);
    void init_DirectedControlSampler(nanobind::module_& m);
    void init_PathControl(nanobind::module_& m);
    void init_PlannerData(nanobind::module_& m);
    void init_SimpleDirectedControlSampler(nanobind::module_& m);
    void init_SimpleSetup(nanobind::module_& m);
    void init_SpaceInformation(nanobind::module_& m);
    void init_StatePropagator(nanobind::module_& m);
    void init_SteeredControlSampler(nanobind::module_& m);
    void initPlannersEst_EST(nanobind::module_& m);
    void initPlannersKpiece_KPIECE1(nanobind::module_& m);
    void initPlannersPdst_PDST(nanobind::module_& m);
    void initPlannersRrt_RRT(nanobind::module_& m);
    void initPlannersSst_SST(nanobind::module_& m);
    void initPlannersSyclop_Decomposition(nanobind::module_& m);
    void initPlannersSyclop_GridDecomposition(nanobind::module_& m);
    void initPlannersSyclop_Syclop(nanobind::module_& m);
    void initPlannersSyclop_SyclopEST(nanobind::module_& m);
    void initPlannersSyclop_SyclopRRT(nanobind::module_& m);
    void initSpaces_DiscreteControlSpace(nanobind::module_& m);
    void initSpaces_RealVectorControlSpace(nanobind::module_& m);
    
}  // namespace ompl::binding::control
