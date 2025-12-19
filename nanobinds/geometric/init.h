#include <nanobind/nanobind.h>

namespace ompl::binding::geometric
{
    void initPlannersPrm_ConnectionStrategy(nanobind::module_& m);
    void initPlannersPrm_LazyPRM(nanobind::module_& m);
    void initPlannersPrm_LazyPRMstar(nanobind::module_& m);
    void initPlannersPrm_PRM(nanobind::module_& m);
    void initPlannersPrm_PRMstar(nanobind::module_& m);
    void initPlannersPrm_SPARS(nanobind::module_& m);
    void initPlannersPrm_SPARStwo(nanobind::module_& m);
    void initPlannersSst_SST(nanobind::module_& m);
    void initPlannersStride_STRIDE(nanobind::module_& m);
    void initPlannersXxl_XXL(nanobind::module_& m);
    void initPlannersXxl_XXLDecomposition(nanobind::module_& m);
    void initPlannersXxl_XXLPlanarDecomposition(nanobind::module_& m);
    void initPlannersXxl_XXLPositionDecomposition(nanobind::module_& m);
    void init_PathGeometric(nanobind::module_& m);
    void init_PathHybridization(nanobind::module_& m);
    void init_PathSimplifier(nanobind::module_& m);
    void init_SimpleSetup(nanobind::module_& m);
}  // namespace ompl::binding::geometric
