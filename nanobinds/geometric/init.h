#include <nanobind/nanobind.h>

namespace ompl::binding::geometric
{
    void initPlannersPdst_PDST(nanobind::module_& m);
    void initPlannersPrm_ConnectionStrategy(nanobind::module_& m);
    void initPlannersPrm_LazyPRM(nanobind::module_& m);
    void initPlannersPrm_LazyPRMstar(nanobind::module_& m);
    void initPlannersPrm_PRM(nanobind::module_& m);
    void initPlannersPrm_PRMstar(nanobind::module_& m);
    void initPlannersPrm_SPARS(nanobind::module_& m);
    void initPlannersPrm_SPARStwo(nanobind::module_& m);
    void initPlannersRlrt_BiRLRT(nanobind::module_& m);
    void initPlannersRlrt_RLRT(nanobind::module_& m);
    void initPlannersRrt_BiTRRT(nanobind::module_& m);
    void initPlannersRrt_InformedRRTstar(nanobind::module_& m);
    void initPlannersRrt_LBTRRT(nanobind::module_& m);
    void initPlannersRrt_LazyLBTRRT(nanobind::module_& m);
    void initPlannersRrt_LazyRRT(nanobind::module_& m);
    void initPlannersRrt_RRT(nanobind::module_& m);
    void initPlannersRrt_RRTConnect(nanobind::module_& m);
    void initPlannersRrt_RRTXstatic(nanobind::module_& m);
    void initPlannersRrt_RRTsharp(nanobind::module_& m);
    void initPlannersRrt_RRTstar(nanobind::module_& m);
    void initPlannersRrt_SORRTstar(nanobind::module_& m);
    void initPlannersRrt_STRRTstar(nanobind::module_& m);
    void initPlannersRrt_TRRT(nanobind::module_& m);
    void initPlannersRrt_TSRRT(nanobind::module_& m);
    void initPlannersRrt_VFRRT(nanobind::module_& m);
    void initPlannersRrt_pRRT(nanobind::module_& m);
    void initPlannersSbl_SBL(nanobind::module_& m);
    void initPlannersSbl_pSBL(nanobind::module_& m);
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
