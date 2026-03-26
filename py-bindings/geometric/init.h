#include <nanobind/nanobind.h>

namespace ompl::binding::geometric
{
    void initPlannersPrm_PRM(nanobind::module_ &m);
    void initPlannersPrm_PRMstar(nanobind::module_ &m);
    void initPlannersRrt_RRT(nanobind::module_ &m);
    void initPlannersRrt_RRTConnect(nanobind::module_ &m);
    void initPlannersRrt_RRTstar(nanobind::module_ &m);
    void initPlannersRrt_InformedRRTstar(nanobind::module_ &m);
    void initPlannersRrt_SORRTstar(nanobind::module_ &m);
    void initPlannersRrt_AORRTC(nanobind::module_ &m);
    void initPlannersInformedtrees_BITstar(nanobind::module_ &m);
    void initPlannersFmt_FMT(nanobind::module_ &m);
    void initPlannersFmt_BFMT(nanobind::module_ &m);
    void initPlannersKpiece_KPIECE1(nanobind::module_ &m);
    void initPlannersKpiece_BKPIECE1(nanobind::module_ &m);
    void initPlannersKpiece_LBKPIECE1(nanobind::module_ &m);
    void initPlannersKpiece_Discretization(nanobind::module_ &m);
    void init_PathGeometric(nanobind::module_ &m);
    void init_PathSimplifier(nanobind::module_ &m);
    void init_SimpleSetup(nanobind::module_ &m);
}  // namespace ompl::binding::geometric
