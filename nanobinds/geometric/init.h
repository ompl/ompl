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
    void init_PathGeometric(nanobind::module_& m);
    void init_PathHybridization(nanobind::module_& m);
    void init_PathSimplifier(nanobind::module_& m);
    void init_SimpleSetup(nanobind::module_& m);
}  // namespace ompl::binding::geometric
