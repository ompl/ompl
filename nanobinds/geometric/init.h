#include <nanobind/nanobind.h>

namespace ompl::binding::geometric
{
    void initPlannersRrt_RRT(nanobind::module_& m);
    void initPlannersRrt_RRTConnect(nanobind::module_& m);
    void init_PathGeometric(nanobind::module_& m);
    void init_PathHybridization(nanobind::module_& m);
    void init_PathSimplifier(nanobind::module_& m);
    void init_SimpleSetup(nanobind::module_& m);
}  // namespace ompl::binding::geometric
