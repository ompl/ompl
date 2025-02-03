#include <nanobind/nanobind.h>

namespace ompl::binding::control
{
    void initAutomaton(nanobind::module_& m);
    void initControl(nanobind::module_& m);
    void initControlSampler(nanobind::module_& m);
    void initControlSpace(nanobind::module_& m);
    void initControlSpaceTypes(nanobind::module_& m);
    void initDecomposition(nanobind::module_& m);
    void initDirectedControlSampler(nanobind::module_& m);
    void initDiscreteControlSpace(nanobind::module_& m);
    void initEST(nanobind::module_& m);
    void initGridDecomposition(nanobind::module_& m);
    void initKPIECE1(nanobind::module_& m);
    void initLTLPlanner(nanobind::module_& m);
    void initLTLProblemDefinition(nanobind::module_& m);
    void initLTLSpaceInformation(nanobind::module_& m);
    void initODESolver(nanobind::module_& m);
    void initPDST(nanobind::module_& m);
    void initPathControl(nanobind::module_& m);
    void initPlannerData(nanobind::module_& m);
    void initPlannerDataStorage(nanobind::module_& m);
    void initPlannerIncludes(nanobind::module_& m);
    void initProductGraph(nanobind::module_& m);
    void initPropositionalDecomposition(nanobind::module_& m);
    void initRRT(nanobind::module_& m);
    void initRealVectorControlSpace(nanobind::module_& m);
    void initSST(nanobind::module_& m);
    void initSimpleDirectedControlSampler(nanobind::module_& m);
    void initSimpleSetup(nanobind::module_& m);
    void initSpaceInformation(nanobind::module_& m);
    void initStatePropagator(nanobind::module_& m);
    void initSteeredControlSampler(nanobind::module_& m);
    void initSyclop(nanobind::module_& m);
    void initSyclopEST(nanobind::module_& m);
    void initSyclopRRT(nanobind::module_& m);
    void initWorld(nanobind::module_& m);
}  // namespace ompl::binding::control
