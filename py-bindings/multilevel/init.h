#include <nanobind/nanobind.h>

namespace ompl::binding::multilevel
{
    void init_FindSectionType(nanobind::module_ &m);
    void init_ProjectionTypes(nanobind::module_ &m);
    void init_Projection(nanobind::module_ &m);
    void init_ProjectionFactory(nanobind::module_ &m);
    void initProjections_Identity(nanobind::module_ &m);
    void initProjections_SE2_R2(nanobind::module_ &m);
    void initProjections_SE2RN_R2(nanobind::module_ &m);
    void initProjections_SE3_R3(nanobind::module_ &m);
    void initProjections_SE3RN_R3(nanobind::module_ &m);
    void initProjections_RN_RM(nanobind::module_ &m);
    void init_Parameter(nanobind::module_ &m);
    void init_ParameterExponentialDecay(nanobind::module_ &m);
    void init_ParameterSmoothStep(nanobind::module_ &m);
    void init_BundleSpace(nanobind::module_ &m);
    void init_BundleSpaceGraph(nanobind::module_ &m);
    void init_BundleSpaceSequence(nanobind::module_ &m);
    void init_PlannerDataVertexAnnotated(nanobind::module_ &m);
    void initMetrics_BundleSpaceMetric(nanobind::module_ &m);
    void initMetrics_Geodesic(nanobind::module_ &m);
    void initPropagators_BundleSpacePropagator(nanobind::module_ &m);
    void initPropagators_Geometric(nanobind::module_ &m);
    void initImportance_BundleSpaceImportance(nanobind::module_ &m);
    void initImportance_Exponential(nanobind::module_ &m);
    void initImportance_Greedy(nanobind::module_ &m);
    void initImportance_Uniform(nanobind::module_ &m);
    void initGraphsampler_GraphSampler(nanobind::module_ &m);
    void initGraphsampler_RandomEdge(nanobind::module_ &m);
    void initGraphsampler_RandomVertex(nanobind::module_ &m);
    void initGraphsampler_RandomDegreeVertex(nanobind::module_ &m);
    void init_PlannerMultiLevel(nanobind::module_ &m);
    void initPlanners_QRRT(nanobind::module_ &m);
    void initPlanners_QRRTStar(nanobind::module_ &m);
    void initPlanners_QMP(nanobind::module_ &m);
    void initPlanners_QMPStar(nanobind::module_ &m);
}  // namespace ompl::binding::multilevel
