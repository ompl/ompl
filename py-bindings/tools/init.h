#include <nanobind/nanobind.h>

namespace ompl::binding::tools
{
    void initBenchmark_Benchmark(nanobind::module_ &m);
    void initBenchmark_MachineSpecs(nanobind::module_ &m);
    void initConfig_MagicConstants(nanobind::module_ &m);
    void initConfig_SelfConfig(nanobind::module_ &m);
    void initLightning_DynamicTimeWarp(nanobind::module_ &m);
    void initLightning_LightningDB(nanobind::module_ &m);
    void initThunder_SPARSdb(nanobind::module_ &m);
    void initThunder_ThunderDB(nanobind::module_ &m);
}  // namespace ompl::binding::tools
