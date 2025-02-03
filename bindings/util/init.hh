#include <nanobind/nanobind.h>

namespace ompl::binding::util
{
    void initClassForward(nanobind::module_& m);
    void initConsole(nanobind::module_& m);
    void initDisableCompilerWarning(nanobind::module_& m);
    void initException(nanobind::module_& m);
    void initGeometricEquations(nanobind::module_& m);
    void initHash(nanobind::module_& m);
    void initPPM(nanobind::module_& m);
    void initProlateHyperspheroid(nanobind::module_& m);
    void initRandomNumbers(nanobind::module_& m);
    void initString(nanobind::module_& m);
    void initTime(nanobind::module_& m);
}  // namespace ompl::binding::util
