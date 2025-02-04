#include <nanobind/nanobind.h>

namespace ompl::binding::util
{
    void init_ClassForward(nanobind::module_& m);
    void init_Console(nanobind::module_& m);
    void init_DisableCompilerWarning(nanobind::module_& m);
    void init_Exception(nanobind::module_& m);
    void init_GeometricEquations(nanobind::module_& m);
    void init_Hash(nanobind::module_& m);
    void init_PPM(nanobind::module_& m);
    void init_ProlateHyperspheroid(nanobind::module_& m);
    void init_RandomNumbers(nanobind::module_& m);
    void init_String(nanobind::module_& m);
    void init_Time(nanobind::module_& m);
}  // namespace ompl::binding::util
