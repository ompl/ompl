#include "init.hh"

NB_MODULE(_util, m)
{
    ompl::binding::util::init_ClassForward(m);
    ompl::binding::util::init_Console(m);
    ompl::binding::util::init_DisableCompilerWarning(m);
    ompl::binding::util::init_Exception(m);
    ompl::binding::util::init_GeometricEquations(m);
    ompl::binding::util::init_Hash(m);
    ompl::binding::util::init_PPM(m);
    ompl::binding::util::init_ProlateHyperspheroid(m);
    ompl::binding::util::init_RandomNumbers(m);
    ompl::binding::util::init_String(m);
    ompl::binding::util::init_Time(m);
}
