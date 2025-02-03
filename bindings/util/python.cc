#include "init.hh"

namespace ob = ompl::binding::util;

NB_MODULE(_util, m)
{
    ob::initClassForward(m);
    ob::initConsole(m);
    ob::initDisableCompilerWarning(m);
    ob::initException(m);
    ob::initGeometricEquations(m);
    ob::initHash(m);
    ob::initPPM(m);
    ob::initProlateHyperspheroid(m);
    ob::initRandomNumbers(m);
    ob::initString(m);
    ob::initTime(m);
}
