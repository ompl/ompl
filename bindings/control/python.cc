#include "init.hh"

namespace ob = ompl::binding::control;

NB_MODULE(_control, m)
{
    ob::initAutomaton(m);
    ob::initControl(m);
    ob::initControlSampler(m);
    ob::initControlSpace(m);
    ob::initControlSpaceTypes(m);
    ob::initDecomposition(m);
    ob::initDirectedControlSampler(m);
    ob::initDiscreteControlSpace(m);
    ob::initEST(m);
    ob::initGridDecomposition(m);
    ob::initKPIECE1(m);
    ob::initLTLPlanner(m);
    ob::initLTLProblemDefinition(m);
    ob::initLTLSpaceInformation(m);
    ob::initODESolver(m);
    ob::initPDST(m);
    ob::initPathControl(m);
    ob::initPlannerData(m);
    ob::initPlannerDataStorage(m);
    ob::initPlannerIncludes(m);
    ob::initProductGraph(m);
    ob::initPropositionalDecomposition(m);
    ob::initRRT(m);
    ob::initRealVectorControlSpace(m);
    ob::initSST(m);
    ob::initSimpleDirectedControlSampler(m);
    ob::initSimpleSetup(m);
    ob::initSpaceInformation(m);
    ob::initStatePropagator(m);
    ob::initSteeredControlSampler(m);
    ob::initSyclop(m);
    ob::initSyclopEST(m);
    ob::initSyclopRRT(m);
    ob::initWorld(m);
}
