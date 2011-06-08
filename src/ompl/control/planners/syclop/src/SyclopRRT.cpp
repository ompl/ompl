#include "ompl/control/planners/syclop/SyclopRRT.h"

ompl::control::SyclopRRT::SyclopRRT(const SpaceInformationPtr &si, Decomposition &d) : Syclop(si,d)
{
}

ompl::control::SyclopRRT::~SyclopRRT(void)
{
}

void ompl::control::SyclopRRT::setup(void)
{
    Syclop::setup();
}

void ompl::control::SyclopRRT::initializeTree(const base::State *s)
{

}

void ompl::control::SyclopRRT::selectAndExtend(int region, std::set<const base::State*> newStates)
{

}
