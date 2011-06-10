#include "ompl/control/planners/syclop/Decomposition.h"

ompl::control::Decomposition::Decomposition(const int n, const ompl::base::RealVectorBounds& b) : numRegions(n), bounds(b)
{
}

ompl::control::Decomposition::~Decomposition()
{
}

int ompl::control::Decomposition::getNumRegions() const
{
    return numRegions;
}

const ompl::base::RealVectorBounds& ompl::control::Decomposition::getBounds() const
{
    return bounds;
}
