#include "ompl/control/planners/syclop/Decomposition.h"

ompl::control::Decomposition::Decomposition(const int numRegions, const ompl::base::RealVectorBounds &b) : regions(numRegions), bounds(b)
{
}

ompl::control::Decomposition::~Decomposition()
{
}

int ompl::control::Decomposition::getNumRegions() const
{
    return regions.size();
}

const ompl::base::RealVectorBounds& ompl::control::Decomposition::getBounds() const
{
    return bounds;
}

void ompl::control::Decomposition::print() const
{
    for (unsigned int i = 0; i < regions.size(); ++i)
    {
        std::cout << "Region " << i << " consists of " << regions[i].states.size() << " states." << std::endl;
    }
}

/*Region& ompl::control::Decomposition::operator[](const int rid)
{
    return regions[rid];
}*/
