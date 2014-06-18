#include "ompl/control/planners/ltl/PropositionalDecomposition.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/base/StateSampler.h"
#include <vector>

ompl::control::PropositionalDecomposition::PropositionalDecomposition(const DecompositionPtr& decomp) :
    Decomposition(decomp->getDimension(), decomp->getBounds()),
    decomp_(decomp)
{
}

ompl::control::PropositionalDecomposition::~PropositionalDecomposition(void)
{
}

int ompl::control::PropositionalDecomposition::getNumRegions(void) const
{
    return decomp_->getNumRegions();
}

double ompl::control::PropositionalDecomposition::getRegionVolume(int rid)
{
    return decomp_->getRegionVolume(rid);
}

int ompl::control::PropositionalDecomposition::locateRegion(const base::State* s) const
{
    return decomp_->locateRegion(s);
}

void ompl::control::PropositionalDecomposition::project(const base::State* s, std::vector<double>& coord) const
{
    return decomp_->project(s, coord);
}

void ompl::control::PropositionalDecomposition::getNeighbors(int rid, std::vector<int>& neighbors) const
{
    decomp_->getNeighbors(rid, neighbors);
}

void ompl::control::PropositionalDecomposition::sampleFromRegion(int rid, RNG& rng, std::vector<double>& coord) const
{
    decomp_->sampleFromRegion(rid, rng, coord);
}

void ompl::control::PropositionalDecomposition::sampleFullState(
    const base::StateSamplerPtr& sampler,
    const std::vector<double>& coord,
    base::State* s) const
{
    decomp_->sampleFullState(sampler, coord, s);
}
