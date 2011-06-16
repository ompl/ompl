#include "ompl/control/planners/syclop/SyclopRRT.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"

ompl::control::SyclopRRT::SyclopRRT(const SpaceInformationPtr &si, Decomposition &d) : Syclop(si,d), siC_(si.get()),
    sampler_(si_->allocStateSampler()), controlSampler_(siC_->allocControlSampler()), goalBias_(0.05)
{
}

ompl::control::SyclopRRT::~SyclopRRT(void)
{
    freeMemory();
}

void ompl::control::SyclopRRT::setup(void)
{
    if (!nn_)
        nn_.reset(new NearestNeighborsSqrtApprox<Motion*>());
    nn_->setDistanceFunction(boost::bind(&SyclopRRT::distanceFunction, this, _1, _2));
    Syclop::setup();
}

void ompl::control::SyclopRRT::clear(void)
{
    Syclop::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::control::SyclopRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
        data.recordEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
}

void ompl::control::SyclopRRT::initializeTree(const base::State *s)
{
    Motion* motion = new Motion(siC_);
    si_->copyState(motion->state, s);
    siC_->nullControl(motion->control);
    nn_->add(motion);
}

void ompl::control::SyclopRRT::selectAndExtend(int region, std::set<Motion*> newMotions)
{

}

void ompl::control::SyclopRRT::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            if (motions[i]->control)
                siC_->freeControl(motions[i]->control);
            delete motions[i];
        }
    }
}
