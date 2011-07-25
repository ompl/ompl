#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/control/planners/syclop/SyclopRRT.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/base/spaces/SE2StateSpace.h"

ompl::control::SyclopRRT::SyclopRRT(const SpaceInformationPtr &si, Decomposition &d) :
    Syclop(si,d,"SyclopRRT"), sampler_(si_->allocStateSampler()),
    controlSampler_(siC_->allocControlSampler())
{
}

ompl::control::SyclopRRT::~SyclopRRT(void)
{
    freeMemory();
}

void ompl::control::SyclopRRT::setup(void)
{
    Syclop::setup();
}

void ompl::control::SyclopRRT::clear(void)
{
    Syclop::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    motions.clear();
}

void ompl::control::SyclopRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    for (std::size_t i = 0 ; i < motions.size() ; ++i)
        data.recordEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
}

ompl::control::Syclop::Motion* ompl::control::SyclopRRT::initializeTree(const base::State *s)
{
    Motion* motion = new Motion(siC_);
    si_->copyState(motion->state, s);
    siC_->nullControl(motion->control);
    motions.push_back(motion);
    return motion;
}

void ompl::control::SyclopRRT::selectAndExtend(Region& region, std::set<Motion*>& newMotions)
{
    std::cout << "selectAndExtend called from region " << region.index << std::endl;
    Motion* nmotion = region.motions[rng.uniformInt(0,region.motions.size()-1)];

    base::State* newState = si_->allocState();
    Control* rctrl = siC_->allocControl();

    controlSampler_->sampleNext(rctrl, nmotion->control, nmotion->state);
    unsigned int duration = controlSampler_->sampleStepCount(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
    duration = siC_->propagateWhileValid(nmotion->state, rctrl, duration, newState);

    base::CompoundState* cs = newState->as<base::CompoundState>();
    base::SE2StateSpace::StateType* location = cs->as<base::SE2StateSpace::StateType>(0);
    if (location->getX() >= 1.95 && location->getX() <= 2.05 && location->getY() >= -2.05 && location->getY() <= -1.95)
    {
        std::cerr << "syclopRRT generated a satisfying state at (" << location->getX() << "," << location->getY() << ")" << std::endl;
    }

    if (duration >= siC_->getMinControlDuration())
    {
        Motion* motion = new Motion(siC_);
        si_->copyState(motion->state, newState);
        siC_->copyControl(motion->control, rctrl);
        motion->steps = duration;
        motion->parent = nmotion;
        motions.push_back(motion);
        newMotions.insert(motion);
    }

    si_->freeState(newState);
    siC_->freeControl(rctrl);
}

void ompl::control::SyclopRRT::freeMemory(void)
{
    for (std::size_t i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->state)
            si_->freeState(motions[i]->state);
        if (motions[i]->control)
            siC_->freeControl(motions[i]->control);
        delete motions[i];
    }
}
