#include "ompl/base/GoalSampleableRegion.h"
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
    base::Goal* goal = getProblemDefinition()->getGoal().get();
    base::GoalSampleableRegion* goalSample = dynamic_cast<base::GoalSampleableRegion*>(goal);
    Motion* rmotion = new Motion(siC_);
    base::State* rstate = rmotion->state;
    Control* rctrl = rmotion->control;
    base::State* newState = si_->allocState();

    if (goalSample && rng.uniform01() < goalBias_ && goalSample->canSample())
        goalSample->sampleGoal(rstate);
    else
        sampler_->sampleUniform(rstate);

    Motion* nmotion = nn_->nearest(rmotion);
    controlSampler_->sampleNext(rctrl, nmotion->control, nmotion->state);
    unsigned int duration = controlSampler_->sampleStepCount(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
    duration = siC_->propagateWhileValid(nmotion->state, rctrl, duration, newState);

    if (duration >= siC_->getMinControlDuration())
    {
        Motion* motion = new Motion(siC_);
        si_->copyState(motion->state, newState);
        siC_->copyControl(motion->control, rctrl);
        motion->steps = duration;
        motion->parent = nmotion;
        nn_->add(motion);
        newMotions.insert(motion);
    }
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
