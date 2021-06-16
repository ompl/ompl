#include <ompl/multilevel/planners/sparse/MultiSPARSImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"
#include <boost/math/constants/constants.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::multilevel;

MultiSPARSImpl::MultiSPARSImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("MultiSPARSImplLevel" + std::to_string(id_));
    randomWorkStates_.resize(5);
    getBundle()->allocStates(randomWorkStates_);

    setMetric("geodesic");
    setGraphSampler("visibilityregion");
    setImportance("exponential");

    firstRun_ = true;
    isInfeasible_ = false;

    // sparseDeltaFraction_ = 0.25;
    sparseDeltaFraction_ = 0.25;
    std::cout << getBundle()->getMaximumExtent() << std::endl;
    // sparseDeltaFraction_ = 0.1;

}

void MultiSPARSImpl::setup()
{
    BaseT::setup();
    setFindSectionStrategy(FindSectionType::PATTERN_DANCE);
    // setFindSectionStrategy(FindSectionType::SIDE_STEP);
}

void MultiSPARSImpl::clear()
{
    BaseT::clear();
    firstRun_ = true;
    isInfeasible_ = false;
}

MultiSPARSImpl::~MultiSPARSImpl()
{
    getBundle()->freeStates(randomWorkStates_);
}

double MultiSPARSImpl::getImportance() const
{
    return 1.0 / (consecutiveFailures_ + 1);
}

void MultiSPARSImpl::grow()
{
    if (firstRun_)
    {
        init();
        firstRun_ = false;

        getGoalPtr()->sampleGoal(qGoal_->state);
        addConfiguration(qGoal_);
        goalConfigurations_.push_back(qGoal_);


        findSection();
    }

    // if(pis_.getSampledGoalsCount() < getGoalPtr()->maxSampleCount())
    // {
    //     const base::State *state = pis_.nextGoal();
    //     Configuration qgoal = new Configuration(getBundle(), state);
    //     qgoal->isGoal = true;
    //     goalConfigurations_.push_back(qgoal);
    // }

    if (!sampleBundleValid(xRandom_->state))
    {
        return;
    }

    Configuration *xNew = new Configuration(getBundle(), xRandom_->state);

    addConfigurationConditional(xNew);

    if (!hasSolution_)
    {
        bool same_component = sameComponent(getStartIndex(), getGoalIndex());
        if (same_component)
        {
            hasSolution_ = true;
        }
    }
}

bool MultiSPARSImpl::hasConverged()
{
    bool progressFailure = (consecutiveFailures_ >= maxFailures_);
    if (progressFailure)
    {
        OMPL_INFORM("Converged with probability %f (no valid samples for %d rounds).",
        (1.0 - 1.0 / (double)consecutiveFailures_), 
        consecutiveFailures_);
    }
    return progressFailure;
}

bool MultiSPARSImpl::isInfeasible()
{
    bool progressFailure = ((consecutiveFailures_ >= maxFailures_) && !hasSolution_);
    if (progressFailure)
    {
        OMPL_INFORM("Infeasibility detected with probability %f (no valid samples for %d rounds).",
                    (1.0 - 1.0 / (double)consecutiveFailures_), consecutiveFailures_);
        isInfeasible_ = true;
    }
    return progressFailure;
}
