#include "ompl/base/samplers/BridgeTestValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/tools/config/MagicConstants.h"

ompl::base::BridgeTestValidStateSampler::BridgeTestValidStateSampler(const SpaceInformation *si)
  : ValidStateSampler(si)
  , sampler_(si->allocStateSampler())
  , stddev_(si->getMaximumExtent() * magic::STD_DEV_AS_SPACE_EXTENT_FRACTION)
{
    name_ = "bridge_test";
    params_.declareParam<double>("standard_deviation", [this](double stddev) { setStdDev(stddev); },
                                 [this] { return getStdDev(); });
}

bool ompl::base::BridgeTestValidStateSampler::sample(State *state)
{
    unsigned int attempts = 0;
    bool valid = false;
    State *endpoint = si_->allocState();
    do
    {
        sampler_->sampleUniform(state);
        bool v1 = si_->isValid(state);
        if (!v1)
        {
            sampler_->sampleGaussian(endpoint, state, stddev_);
            bool v2 = si_->isValid(endpoint);
            if (!v2)
            {
                si_->getStateSpace()->interpolate(endpoint, state, 0.5, state);
                valid = si_->isValid(state);
            }
        }
        ++attempts;
    } while (!valid && attempts < attempts_);

    si_->freeState(endpoint);
    return valid;
}

bool ompl::base::BridgeTestValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    unsigned int attempts = 0;
    bool valid = false;
    State *endpoint = si_->allocState();
    do
    {
        sampler_->sampleUniformNear(state, near, distance);
        bool v1 = si_->isValid(state);
        if (!v1)
        {
            sampler_->sampleGaussian(endpoint, state, distance);
            bool v2 = si_->isValid(endpoint);
            if (!v2)
            {
                si_->getStateSpace()->interpolate(endpoint, state, 0.5, state);
                valid = si_->isValid(state);
            }
        }
        ++attempts;
    } while (!valid && attempts < attempts_);

    si_->freeState(endpoint);
    return valid;
}
