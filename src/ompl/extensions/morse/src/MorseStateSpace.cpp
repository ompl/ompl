/* MorseStateSpace.cpp */

#include "ompl/extensions/morse/MorseStateSpace.h"

#include "ompl/base/spaces/DiscreteStateSpace.h"

#include <boost/lexical_cast.hpp>

ompl::base::MorseStateSpace::MorseStateSpace(const MorseEnvironmentPtr &env, double positionWeight, double linVelWeight,
                                             double angVelWeight, double orientationWeight) :
    CompoundStateSpace(), env_(env)
{
    setName("Morse" + getName());
    type_ = STATE_SPACE_TYPE_COUNT + 1;
    for (unsigned int i = 0 ; i < env_->rigidBodies_; ++i)
    {
        std::string body = ":B" + boost::lexical_cast<std::string>(i);

        addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), positionWeight); // position
        components_.back()->setName(components_.back()->getName() + body + ":position");

        addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), linVelWeight);   // linear velocity
        components_.back()->setName(components_.back()->getName() + body + ":linvel");

        addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), angVelWeight);   // angular velocity
        components_.back()->setName(components_.back()->getName() + body + ":angvel");

        addSubspace(StateSpacePtr(new SO3StateSpace()), orientationWeight);      // orientation
        components_.back()->setName(components_.back()->getName() + body + ":orientation");
    }
    // Add the goal region satisfaction flag as a subspace.
    addSubspace(StateSpacePtr(new DiscreteStateSpace(0, 1)), 0.01);
    components_.back()->setName(components_.back()->getName() + ":goalRegionSat");
    
    lock();
    setBounds();
}

void ompl::base::MorseStateSpace::setBounds(void)
{
    RealVectorBounds pbounds(3), lbounds(3), abounds(3);
    for (unsigned int i = 0; i < 3; i++)
    {
        pbounds.low[i] = env_->positionBounds_[2*i];
        pbounds.high[i] = env_->positionBounds_[2*i+1];
        lbounds.low[i] = env_->linvelBounds_[2*i];
        lbounds.high[i] = env_->linvelBounds_[2*i+1];
        abounds.low[i] = env_->angvelBounds_[2*i];
        abounds.high[i] = env_->angvelBounds_[2*i+1];
    }
    setPositionBounds(pbounds);
    setLinearVelocityBounds(lbounds);
    setAngularVelocityBounds(abounds);
}

void ompl::base::MorseStateSpace::copyState(State *destination, const State *source) const
{
    CompoundStateSpace::copyState(destination, source);
    destination->as<StateType>()->validCollision = source->as<StateType>()->validCollision;
}

bool ompl::base::MorseStateSpace::satisfiesBounds(const State *state) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (i % 4 != 3)
            if (!components_[i]->satisfiesBounds(state->as<CompoundStateSpace::StateType>()->components[i]))
                return false;
    return true;
}

void ompl::base::MorseStateSpace::setPositionBounds(const RealVectorBounds &bounds)
{
    for (unsigned int i = 0 ; i < env_->rigidBodies_; ++i)
        components_[i * 4]->as<RealVectorStateSpace>()->setBounds(bounds);
}

void ompl::base::MorseStateSpace::setLinearVelocityBounds(const RealVectorBounds &bounds)
{
    for (unsigned int i = 0 ; i < env_->rigidBodies_; ++i)
        components_[i * 4 + 1]->as<RealVectorStateSpace>()->setBounds(bounds);
}

void ompl::base::MorseStateSpace::setAngularVelocityBounds(const RealVectorBounds &bounds)
{
    for (unsigned int i = 0 ; i < env_->rigidBodies_; ++i)
        components_[i * 4 + 2]->as<RealVectorStateSpace>()->setBounds(bounds);
}

ompl::base::State* ompl::base::MorseStateSpace::allocState(void) const
{
    StateType *state = new StateType();
    allocStateComponents(state);
    return static_cast<State*>(state);
}

void ompl::base::MorseStateSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

// this function should most likely not be used with MORSE propagations, but just in case it is called, we need to make sure the collision information
// is cleared from the resulting state
void ompl::base::MorseStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    CompoundStateSpace::interpolate(from, to, t, state);
    state->as<StateType>()->validCollision = true;
}

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        // we need to make sure any collision information is cleared when states are sampled (just in case this ever happens)
        class WrapperForMorseSampler : public StateSampler
        {
        public:
            WrapperForMorseSampler(const StateSpace *space, const StateSamplerPtr &wrapped) : StateSampler(space), wrapped_(wrapped)
            {
            }

            virtual void sampleUniform(State *state)
            {
                wrapped_->sampleUniform(state);
                state->as<MorseStateSpace::StateType>()->validCollision = true;
            }

            virtual void sampleUniformNear(State *state, const State *near, const double distance)
            {
                wrapped_->sampleUniformNear(state, near, distance);
                state->as<MorseStateSpace::StateType>()->validCollision = true;
            }

            virtual void sampleGaussian(State *state, const State *mean, const double stdDev)
            {
                wrapped_->sampleGaussian(state, mean, stdDev);
                state->as<MorseStateSpace::StateType>()->validCollision = true;
            }
        private:
            StateSamplerPtr wrapped_;
        };
    }
}
/// @endcond

ompl::base::StateSamplerPtr ompl::base::MorseStateSpace::allocDefaultStateSampler(void) const
{
    StateSamplerPtr sampler = CompoundStateSpace::allocDefaultStateSampler();
    return StateSamplerPtr(new WrapperForMorseSampler(this, sampler));
}

ompl::base::StateSamplerPtr ompl::base::MorseStateSpace::allocStateSampler(void) const
{
    StateSamplerPtr sampler = CompoundStateSpace::allocStateSampler();
    if (dynamic_cast<WrapperForMorseSampler*>(sampler.get()))
        return sampler;
    else
        return StateSamplerPtr(new WrapperForMorseSampler(this, sampler));
}

void ompl::base::MorseStateSpace::readState(State *state) const
{
    env_->readState(state);
    // Bullet can get a little imprecise with the quaternions for OMPL's tastes
    // TODO: if we can trust Bullet not to get too crazy with them, maybe we can make a QuickSO3StateSpace that
    //   doesn't ever check the bounds, to save time
    for (unsigned int i = 0; i < env_->rigidBodies_*4; i+=4)
    {
        SO3StateSpace::StateType *quat = state->as<StateType>()->as<SO3StateSpace::StateType>(i+3);
        getSubspace(i+3)->as<SO3StateSpace>()->enforceBounds(quat);
    }
    state->as<StateType>()->validCollision = true;
}

void ompl::base::MorseStateSpace::writeState(const State *state) const
{
    env_->writeState(state);
}
