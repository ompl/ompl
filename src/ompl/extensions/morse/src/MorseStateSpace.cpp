/* MorseStateSpace.cpp */

#include "ompl/extensions/morse/MorseStateSpace.h"
#include "ompl/util/Console.h"

#include <boost/lexical_cast.hpp>
#include <limits>
#include <queue>

ompl::base::MorseStateSpace::MorseStateSpace(const control::MorseEnvironmentPtr &env,
                                                  double positionWeight, double linVelWeight, double angVelWeight, double orientationWeight) :
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
    lock();
    setDefaultBounds();
}

void ompl::base::MorseStateSpace::setDefaultBounds(void)
{
    // limit all velocities to 1 m/s, 1 rad/s, respectively
    RealVectorBounds bounds1(3);
    bounds1.setLow(-1);
    bounds1.setHigh(1);
    setLinearVelocityBounds(bounds1);
    setAngularVelocityBounds(bounds1);

    // find the bounding box that contains all objects included in the collision space
    double mX, mY, mZ, MX, MY, MZ;
    mX = mY = mZ = std::numeric_limits<double>::infinity();
    MX = MY = MZ = -std::numeric_limits<double>::infinity();
    bool found = false;
    
    std::queue<double> extremes;
    for (unsigned int i = 0; i < env_->allBodies_; i++)
    {
        double bb[24];
        env_->getBoundBox(bb, i);
        for (unsigned char i = 0; i < 24; i++)
            extremes.push(bb[i]);
    }

    while (!extremes.empty())
    {
        double x = extremes.front();
        extremes.pop();
        double y = extremes.front();
        extremes.pop();
        double z = extremes.front();
        extremes.pop();
        
        found = true;
        if (x < mX) mX = x;
        if (x > MX) MX = x;
        if (y < mY) mY = y;
        if (y > MY) MY = y;
        if (z < mZ) mZ = z;
        if (z > MZ) MZ = z;
    }

    if (found)
    {
        double dx = MX - mX;
        double dy = MY - mY;
        double dz = MZ - mZ;
        double dM = std::max(dx, std::max(dy, dz));

        // add 10% in each dimension + 1% of the max dimension
        dx = dx / 10.0 + dM / 100.0;
        dy = dy / 10.0 + dM / 100.0;
        dz = dz / 10.0 + dM / 100.0;

        bounds1.low[0] = mX - dx;
        bounds1.high[0] = MX + dx;
        bounds1.low[1] = mY - dy;
        bounds1.high[1] = MY + dy;
        bounds1.low[2] = mZ - dz;
        bounds1.high[2] = MZ + dz;

        setVolumeBounds(bounds1);
    }
}

void ompl::base::MorseStateSpace::copyState(State *destination, const State *source) const
{
    CompoundStateSpace::copyState(destination, source);
    destination->as<StateType>()->validCollision = source->as<StateType>()->validCollision;
}

bool ompl::base::MorseStateSpace::satisfiesBoundsExceptRotation(const StateType *state) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (i % 4 != 3)
            if (!components_[i]->satisfiesBounds(state->components[i]))
                return false;
    return true;
}

void ompl::base::MorseStateSpace::setVolumeBounds(const RealVectorBounds &bounds)
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
    return state;
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
    env_->prepareStateRead();
    
    StateType *s = state->as<StateType>();
    for (int i = (int)env_->rigidBodies_ - 1 ; i >= 0 ; --i)
    {
        unsigned int _i4 = i * 4;

        double pos[3], vel[3], ang[3];
        env_->getPosition(pos, i);
        env_->getLinearVelocity(vel, i);
        env_->getAngularVelocity(ang, i);
        double *s_pos = s->as<RealVectorStateSpace::StateType>(_i4)->values; ++_i4;
        double *s_vel = s->as<RealVectorStateSpace::StateType>(_i4)->values; ++_i4;
        double *s_ang = s->as<RealVectorStateSpace::StateType>(_i4)->values; ++_i4;

        for (int j = 0; j < 3; ++j)
        {
            s_pos[j] = pos[j];
            s_vel[j] = vel[j];
            s_ang[j] = ang[j];
        }

        double rot[4];
        env_->getQuaternion(rot, i);
        SO3StateSpace::StateType &s_rot = *s->as<SO3StateSpace::StateType>(_i4);

        s_rot.w = rot[0];
        s_rot.x = rot[1];
        s_rot.y = rot[2];
        s_rot.z = rot[3];
    }
    s->validCollision = true;
}

void ompl::base::MorseStateSpace::writeState(const State *state) const
{
    const StateType *s = state->as<StateType>();
    for (int i = (int)env_->rigidBodies_ - 1; i >= 0 ; --i)
    {
        unsigned int _i4 = i * 4;

        double *s_pos = s->as<RealVectorStateSpace::StateType>(_i4)->values; ++_i4;
        env_->setPosition(i, s_pos);

        double *s_vel = s->as<RealVectorStateSpace::StateType>(_i4)->values; ++_i4;
        env_->setLinearVelocity(i, s_vel);

        double *s_ang = s->as<RealVectorStateSpace::StateType>(_i4)->values; ++_i4;
        env_->setAngularVelocity(i,  s_ang);

        const SO3StateSpace::StateType &s_rot = *s->as<SO3StateSpace::StateType>(_i4);
        double q[4];
        q[0] = s_rot.w;
        q[1] = s_rot.x;
        q[2] = s_rot.y;
        q[3] = s_rot.z;
        env_->setQuaternion(i, q);
    }
    
    env_->finalizeStateWrite();
}
