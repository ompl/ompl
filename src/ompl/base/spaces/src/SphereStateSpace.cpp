#include "ompl/base/spaces/SphereStateSpace.h"
#include "ompl/tools/config/MagicConstants.h"
#include <cstring>

using namespace ompl::base;
SphereStateSampler::SphereStateSampler(const StateSpace *space) : StateSampler(space)
{
}

void SphereStateSampler::sampleUniform(State *state) 
{
    //see for example http://corysimon.github.io/articles/uniformdistn-on-sphere/
    double theta = 2.0 * M_PI * rng_.uniformReal(0,1) - M_PI;
    double phi = acos(1.0 - 2.0 * rng_.uniformReal(0,1));
    SphereStateSpace::StateType *S = 
     state->as<SphereStateSpace::StateType>();
    S->setThetaPhi(theta, phi);
}

void SphereStateSampler::sampleUniformNear(State *state, const State *near, double distance) 
{
    SphereStateSpace::StateType *S = 
      state->as<SphereStateSpace::StateType>();
    const SphereStateSpace::StateType *Snear = 
      near->as<SphereStateSpace::StateType>();
    S->setTheta( rng_.uniformReal(Snear->getTheta() - distance, Snear->getTheta() + distance));
    S->setPhi( rng_.uniformReal(Snear->getPhi() - distance, Snear->getPhi() + distance));
    space_->enforceBounds(state);
}

void SphereStateSampler::sampleGaussian(State *state, const State *mean, double stdDev) 
{
    SphereStateSpace::StateType *S = 
      state->as<SphereStateSpace::StateType>();
    const SphereStateSpace::StateType *Smean = 
      mean->as<SphereStateSpace::StateType>();
    S->setTheta( rng_.gaussian(Smean->getTheta(), stdDev) );
    S->setPhi( rng_.gaussian(Smean->getPhi(), stdDev) );
    space_->enforceBounds(state);
}

StateSamplerPtr SphereStateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<SphereStateSampler>(this);
}

ompl::base::SphereStateSpace::SphereStateSpace()
{
    setName("Sphere" + getName());
    type_ = STATE_SPACE_UNKNOWN;
    addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
    addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
    lock();
}

double SphereStateSpace::distance(const State *state1, const State *state2) const
{
    //https://en.wikipedia.org/wiki/Great-circle_distance
    //https://en.wikipedia.org/wiki/Vincenty%27s_formulae

    const SphereStateSpace::StateType *S1 = 
      state1->as<SphereStateSpace::StateType>();
    const SphereStateSpace::StateType *S2 = 
      state2->as<SphereStateSpace::StateType>();

    double t1 = S1->getTheta();
    double p1 = S1->getPhi();

    double t2 = S2->getTheta();
    double p2 = S2->getPhi();

    double dt = t2 - t1;
    double d1 = powf(cos(p2)*sin(dt),2);
    double d2 = powf(cos(p1)*sin(p2)-sin(p1)*cos(p2)*cos(dt),2);

    double numerator = sqrtf(d1 + d2);

    double denumerator = sin(p1)*sin(p2)+cos(p1)*cos(p2)*cos(dt);

    return radius_ * atan2(numerator, denumerator);
}

double SphereStateSpace::getMeasure() const
{
    return 4*M_PI*radius_*radius_;
}

ompl::base::State *ompl::base::SphereStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::base::SphereStateSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

