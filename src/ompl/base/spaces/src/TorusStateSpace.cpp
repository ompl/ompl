#include <ompl/base/spaces/TorusStateSpace.h>
#include <ompl/tools/config/MagicConstants.h>
#include <cstring>

#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants; //pi

using namespace ompl::base;

TorusStateSampler::TorusStateSampler(const StateSpace *space) : StateSampler(space)
{
}

void TorusStateSampler::sampleUniform(State *state)
{
    // https://stackoverflow.com/questions/26300510/generating-random-points-on-a-surface-of-an-n-dimensional-torus
    // Based on publication "Random selection of points distributed on curved surfaces."
    // Link: https://iopscience.iop.org/article/10.1088/0031-9155/32/10/009/pdf
    const TorusStateSpace* T = static_cast<const TorusStateSpace*>(space_);

    bool acceptedSampleFound = false;
    while(!acceptedSampleFound)
    {
        double u = rng_.uniformReal(-pi, pi);
        double v = rng_.uniformReal(-pi, pi);

        const double &R = T->getMajorRadius();
        const double &r = T->getMinorRadius();

        double vprime = (R + r*cos(v))/(R + r);

        double mu = rng_.uniformReal(0, 1);
        if(mu <= vprime)
        {
            TorusStateSpace::StateType *T = 
              state->as<TorusStateSpace::StateType>();
            T->setS1S2(u, v);
            acceptedSampleFound = true;
        }
    }
}

void TorusStateSampler::sampleUniformNear(State *state, const State *near, double distance) 
{
    TorusStateSpace::StateType *T = 
      state->as<TorusStateSpace::StateType>();
    const TorusStateSpace::StateType *Tnear = 
      near->as<TorusStateSpace::StateType>();
    T->setS1( rng_.uniformReal(Tnear->getS1() - distance, Tnear->getS1() + distance));
    T->setS2( rng_.uniformReal(Tnear->getS2() - distance, Tnear->getS2() + distance));
    space_->enforceBounds(state);
}

void TorusStateSampler::sampleGaussian(State *state, const State *mean, double stdDev) 
{
    TorusStateSpace::StateType *T = 
      state->as<TorusStateSpace::StateType>();
    const TorusStateSpace::StateType *Tmean = 
      mean->as<TorusStateSpace::StateType>();
    T->setS1( rng_.gaussian(Tmean->getS1(), stdDev) );
    T->setS2( rng_.gaussian(Tmean->getS2(), stdDev) );

    space_->enforceBounds(state);
}

TorusStateSpace::TorusStateSpace(double majorRadius, double minorRadius):
  majorRadius_(majorRadius), minorRadius_(minorRadius)
{
    setName("Torus" + getName());
    type_ = STATE_SPACE_UNKNOWN;
    addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
    addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
    lock();
}

StateSamplerPtr TorusStateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<TorusStateSampler>(this);
}

double TorusStateSpace::distance(const State *state1, const State *state2) const
{
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);
    double x = components_[0]->distance(cstate1->components[0], cstate2->components[0]);
    double y = components_[1]->distance(cstate1->components[1], cstate2->components[1]);
    return sqrtf(x*x + y*y);
}

State *TorusStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void TorusStateSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

double TorusStateSpace::getMajorRadius() const
{
    return majorRadius_;
}

double TorusStateSpace::getMinorRadius() const
{
    return minorRadius_;
}
