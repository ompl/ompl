#include "ompl/base/spaces/MobiusStateSpace.h"
#include "ompl/tools/config/MagicConstants.h"
#include <cstring>
#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants; //pi
using namespace ompl::base;

MobiusStateSpace::MobiusStateSpace(double intervalMax)
{
    setName("Mobius" + getName());
    type_ = STATE_SPACE_UNKNOWN;

    StateSpacePtr SO2(std::make_shared<SO2StateSpace>());
    StateSpacePtr R1(std::make_shared<RealVectorStateSpace>(1));
    R1->as<RealVectorStateSpace>()->setBounds(-intervalMax, +intervalMax);

    addSubspace(SO2, 1.0);
    addSubspace(R1, 1.0);
    lock();
}

double MobiusStateSpace::distance(const State *state1, const State *state2) const
{
    double theta1 = state1->as<MobiusStateSpace::StateType>()->getS1();
    double theta2 = state2->as<MobiusStateSpace::StateType>()->getS1();

    double diff = theta2 - theta1;

    if (fabs(diff) <= pi)
    {
        return CompoundStateSpace::distance(state1, state2);
    }else{
        //requires interpolation over the gluing strip 
        const auto *cstate1 = static_cast<const CompoundState *>(state1);
        const auto *cstate2 = static_cast<const CompoundState *>(state2);

        //distance on S1 as usual
        double dist = 0.0;
        dist += weights_[0] * components_[0]->distance(cstate1->components[0], cstate2->components[0]);

        double r1 = state1->as<MobiusStateSpace::StateType>()->getR1();
        double r2 = state2->as<MobiusStateSpace::StateType>()->getR1();

        r2 = -r2;

        dist += sqrt((r2 - r1)*(r2 - r1));
        return dist;
    }
}

void MobiusStateSpace::interpolate( 
    const State *from, 
    const State *to, 
    double t, 
    State *state) const
{
    double theta1 = from->as<MobiusStateSpace::StateType>()->getS1();
    double theta2 = to->as<MobiusStateSpace::StateType>()->getS1();

    double diff = theta2 - theta1;

    if (fabs(diff) <= pi)
    {
        //interpolate as it would be a cylinder
        CompoundStateSpace::interpolate(from, to, t, state);
    }else{
        //requires interpolation over the gluing strip 
        const auto *cfrom = static_cast<const CompoundState *>(from);
        const auto *cto = static_cast<const CompoundState *>(to);
        auto *cstate = static_cast<CompoundState *>(state);

        //interpolate S1 as usual
        components_[0]->interpolate(
            cfrom->components[0], cto->components[0], t, cstate->components[0]);

        double r1 = from->as<MobiusStateSpace::StateType>()->getR1();
        double r2 = to->as<MobiusStateSpace::StateType>()->getR1();

        //Need to mirror point for interpolation
        r2 = -r2;

        double r = r1 + (r2 - r1) * t;

        //check again if we need to invert (only if we already crossed gluing
        //line)
        double thetaNew = state->as<MobiusStateSpace::StateType>()->getS1();
        double diff2 = theta2 - thetaNew;

        if (fabs(diff2) <= pi)
        {
          r = -r;
        }

        state->as<MobiusStateSpace::StateType>()->setR1(r);

    }

}

State *MobiusStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void MobiusStateSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

