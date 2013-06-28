/* MorseEnvironment.cpp */

#include "ompl/extensions/morse/MorseEnvironment.h"

void ompl::base::MorseEnvironment::getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const
{
    lower.resize(controlDim_);
    upper.resize(controlDim_);
    for (unsigned int i = 0; i < controlBounds_.size()/2; i++)
    {
        lower[i] = controlBounds_[2*i];
        upper[i] = controlBounds_[2*i+1];
    }
}

void ompl::base::MorseEnvironment::setPosition(const unsigned int obj, const double pos[3])
{
    positions[3*obj] = pos[0];
    positions[3*obj+1] = pos[1];
    positions[3*obj+2] = pos[2];
}

void ompl::base::MorseEnvironment::setLinearVelocity(const unsigned int obj, const double lin[3])
{
    linVelocities[3*obj] = lin[0];
    linVelocities[3*obj+1] = lin[1];
    linVelocities[3*obj+2] = lin[2];
}

void ompl::base::MorseEnvironment::setAngularVelocity(const unsigned int obj, const double ang[3])
{
    angVelocities[3*obj] = ang[0];
    angVelocities[3*obj+1] = ang[1];
    angVelocities[3*obj+2] = ang[2];
}

void ompl::base::MorseEnvironment::setQuaternion(const unsigned int obj, const double rot[4])
{
    quaternions[4*obj] = rot[0];
    quaternions[4*obj+1] = rot[1];
    quaternions[4*obj+2] = rot[2];
    quaternions[4*obj+3] = rot[3];
}

