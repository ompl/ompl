/* MorseEnvironment.cpp */

#include "ompl/extensions/morse/MorseEnvironment.h"

void ompl::control::MorseEnvironment::getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const
{
    lower = boundsLow_;
    upper = boundsHigh_;
}

void ompl::control::MorseEnvironment::getPosition(double result[3], unsigned int obj) const
{
    result[0] = positions[3*obj];
    result[1] = positions[3*obj+1];
    result[2] = positions[3*obj+2];
}

void ompl::control::MorseEnvironment::getLinearVelocity(double result[3], unsigned int obj) const
{
    result[0] = linVelocities[3*obj];
    result[1] = linVelocities[3*obj+1];
    result[2] = linVelocities[3*obj+2];
}

void ompl::control::MorseEnvironment::getAngularVelocity(double result[3], unsigned int obj) const
{
    result[0] = angVelocities[3*obj];
    result[1] = angVelocities[3*obj+1];
    result[2] = angVelocities[3*obj+2];
}

void ompl::control::MorseEnvironment::getQuaternion(double result[4], unsigned int obj) const
{
    result[0] = quaternions[4*obj];
    result[1] = quaternions[4*obj+1];
    result[2] = quaternions[4*obj+2];
    result[3] = quaternions[4*obj+3];
}


void ompl::control::MorseEnvironment::setPosition(unsigned int obj, double pos[3])
{
    positions[3*obj] = pos[0];
    positions[3*obj+1] = pos[1];
    positions[3*obj+2] = pos[2];
}

void ompl::control::MorseEnvironment::setLinearVelocity(unsigned int obj, double lin[3])
{
    linVelocities[3*obj] = lin[0];
    linVelocities[3*obj+1] = lin[1];
    linVelocities[3*obj+2] = lin[2];
}

void ompl::control::MorseEnvironment::setAngularVelocity(unsigned int obj, double ang[3])
{
    angVelocities[3*obj] = ang[0];
    angVelocities[3*obj+1] = ang[1];
    angVelocities[3*obj+2] = ang[2];
}

void ompl::control::MorseEnvironment::setQuaternion(unsigned int obj, double rot[4])
{
    quaternions[4*obj] = rot[0];
    quaternions[4*obj+1] = rot[1];
    quaternions[4*obj+2] = rot[2];
    quaternions[4*obj+3] = rot[3];
}

