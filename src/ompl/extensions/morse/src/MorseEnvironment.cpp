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

