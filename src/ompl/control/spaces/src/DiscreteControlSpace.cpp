/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include "ompl/control/spaces/DiscreteControlSpace.h"
#include "ompl/util/Exception.h"

void ompl::control::DiscreteControlSampler::sample(Control *control)
{
    control->as<DiscreteControlSpace::ControlType>()->value = rng_.uniformInt(
        space_->as<DiscreteControlSpace>()->getLowerBound(), space_->as<DiscreteControlSpace>()->getUpperBound());
}

unsigned int ompl::control::DiscreteControlSpace::getDimension() const
{
    return 1;
}

void ompl::control::DiscreteControlSpace::copyControl(Control *destination, const Control *source) const
{
    destination->as<ControlType>()->value = source->as<ControlType>()->value;
}

bool ompl::control::DiscreteControlSpace::equalControls(const Control *control1, const Control *control2) const
{
    return control1->as<ControlType>()->value == control2->as<ControlType>()->value;
}

ompl::control::ControlSamplerPtr ompl::control::DiscreteControlSpace::allocDefaultControlSampler() const
{
    return std::make_shared<DiscreteControlSampler>(this);
}

ompl::control::Control *ompl::control::DiscreteControlSpace::allocControl() const
{
    return new ControlType();
}

void ompl::control::DiscreteControlSpace::freeControl(Control *control) const
{
    delete static_cast<ControlType *>(control);
}

void ompl::control::DiscreteControlSpace::nullControl(Control *control) const
{
    control->as<ControlType>()->value = lowerBound_;
}

void ompl::control::DiscreteControlSpace::printControl(const Control *control, std::ostream &out) const
{
    out << "DiscreteControl [";
    if (control != nullptr)
        out << control->as<ControlType>()->value;
    else
        out << "nullptr";
    out << ']' << std::endl;
}

void ompl::control::DiscreteControlSpace::printSettings(std::ostream &out) const
{
    out << "Discrete control space '" << getName() << "' with bounds [" << lowerBound_ << ", " << upperBound_ << "]"
        << std::endl;
}

void ompl::control::DiscreteControlSpace::setup()
{
    if (lowerBound_ > upperBound_)
        throw Exception("Lower bound cannot be larger than upper bound for a discrete space");
    ControlSpace::setup();
}

unsigned int ompl::control::DiscreteControlSpace::getSerializationLength() const
{
    return sizeof(int);
}

void ompl::control::DiscreteControlSpace::serialize(void *serialization, const Control *ctrl) const
{
    memcpy(serialization, &ctrl->as<ControlType>()->value, sizeof(int));
}

void ompl::control::DiscreteControlSpace::deserialize(Control *ctrl, const void *serialization) const
{
    memcpy(&ctrl->as<ControlType>()->value, serialization, sizeof(int));
}
