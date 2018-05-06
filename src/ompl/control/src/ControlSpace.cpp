/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

#include <utility>

#include "ompl/control/ControlSpace.h"
#include "ompl/util/Exception.h"

/// @cond IGNORE
namespace ompl
{
    namespace control
    {
        static void computeControlSpaceSignatureHelper(const ControlSpace *space, std::vector<int> &signature)
        {
            signature.push_back(space->getType());
            signature.push_back(space->getDimension());

            if (space->isCompound())
            {
                unsigned int c = space->as<CompoundControlSpace>()->getSubspaceCount();
                for (unsigned int i = 0; i < c; ++i)
                    computeControlSpaceSignatureHelper(space->as<CompoundControlSpace>()->getSubspace(i).get(),
                                                       signature);
            }
        }
    }
}
/// @endcond

ompl::control::ControlSpace::ControlSpace(base::StateSpacePtr stateSpace) : stateSpace_(std::move(stateSpace))
{
    name_ = "Control[" + stateSpace_->getName() + "]";
    type_ = CONTROL_SPACE_UNKNOWN;
}

ompl::control::ControlSpace::~ControlSpace() = default;

const std::string &ompl::control::ControlSpace::getName() const
{
    return name_;
}

void ompl::control::ControlSpace::setName(const std::string &name)
{
    name_ = name;
}

void ompl::control::ControlSpace::setup()
{
}

ompl::control::ControlSamplerPtr ompl::control::ControlSpace::allocControlSampler() const
{
    if (csa_)
        return csa_(this);
    return allocDefaultControlSampler();
}

void ompl::control::ControlSpace::setControlSamplerAllocator(const ControlSamplerAllocator &csa)
{
    csa_ = csa;
}

void ompl::control::ControlSpace::clearControlSamplerAllocator()
{
    csa_ = ControlSamplerAllocator();
}

double *ompl::control::ControlSpace::getValueAddressAtIndex(Control * /*control*/, const unsigned int /*index*/) const
{
    return nullptr;
}

void ompl::control::ControlSpace::printControl(const Control *control, std::ostream &out) const
{
    out << "Control instance: " << control << std::endl;
}

void ompl::control::ControlSpace::printSettings(std::ostream &out) const
{
    out << "ControlSpace '" << getName() << "' instance: " << this << std::endl;
}

unsigned int ompl::control::ControlSpace::getSerializationLength() const
{
    return 0;
}

void ompl::control::ControlSpace::serialize(void * /*serialization*/, const Control * /*ctrl*/) const
{
}

void ompl::control::ControlSpace::deserialize(Control * /*ctrl*/, const void * /*serialization*/) const
{
}

void ompl::control::ControlSpace::computeSignature(std::vector<int> &signature) const
{
    signature.clear();
    computeControlSpaceSignatureHelper(this, signature);
    signature.insert(signature.begin(), signature.size());
}

bool ompl::control::ControlSpace::isCompound() const
{
    return false;
}

void ompl::control::CompoundControlSpace::addSubspace(const ControlSpacePtr &component)
{
    if (locked_)
        throw Exception("This control space is locked. No further components can be added");

    components_.push_back(component);
    componentCount_ = components_.size();
}

unsigned int ompl::control::CompoundControlSpace::getSubspaceCount() const
{
    return componentCount_;
}

const ompl::control::ControlSpacePtr &ompl::control::CompoundControlSpace::getSubspace(const unsigned int index) const
{
    if (componentCount_ > index)
        return components_[index];
    throw Exception("Subspace index does not exist");
}

const ompl::control::ControlSpacePtr &ompl::control::CompoundControlSpace::getSubspace(const std::string &name) const
{
    for (const auto &component : components_)
        if (component->getName() == name)
            return component;
    throw Exception("Subspace " + name + " does not exist");
}

unsigned int ompl::control::CompoundControlSpace::getDimension() const
{
    unsigned int dim = 0;
    for (const auto &component : components_)
        dim += component->getDimension();
    return dim;
}

ompl::control::Control *ompl::control::CompoundControlSpace::allocControl() const
{
    auto *control = new CompoundControl();
    control->components = new Control *[componentCount_];
    for (unsigned int i = 0; i < componentCount_; ++i)
        control->components[i] = components_[i]->allocControl();
    return static_cast<Control *>(control);
}

void ompl::control::CompoundControlSpace::freeControl(Control *control) const
{
    auto *ccontrol = static_cast<CompoundControl *>(control);
    for (unsigned int i = 0; i < componentCount_; ++i)
        components_[i]->freeControl(ccontrol->components[i]);
    delete[] ccontrol->components;
    delete ccontrol;
}

void ompl::control::CompoundControlSpace::copyControl(Control *destination, const Control *source) const
{
    auto *cdest = static_cast<CompoundControl *>(destination);
    const auto *csrc = static_cast<const CompoundControl *>(source);
    for (unsigned int i = 0; i < componentCount_; ++i)
        components_[i]->copyControl(cdest->components[i], csrc->components[i]);
}

bool ompl::control::CompoundControlSpace::equalControls(const Control *control1, const Control *control2) const
{
    const auto *ccontrol1 = static_cast<const CompoundControl *>(control1);
    const auto *ccontrol2 = static_cast<const CompoundControl *>(control2);
    for (unsigned int i = 0; i < componentCount_; ++i)
        if (!components_[i]->equalControls(ccontrol1->components[i], ccontrol2->components[i]))
            return false;
    return true;
}

void ompl::control::CompoundControlSpace::nullControl(Control *control) const
{
    auto *ccontrol = static_cast<CompoundControl *>(control);
    for (unsigned int i = 0; i < componentCount_; ++i)
        components_[i]->nullControl(ccontrol->components[i]);
}

ompl::control::ControlSamplerPtr ompl::control::CompoundControlSpace::allocDefaultControlSampler() const
{
    auto ss(std::make_shared<CompoundControlSampler>(this));
    for (const auto &component : components_)
        ss->addSampler(component->allocControlSampler());
    return ss;
}

void ompl::control::CompoundControlSpace::lock()
{
    locked_ = true;
}

double *ompl::control::CompoundControlSpace::getValueAddressAtIndex(Control *control, const unsigned int index) const
{
    auto *ccontrol = static_cast<CompoundControl *>(control);
    unsigned int idx = 0;

    for (unsigned int i = 0; i < componentCount_; ++i)
        for (unsigned int j = 0; j <= index; ++j)
        {
            double *va = components_[i]->getValueAddressAtIndex(ccontrol->components[i], j);
            if (va != nullptr)
            {
                if (idx == index)
                    return va;
                idx++;
            }
            else
                break;
        }
    return nullptr;
}

void ompl::control::CompoundControlSpace::printControl(const Control *control, std::ostream &out) const
{
    out << "Compound control [" << std::endl;
    const auto *ccontrol = static_cast<const CompoundControl *>(control);
    for (unsigned int i = 0; i < componentCount_; ++i)
        components_[i]->printControl(ccontrol->components[i], out);
    out << "]" << std::endl;
}

void ompl::control::CompoundControlSpace::printSettings(std::ostream &out) const
{
    out << "Compound control space '" << getName() << "' [" << std::endl;
    for (unsigned int i = 0; i < componentCount_; ++i)
        components_[i]->printSettings(out);
    out << "]" << std::endl;
}

void ompl::control::CompoundControlSpace::setup()
{
    for (const auto &component : components_)
        component->setup();
    ControlSpace::setup();
}

unsigned int ompl::control::CompoundControlSpace::getSerializationLength() const
{
    unsigned int l = 0;
    for (const auto &component : components_)
        l += component->getSerializationLength();
    return l;
}

void ompl::control::CompoundControlSpace::serialize(void *serialization, const Control *ctrl) const
{
    const auto *compctrl = static_cast<const CompoundControl *>(ctrl);
    unsigned int l = 0;
    for (unsigned int i = 0; i < componentCount_; ++i)
    {
        components_[i]->serialize(reinterpret_cast<char *>(serialization) + l, compctrl->components[i]);
        l += components_[i]->getSerializationLength();
    }
}

void ompl::control::CompoundControlSpace::deserialize(Control *ctrl, const void *serialization) const
{
    auto *compctrl = static_cast<CompoundControl *>(ctrl);
    unsigned int l = 0;
    for (unsigned int i = 0; i < componentCount_; ++i)
    {
        components_[i]->deserialize(compctrl->components[i], reinterpret_cast<const char *>(serialization) + l);
        l += components_[i]->getSerializationLength();
    }
}

bool ompl::control::CompoundControlSpace::isCompound() const
{
    return true;
}
