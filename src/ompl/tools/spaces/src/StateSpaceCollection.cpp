/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

#include "ompl/tools/spaces/StateSpaceCollection.h"
#include "ompl/util/Exception.h"

const std::string& ompl::StateSpaceCollection::getName(void) const
{
    return name_;
}

void ompl::StateSpaceCollection::setName(const std::string &name)
{
    name_ = name;
}

void ompl::StateSpaceCollection::collect(const std::vector<base::StateSpacePtr> &spaces)
{
    for (std::size_t i = 0 ; i < spaces.size() ; ++i)
        collect(spaces[i]);
}

void ompl::StateSpaceCollection::collect(const base::StateSpacePtr &space)
{
    if (haveSpace(space))
        return;

    msg_.debug("Became aware of space '%s'", space->getName().c_str());

    spaces_.push_back(space);
    if (space->isCompound())
        collect(space->as<base::CompoundStateSpace>()->getSubSpaces());
}

const ompl::base::StateSpacePtr& ompl::StateSpaceCollection::getSpace(const std::string &name) const
{
    for (std::size_t i = 0 ; i < spaces_.size() ; ++i)
        if (spaces_[i]->getName() == name)
            return spaces_[i];
    throw Exception("Collection does not include state space '" + name + "'");
}

bool ompl::StateSpaceCollection::haveSpace(const std::string &name) const
{
    for (std::size_t i = 0 ; i < spaces_.size() ; ++i)
        if (spaces_[i]->getName() == name)
            return true;
    return false;
}

bool ompl::StateSpaceCollection::haveSpace(const base::StateSpacePtr &space) const
{
    return haveSpace(space->getName());
}

const ompl::base::StateSpacePtr& ompl::StateSpaceCollection::combine(const std::vector<base::StateSpacePtr> &components)
{
    std::vector<double> weights(components.size(), 1.0);
    return combine(components, weights);
}

const ompl::base::StateSpacePtr& ompl::StateSpaceCollection::combine(const std::vector<base::StateSpacePtr> &components, const std::vector<bool> &mask)
{
    std::vector<double> weights(components.size(), 1.0);
    return combine(components, mask, weights);
}

const ompl::base::StateSpacePtr& ompl::StateSpaceCollection::combine(const std::vector<base::StateSpacePtr> &components,
                                                                     const std::vector<bool> &mask, const std::vector<double> &weights)
{
    if (components.size() != mask.size())
        throw Exception("Number of components not equal to number of mask bits");
    if (components.size() != weights.size())
        throw Exception("Number of components not equal to number of weights");

    std::vector<base::StateSpacePtr> realComp;
    std::vector<double>              realW;
    for (std::size_t i = 0 ; i < components.size() ; ++i)
        if (mask[i])
        {
            realComp.push_back(components[i]);
            realW.push_back(weights[i]);
        }
    return combine(realComp, realW);
}

const ompl::base::StateSpacePtr& ompl::StateSpaceCollection::combine(const std::vector<base::StateSpacePtr> &components, const std::vector<double> &weights)
{
    if (components.empty())
        throw Exception("No spaces to combine");
    if (components.size() != weights.size())
        throw Exception("Number of components not equal to number of weights");

    collect(components);

    if (components.size() == 1)
        return getSpace(components[0]->getName());

    for (std::size_t i = 0 ; i < spaces_.size() ; ++i)
        if (spaces_[i]->isCompound())
            if (spaces_[i]->as<base::CompoundStateSpace>()->getSubSpaces() == components &&
                spaces_[i]->as<base::CompoundStateSpace>()->getSubSpaceWeights() == weights)
                return spaces_[i];

    base::StateSpacePtr space(new base::CompoundStateSpace(components, weights));
    std::string name = components[0]->getName();
    for (std::size_t i = 1 ; i < components.size() ; ++i)
        name += join_ + components[i]->getName();
    name = prefix_ + name + suffix_;
    if (!name_.empty())
        name = name_ + ":" + name;
    space->setName(name);

    msg_.debug("Created new state space: '%s'", space->getName().c_str());

    spaces_.push_back(space);
    return spaces_.back();
}

void ompl::StateSpaceCollection::setAutomaticNames(const std::string &join, const std::string &prefix, const std::string &suffix)
{
    join_ = join;
    prefix_ = prefix;
    suffix_ = suffix;
}

/// @cond IGNORE
namespace ompl
{
    static void generateBits(std::vector< std::vector<bool> > &options, std::vector<bool> &bits, unsigned int start)
    {
        if (start >= bits.size())
        {
            for (unsigned int i = 0 ; i < bits.size() ; ++i)
                if (bits[i])
                {
                    options.push_back(bits);
                    break;
                }
        }
        else
        {
            bits[start] = false;
            generateBits(options, bits, start + 1);
            bits[start] = true;
            generateBits(options, bits, start + 1);
        }
    }

    static std::vector< std::vector<bool> > generateCombinations(unsigned int size)
    {
        std::vector< std::vector<bool> > options;
        std::vector<bool> bits(size);
        generateBits(options, bits, 0);
        return options;
    }
}
/// @endcond

std::vector<ompl::base::StateSpacePtr> ompl::StateSpaceCollection::allCombinations(const std::vector<base::StateSpacePtr> &components, const std::vector<double> &weights)
{
    std::vector<base::StateSpacePtr> opt;
    const std::vector< std::vector<bool> > &comb = generateCombinations(components.size());
    for (std::size_t i = 0 ; i < comb.size() ; ++i)
        opt.push_back(combine(components, comb[i], weights));
    return opt;
}

std::vector<ompl::base::StateSpacePtr> ompl::StateSpaceCollection::allCombinations(const std::vector<base::StateSpacePtr> &components)
{
    std::vector<double> weights(components.size(), 1.0);
    return allCombinations(components, weights);
}
