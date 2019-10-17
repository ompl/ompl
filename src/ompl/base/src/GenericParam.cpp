/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage
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

#include "ompl/base/GenericParam.h"
#include "ompl/util/Exception.h"
#include <limits>

namespace ompl
{
    namespace base
    {
        template <>
        bool SpecificParam<bool>::lexical_cast(const std::string &value) const
        {
            return !(value.empty() || value == "0" || value == "false" || value == "FALSE" || value == "False" ||
                     value == "f" || value == "F");
        }
        template <>
        int SpecificParam<int>::lexical_cast(const std::string &value) const
        {
            return std::stoi(value);
        }
        template <>
        long SpecificParam<long>::lexical_cast(const std::string &value) const
        {
            return std::stol(value);
        }
        template <>
        long long SpecificParam<long long>::lexical_cast(const std::string &value) const
        {
            return std::stoll(value);
        }
        template <>
        unsigned int SpecificParam<unsigned int>::lexical_cast(const std::string &value) const
        {
            return std::stoul(value);
        }
        template <>
        unsigned long SpecificParam<unsigned long>::lexical_cast(const std::string &value) const
        {
            return std::stoul(value);
        }
        template <>
        unsigned long long SpecificParam<unsigned long long>::lexical_cast(const std::string &value) const
        {
            return std::stoull(value);
        }
        template <>
        float SpecificParam<float>::lexical_cast(const std::string &value) const
        {
            return ompl::stof(value);
        }
        template <>
        double SpecificParam<double>::lexical_cast(const std::string &value) const
        {
            return ompl::stod(value);
        }
        template <>
        long double SpecificParam<long double>::lexical_cast(const std::string &value) const
        {
            return ompl::stold(value);
        }
        template <>
        char SpecificParam<char>::lexical_cast(const std::string &value) const
        {
            static const int minChar = std::numeric_limits<char>::min(), maxChar = std::numeric_limits<char>::max();
            int val = std::stoi(value);
            if (val < minChar || val > maxChar)
                throw std::invalid_argument("character value out of range");
            return val;
        }
        template <>
        std::string SpecificParam<std::string>::lexical_cast(const std::string &value) const
        {
            return value;
        }

    }  // namespace base
}  // namespace ompl

bool ompl::base::ParamSet::setParam(const std::string &key, const std::string &value)
{
    std::map<std::string, GenericParamPtr>::const_iterator it = params_.find(key);
    if (it != params_.end())
        return it->second->setValue(value);

    OMPL_ERROR("Parameter '%s' was not found", key.c_str());
    return false;
}

bool ompl::base::ParamSet::setParams(const std::map<std::string, std::string> &kv, bool ignoreUnknown)
{
    bool result = true;
    for (const auto &it : kv)
    {
        if (ignoreUnknown)
            if (!hasParam(it.first))
                continue;
        bool r = setParam(it.first, it.second);
        result = result && r;
    }
    return result;
}

bool ompl::base::ParamSet::getParam(const std::string &key, std::string &value) const
{
    auto it = params_.find(key);
    if (it != params_.end())
    {
        value = it->second->getValue();
        return true;
    }
    return false;
}

void ompl::base::ParamSet::getParamNames(std::vector<std::string> &params) const
{
    params.clear();
    params.reserve(params_.size());
    for (const auto &param : params_)
        params.push_back(param.first);
}

void ompl::base::ParamSet::getParamValues(std::vector<std::string> &vals) const
{
    std::vector<std::string> names;
    getParamNames(names);
    vals.resize(names.size());
    for (std::size_t i = 0; i < names.size(); ++i)
        vals[i] = params_.find(names[i])->second->getValue();
}

const std::map<std::string, ompl::base::GenericParamPtr> &ompl::base::ParamSet::getParams() const
{
    return params_;
}

const ompl::base::GenericParamPtr &ompl::base::ParamSet::getParam(const std::string &key) const
{
    static GenericParamPtr empty;
    auto it = params_.find(key);
    if (it != params_.end())
        return it->second;
    return empty;
}

void ompl::base::ParamSet::getParams(std::map<std::string, std::string> &params) const
{
    for (const auto &param : params_)
        params[param.first] = param.second->getValue();
}

bool ompl::base::ParamSet::hasParam(const std::string &key) const
{
    return params_.find(key) != params_.end();
}

ompl::base::GenericParam &ompl::base::ParamSet::operator[](const std::string &key)
{
    if (!hasParam(key))
        throw Exception("Parameter '%s' is not defined", key);
    return *getParam(key);
}

void ompl::base::ParamSet::include(const ParamSet &other, const std::string &prefix)
{
    const std::map<std::string, GenericParamPtr> &p = other.getParams();
    if (prefix.empty())
        for (const auto &it : p)
            params_[it.first] = it.second;
    else
        for (const auto &it : p)
            params_[prefix + "." + it.first] = it.second;
}

void ompl::base::ParamSet::add(const GenericParamPtr &param)
{
    params_[param->getName()] = param;
}

void ompl::base::ParamSet::remove(const std::string &name)
{
    params_.erase(name);
}

void ompl::base::ParamSet::clear()
{
    params_.clear();
}

void ompl::base::ParamSet::print(std::ostream &out) const
{
    for (const auto &param : params_)
        out << param.first << " = " << param.second->getValue() << std::endl;
}
