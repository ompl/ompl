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

#include "ompl/util/MDP.h"
#include "ompl/util/Exception.h"
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <limits>
#include <cmath>
#include <set>

void ompl::MDP::clear(void)
{
    act_.clear();
    act_.resize(stateCount_);
    stateNames_.clear();
    actionNames_.clear();
    clearPolicy();
}

void ompl::MDP::clearPolicy(void)
{
    Pi_.clear();
    V_.clear();
    err_ = 0.0;
    steps_ = 0;
}

void ompl::MDP::addPossibleAction(std::size_t from, std::size_t to, std::size_t act,
                                  double prob, double reward)
{
    act_[from][act][to] = std::make_pair(prob, reward);
}

double ompl::MDP::getActionProbability(std::size_t from, std::size_t to, std::size_t act) const
{
    if (from < act_.size())
    {
        PossibleActions::const_iterator it = act_[from].find(act);
        if (it != act_[from].end())
        {
            PossibleDestinations::const_iterator jt = it->second.find(to);
            if (jt != it->second.end())
                return jt->second.first;
        }
    }
    return 0.0;
}

double ompl::MDP::getActionReward(std::size_t from, std::size_t to, std::size_t act) const
{
    if (from < act_.size())
    {
        PossibleActions::const_iterator it = act_[from].find(act);
        if (it != act_[from].end())
        {
            PossibleDestinations::const_iterator jt = it->second.find(to);
            if (jt != it->second.end())
                return jt->second.second;
        }
    }
    return 0.0;
}

void ompl::MDP::setStateName(std::size_t state, const std::string &name)
{
    stateNames_[state] = name;
}

void ompl::MDP::setActionName(std::size_t act, const std::string &name)
{
    actionNames_[act] = name;
}

bool ompl::MDP::valueIteration(double tolerance, unsigned int maxSteps)
{
    clearPolicy();

    V_.resize(stateCount_, 0.0);
    Pi_.resize(stateCount_, -1);

    err_ = std::numeric_limits<double>::infinity();
    while (err_ > tolerance && steps_ < maxSteps)
    {
        ++steps_;
        err_ = 0.0;
        for (std::size_t s = 0 ; s < stateCount_ ; ++s)
        {
            int maxA = -1;
            double maxVA = -std::numeric_limits<double>::infinity();
            const PossibleActions &a = act_[s];
            for (PossibleActions::const_iterator it = a.begin() ; it != a.end() ; ++it)
            {
                double va = 0.0;
                const PossibleDestinations &d = it->second;
                for (PossibleDestinations::const_iterator jt = d.begin() ; jt != d.end() ; ++jt)
                    va += jt->second.first * (jt->second.second + discount_ * V_[jt->first]);
                if (va > maxVA)
                {
                    maxA = it->first;
                    maxVA = va;
                }
            }
            if (maxA >= 0)
            {
                err_ += fabs(V_[s] - maxVA);
                V_[s] = maxVA;
                Pi_[s] = maxA;
            }
        }
    }
    return err_ < tolerance;
}

void ompl::MDP::normalize(void)
{
    for (std::size_t i = 0 ; i < act_.size() ; ++i)
    {
        PossibleActions &a = act_[i];
        for (PossibleActions::iterator it = a.begin() ; it != a.end() ; ++it)
        {
            double p = 0.0;
            PossibleDestinations &d = it->second;
            for (PossibleDestinations::iterator jt = d.begin() ; jt != d.end() ; ++jt)
                p += jt->second.first;
            if (p > std::numeric_limits<double>::epsilon())
                for (PossibleDestinations::iterator jt = d.begin() ; jt != d.end() ; ++jt)
                    jt->second.first /= p;
        }
    }
}

bool ompl::MDP::check(void) const
{
    bool valid = true;

    for (std::size_t i = 0 ; i < act_.size() ; ++i)
    {
        const PossibleActions &a = act_[i];
        for (PossibleActions::const_iterator it = a.begin() ; it != a.end() ; ++it)
        {
            double p = 0.0;
            const PossibleDestinations &d = it->second;
            for (PossibleDestinations::const_iterator jt = d.begin() ; jt != d.end() ; ++jt)
            {
                if (jt->second.first < 0.0)
                    msg_.error("Negative probabiliy found from action %u from state %u: %lf", it->first, i, jt->second.first);
                p += jt->second.first;
            }
            if (fabs(p - 1.0) > 1e-12)
            {
                valid = false;
                msg_.error("Action %u from state %u has probabilities summing up to %lf instead of 1.0", it->first, i, p);
            }
        }
    }

    return valid;
}

bool ompl::MDP::extractRewardingPath(std::size_t start, std::size_t goal, std::vector<std::size_t> &s, std::vector<std::size_t> &a) const
{
    s.clear();
    a.clear();
    if (Pi_.empty() || V_.empty())
    {
        msg_.error("MDP not solved");
        return false;
    }
    else
    {
        std::set<std::size_t> seen;
        std::size_t st = start;
        bool found = true;
        while (found)
        {
            s.push_back(st);
            if (st == goal)
                break;
            if (seen.find(st) != seen.end())
            {
                msg_.debug("Cycle found as part of MDP policy");
                return false;
            }
            seen.insert(st);

            std::size_t ac = Pi_[st];
            a.push_back(ac);

            PossibleActions::const_iterator it = act_[st].find(ac);
            if (it == act_[st].end())
                throw Exception("Unknown error in extracting MDP path. Please file a ticket.");
            double maxR = -std::numeric_limits<double>::infinity();
            bool found = false;
            for (PossibleDestinations::const_iterator jt = it->second.begin() ; jt != it->second.end() ; ++jt)
            {
                double r = jt->second.first * jt->second.second;
                if (r > maxR)
                {
                    maxR = r;
                    st = jt->first;
                    found = true;
                }
            }
            if (!found)
            {
                msg_.error("No action to perform at state %u", st);
                return false;
            }
        }
        return true;
    }
}

void ompl::MDP::printGraph(std::ostream &out) const
{
    out << "digraph MDP {" << std::endl;

    class NName
    {
    public:

        NName(const MDP &mdp) : mdp_(mdp)
        {
        }

        std::string operator()(std::size_t n)
        {
            std::map<std::size_t, std::string>::const_iterator s_it = mdp_.stateNames_.find(n);
            std::string sName = s_it != mdp_.stateNames_.end() ? s_it->second : boost::lexical_cast<std::string>(n);

            if (mdp_.V_.size() > n && mdp_.Pi_.size() > n)
            {
                std::map<std::size_t, std::string>::const_iterator a_it = mdp_.actionNames_.find(mdp_.Pi_[n]);
                std::string aName = a_it != mdp_.actionNames_.end() ? a_it->second : boost::lexical_cast<std::string>(mdp_.Pi_[n]);
                return "\"" + sName + ": " + aName + ", " + boost::lexical_cast<std::string>(mdp_.V_[n]) + "\"";
            }
            else
                return "\"" + sName + "\"";
        }

    private:
        const MDP &mdp_;
    };

    NName nm(*this);

    for (std::size_t i = 0 ; i < act_.size() ; ++i)
    {
        const PossibleActions &a = act_[i];
        for (PossibleActions::const_iterator it = a.begin() ; it != a.end() ; ++it)
        {
            const PossibleDestinations &d = it->second;
            std::map<std::size_t, std::string>::const_iterator a_it = actionNames_.find(it->first);
            std::string aName = a_it != actionNames_.end() ? a_it->second : boost::lexical_cast<std::string>(it->first);
            for (PossibleDestinations::const_iterator jt = d.begin() ; jt != d.end() ; ++jt)
            {
                out << nm(i) << " -> " << nm(jt->first) << " [label=\"" << aName
                    << "; " << jt->second.first << "; " << jt->second.second << "\"]" << std::endl;
            }
        }
    }
    out << "}" << std::endl;
}

void ompl::MDP::printSettings(std::ostream &out) const
{
    out << "MDP with " << stateCount_ << " states and " << actionCount_ << " actions. Discount factor = " << discount_ << std::endl;

    for (std::size_t i = 0 ; i < act_.size() ; ++i)
    {
        out << "  Actions from state " << i;
        std::map<std::size_t, std::string>::const_iterator s_it = stateNames_.find(i);
        if (s_it != stateNames_.end())
            out << "(" << s_it->second << ")";
        out << ":" << std::endl;
        const PossibleActions &a = act_[i];
        for (PossibleActions::const_iterator it = a.begin() ; it != a.end() ; ++it)
        {
            out << "    - action " << it->first;
            std::map<std::size_t, std::string>::const_iterator a_it = actionNames_.find(it->first);
            if (s_it != actionNames_.end())
                out << "(" << a_it->second << ")";
            out << " can lead to:" << std::endl;
            const PossibleDestinations &d = it->second;
            for (PossibleDestinations::const_iterator jt = d.begin() ; jt != d.end() ; ++jt)
            {
                out << "       state " << jt->first << " (";
                s_it = stateNames_.find(jt->first);
                if (s_it != stateNames_.end())
                    out << s_it->second << ", ";
                out << "probability " << jt->second.first
                    << ", reward " << jt->second.second << ")" << std::endl;
            }
        }
    }

    out << "Values: ";
    for (std::size_t i = 0 ; i < V_.size() ; ++i)
        out << "V[" << i << "]=" << V_[i] << " ";
    out << std::endl;

    out << "Policy: ";
    for (std::size_t i = 0 ; i < Pi_.size() ; ++i)
        out << "A[" << i << "]=" << Pi_[i] << " ";
    out << std::endl;
    out << "Error = " << err_ << ", after " << steps_ << " steps" << std::endl;
}
