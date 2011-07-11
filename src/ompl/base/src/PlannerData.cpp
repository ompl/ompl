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

#include "ompl/base/PlannerData.h"

void ompl::base::PlannerData::clear(void)
{
    stateIndex.clear();
    states.clear();
    tags.clear();
    edges.clear();
    properties.clear();
    si.reset();
}

void ompl::base::PlannerData::tagState(const State *s, int tag)
{
    if (s != NULL)
    {
        std::map<const State*, unsigned int>::iterator it = stateIndex.find(s);
        if (it == stateIndex.end())
        {
            unsigned int p = states.size();
            states.push_back(s);
            tags.push_back(tag);
            stateIndex[s] = p;
            edges.resize(states.size());
        }
        else
            tags[it->second] = tag;
    }
}

int ompl::base::PlannerData::recordEdge(const State *s1, const State *s2)
{
    if (s1 == NULL || s2 == NULL)
    {
        const State *s = s1 == NULL ? s2 : s1;
        if (s != NULL)
        {
            std::map<const State*, unsigned int>::iterator it = stateIndex.find(s);
            if (it == stateIndex.end())
            {
                unsigned int p = states.size();
                states.push_back(s);
                tags.push_back(0);
                stateIndex[s] = p;
                edges.resize(states.size());
            }
        }
        return -1;
    }
    else
    {
        std::map<const State*, unsigned int>::iterator it1 = stateIndex.find(s1);
        std::map<const State*, unsigned int>::iterator it2 = stateIndex.find(s2);

        bool newEdge = false;

        unsigned int p1;
        if (it1 == stateIndex.end())
        {
            p1 = states.size();
            states.push_back(s1);
            tags.push_back(0);
            stateIndex[s1] = p1;
            edges.resize(states.size());
            newEdge = true;
        }
        else
            p1 = it1->second;

        unsigned int p2;
        if (it2 == stateIndex.end())
        {
            p2 = states.size();
            states.push_back(s2);
            tags.push_back(0);
            stateIndex[s2] = p2;
            edges.resize(states.size());
            newEdge = true;
        }
        else
            p2 = it2->second;

        // if we are not yet sure this is a new edge, we check indeed if this edge exists
        if (!newEdge)
        {
            newEdge = true;
            for (unsigned int i = 0 ; i < edges[p1].size() ; ++i)
                if (edges[p1][i] == p2)
                {
                    newEdge = false;
                    break;
                }
        }

        if (newEdge)
        {
            edges[p1].push_back(p2);
            return p1;
        }
        else
            return -1;
    }
}

void ompl::base::PlannerData::print(std::ostream &out) const
{
    out << states.size() << std::endl;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
        out << i << " (tag="<< tags[i] << "): ";
        if (si)
            si->printState(states[i], out);
        else
            out << states[i] << std::endl;
    }

    for (unsigned int i = 0 ; i < edges.size() ; ++i)
    {
        if (edges[i].empty())
            continue;
        out << i << ": ";
        for (unsigned int j = 0 ; j < edges[i].size() ; ++j)
            out << edges[i][j] << ' ';
        out << std::endl;
    }
}
