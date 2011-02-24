/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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


/** \author Ioan Sucan */

#include "ompl/util/Profiler.h"

ompl::Profiler* ompl::Profiler::Instance(void)
{
    static Profiler p(true, true);
    return &p;
}

#if ENABLE_PROFILING

#include <vector>
#include <algorithm>

void ompl::Profiler::start(void)
{
    lock_.lock();
    if (!running_)
    {
        tinfo_.set();
        running_ = true;
    }
    lock_.unlock();
}

void ompl::Profiler::stop(void)
{
    lock_.lock();
    if (running_)
    {
        tinfo_.update();
        running_ = false;
    }
    lock_.unlock();
}

void ompl::Profiler::clear(void)
{
    lock_.lock();
    data_.clear();
    tinfo_ = TimeInfo();
    if (running_)
        tinfo_.set();
    lock_.unlock();
}

void ompl::Profiler::event(const std::string &name, const unsigned int times)
{
    lock_.lock();
    data_[boost::this_thread::get_id()].events[name] += times;
    lock_.unlock();
}

void ompl::Profiler::begin(const std::string &name)
{
    lock_.lock();
    data_[boost::this_thread::get_id()].time[name].set();
    lock_.unlock();
}

void ompl::Profiler::end(const std::string &name)
{
    lock_.lock();
    data_[boost::this_thread::get_id()].time[name].update();
    lock_.unlock();
}

void ompl::Profiler::status(std::ostream &out, bool merge)
{
    stop();
    lock_.lock();
    printOnDestroy_ = false;

    out << std::endl;
    out << " *** Profiling statistics. Total counted time : " << time::seconds(tinfo_.total) << " seconds" << std::endl;

    if (merge)
    {
        PerThread combined;
        for (std::map<boost::thread::id, PerThread>::const_iterator it = data_.begin() ; it != data_.end() ; ++it)
        {
            for (std::map<std::string, unsigned long int>::const_iterator iev = it->second.events.begin() ; iev != it->second.events.end(); ++iev)
                combined.events[iev->first] += iev->second;
            for (std::map<std::string, TimeInfo>::const_iterator itm = it->second.time.begin() ; itm != it->second.time.end(); ++itm)
            {
                TimeInfo &tc = combined.time[itm->first];
                tc.total = tc.total + itm->second.total;
                tc.parts = tc.parts + itm->second.parts;
                if (tc.shortest > itm->second.shortest)
                    tc.shortest = itm->second.shortest;
                if (tc.longest < itm->second.longest)
                    tc.longest = itm->second.longest;
            }
        }
        printThreadInfo(out, combined);
    }
    else
        for (std::map<boost::thread::id, PerThread>::const_iterator it = data_.begin() ; it != data_.end() ; ++it)
        {
            out << "Thread " << it->first << ":" << std::endl;
            printThreadInfo(out, it->second);
        }
    lock_.unlock();
}

namespace ompl
{

    struct dEnv
    {
        std::string       name;
        unsigned long int value;
    };

    struct SortEnvByValue
    {
        bool operator()(const dEnv &a, const dEnv &b) const
        {
            return a.value > b.value;
        }
    };

    struct dTm
    {
        std::string  name;
        double       value;
    };

    struct SortTmByValue
    {
        bool operator()(const dTm &a, const dTm &b) const
        {
            return a.value > b.value;
        }
    };
}

void ompl::Profiler::printThreadInfo(std::ostream &out, const PerThread &data)
{
    double total = time::seconds(tinfo_.total);

    std::vector<dEnv> events;

    for (std::map<std::string, unsigned long int>::const_iterator iev = data.events.begin() ; iev != data.events.end() ; ++iev)
    {
        dEnv next = {iev->first, iev->second};
        events.push_back(next);
    }

    std::sort(events.begin(), events.end(), SortEnvByValue());

    for (unsigned int i = 0 ; i < events.size() ; ++i)
        out << events[i].name << ": " << events[i].value << std::endl;

    std::vector<dTm> time;

    for (std::map<std::string, TimeInfo>::const_iterator itm = data.time.begin() ; itm != data.time.end() ; ++itm)
    {
        dTm next = {itm->first, time::seconds(itm->second.total)};
        time.push_back(next);
    }

    std::sort(time.begin(), time.end(), SortTmByValue());

    double unaccounted = total;
    for (unsigned int i = 0 ; i < time.size() ; ++i)
    {
        const TimeInfo &d = data.time.find(time[i].name)->second;

        double tS = time::seconds(d.shortest);
        double tL = time::seconds(d.longest);
        out << time[i].name << ": " << time[i].value << "s (" << (100.0 * time[i].value/total) << "%), ["
            << tS << "s --> " << tL << " s], " << d.parts << " parts" << std::endl;
        unaccounted -= time[i].value;
    }
    out << "Unaccounted time : " << unaccounted << " (" << (100.0 * unaccounted / total) << " %)" << std::endl;

    out << std::endl;
}

#endif
