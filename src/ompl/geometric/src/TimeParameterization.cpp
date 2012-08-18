/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

#include "ompl/geometric/TimeParameterization.h"
#include <algorithm>
#include <cmath>
#include "ompl/base/spaces/SE2StateSpace.h"
#include <boost/math/special_functions/sign.hpp>
#include <boost/math/constants/constants.hpp>

bool ompl::geometric::TimeParameterization::computeFastTimeParametrization(const PathGeometric &path, double maxVel, double maxAcc, std::vector<double> &times, std::vector<double> &velocities,
                                                                           double startVel, double endVel, unsigned int maxSteps) const
{
    std::map<std::string, double> maxVelMap;
    maxVelMap[path.getSpaceInformation()->getStateSpace()->getName()] = maxVel;
    std::map<std::string, double> maxAccMap;
    maxAccMap[path.getSpaceInformation()->getStateSpace()->getName()] = maxAcc;
    std::map<std::string, std::vector<double> > vel;    
    if (computeFastTimeParametrization(path, maxVelMap, maxAccMap, times, vel, startVel, endVel, maxSteps))
    {
        velocities.swap(vel.begin()->second);
        return true;
    }
    else
        return false;
}

bool ompl::geometric::TimeParameterization::computeFastTimeParametrization(const PathGeometric &path,
                                                                           const std::map<std::string, double> &maxVel, 
                                                                           const std::map<std::string, double> &maxAcc, 
                                                                           std::vector<double> &times, std::map<std::string, std::vector<double> > &velocities, 
                                                                           double startVel, double endVel, unsigned int maxSteps) const
{
    std::map<std::string, double> startVelMap;
    std::map<std::string, double> endVelMap;
    for (std::map<std::string, double>::const_iterator it = maxVel.begin() ; it != maxVel.end() ; ++it)
    {
      startVelMap[it->first] = startVel;
      endVelMap[it->first] = endVel;
    }
    return computeFastTimeParametrization(path, maxVel, maxAcc, times, velocities, startVelMap, endVelMap, maxSteps);
}

bool ompl::geometric::TimeParameterization::computeFastTimeParametrization(const PathGeometric &path,
                                                                           const std::map<std::string, double> &maxVel,
                                                                           const std::map<std::string, double> &maxAcc,
                                                                           std::vector<double> &times,
                                                                           std::map<std::string, std::vector<double> > &velocities, 
                                                                           const std::map<std::string, double> &startVel,
                                                                           const std::map<std::string, double> &endVel,
                                                                           unsigned int maxSteps) const
{
    times.clear();
    velocities.clear();

    if (maxVel.size() != maxAcc.size() || startVel.size() != endVel.size() || maxVel.size() != startVel.size())
    {
        logError("The number of parameters for max accelerations/velocities and start/end velocities do not match.");
        return false;
    }
    
    std::map<std::string, std::vector<double> > timeMap;
    const std::map<std::string, base::StateSpace::SubstateLocation> &substates = path.getSpaceInformation()->getStateSpace()->getSubstateLocationsByName();
    for (std::map<std::string, double>::const_iterator it = maxVel.begin() ; it != maxVel.end() ; ++it)
    {
        std::map<std::string, base::StateSpace::SubstateLocation>::const_iterator sl_it = substates.find(it->first);
        if (sl_it == substates.end())
        {
          logError("Substate %s is not known. Cannot compute time parameterization.", it->first.c_str());
          return false;
        }
        std::map<std::string, double>::const_iterator acc_it = maxAcc.find(it->first);
        if (acc_it == maxAcc.end())
        {
          logError("Maximum acceleration for substate %s is not specified. Cannot compute time parameterization.", it->first.c_str());
          return false;
        }
        std::map<std::string, double>::const_iterator s_it = startVel.find(it->first);
        if (s_it == startVel.end())
        {
          logError("Start velocity for substate %s is not specified. Cannot compute time parameterization.", it->first.c_str());
          return false;
        }
        std::map<std::string, double>::const_iterator e_it = endVel.find(it->first);
        if (e_it == endVel.end())
        {
          logError("End velocity for substate %s is not specified. Cannot compute time parameterization.", it->first.c_str());
          return false;
        }
        // compute the time & velocity for this part
        if (!computeFastTimeParametrizationPart(path, sl_it->second, it->second, acc_it->second, timeMap[it->first], velocities[it->first], s_it->second, e_it->second, maxSteps))
          return false;
    }
    
    // take the time of the slowest part for each state 
    times.resize(path.getStateCount());
    if (!times.empty())
    {
        times[0] = 0.0;
        for (std::size_t i = 1 ; i < times.size() ; ++i)
        {
            double mt = 0.0;
            for (std::map<std::string, std::vector<double> >::const_iterator it = timeMap.begin() ; it != timeMap.end() ; ++it)
                if (it->second[i] > mt)
                    mt = it->second[i];
            times[i] = times[i-1] + mt ;
        }
    }
    
    const base::StateSpacePtr &ss = path.getSpaceInformation()->getStateSpace();

    for (std::map<std::string, std::vector<double> >::iterator it = velocities.begin() ; it != velocities.end() ; ++it)
    {   
        std::map<std::string, base::StateSpace::SubstateLocation>::const_iterator sl_it = substates.find(it->first);
        const base::StateSpace::SubstateLocation &sloc = sl_it->second;
      
        std::vector<double> L(path.getStateCount() - 1);
        for (std::size_t i = 0 ; i < L.size() ; ++i)
            L[i] = sloc.space->distance(ss->getSubstateAtLocation(path.getState(i), sloc), ss->getSubstateAtLocation(path.getState(i + 1), sloc));

        std::map<std::string, double>::const_iterator vel_it = maxVel.find(it->first);
      
        if (it->second.size() >1)
            for (std::size_t i = 1 ; i < it->second.size() -1; ++i)
            {
                double dt = times[i] - times[i-1];
                if (dt > std::numeric_limits<double>::epsilon())
                    it->second[i] = boost::math::sign(it->second[i]) * std::min(vel_it->second, L[i] / dt);
            }
    }
    
    return true;
}

static bool getTime(double v0, double vf, double dist, double maxvel, double maxacc,
                    double &possible_v0, double &possible_vf, double &t, bool forward)
{
    // the time needed to reach max velocity from initial velocity
    double t_acc = (maxvel - fabs(v0)) / maxacc;
  
    // the time needed to break from max velocity to desired final velocity
    double t_break = (maxvel - fabs(vf)) / maxacc;
  
    // the distance traveled if we only accelerate and decelerate
    double d_acc_decc = maxacc * (t_acc * t_acc - t_break * t_break) / 2.0 + fabs(v0) * t_acc + maxvel * t_break;
  
    // if that distance is less than what we have to travel, we just assume we travel at maximum velocity
    // for the middle part of the segment (we accelerate at max, we use acceleration as 0, then we decelerate at maximum)
    if (d_acc_decc - dist <= std::numeric_limits<double>::epsilon())
    {
        t = t_acc + t_break + (dist - d_acc_decc) / maxvel;
        return true;
    }
    else
    {
        // check if we can at least decelerate in time:
        if (vf <= v0)
        {
           double t_break_0f = (v0 - vf) / maxacc;
           double d_dec = v0 * t_break_0f - maxacc * t_break_0f * t_break_0f / 2.0;
            
            // we cannot decelerate in time;
            // with maximum deceleration, the best we can do in terms of reducing speed given the constraints,
            if (fabs(d_dec) - dist > std::numeric_limits<double>::epsilon())
            {
                if (forward)
                {
                    // compute the final velocity that can be possibly achieved, within limits
                    double abs_vf = sqrt(v0 * v0 - 2.0 * boost::math::sign(d_dec) * dist * maxacc);
                    if (vf - abs_vf < std::numeric_limits<double>::epsilon() && abs_vf - v0 < std::numeric_limits<double>::epsilon())
                        possible_vf = abs_vf;
                    else
                        possible_vf = -abs_vf;
                    assert(abs_vf <= maxvel + std::numeric_limits<double>::epsilon());
                }
                else
                {
                    double abs_v0 = sqrt(vf * vf + 2.0 * boost::math::sign(d_dec) * dist * maxacc);
                    if (vf - abs_v0 < std::numeric_limits<double>::epsilon() && abs_v0 - v0 < std::numeric_limits<double>::epsilon())
                        possible_v0 = abs_v0;
                    else
                        possible_v0 = -abs_v0;
                    assert(abs_v0 <= maxvel + std::numeric_limits<double>::epsilon());                
                }
                return false;
            }
            else
            {
                // we can decelerate in time; just compute how long it takes
                t = (v0 - vf) / maxacc;
                return true;
            }
        }
    
        // check if we can accelerate in time:
        else
        {
            double t_acc_f0 = (vf - v0) / maxacc;
            double d_acc = v0 * t_acc_f0 + maxacc * t_acc_f0 * t_acc_f0 / 2.0;
            if (fabs(d_acc) - dist > std::numeric_limits<double>::epsilon())
            {
                // we cannot accelerate in time;
                // with maximum acceleration, the best we can do is:

                if (forward)
                {     
                    // compute the final velocity that can be possibly achieved, within limits
                    double abs_vf = sqrt(v0 * v0 + 2.0 * boost::math::sign(d_acc) * dist * maxacc);
                    if (v0 - abs_vf < std::numeric_limits<double>::epsilon() && abs_vf - vf < std::numeric_limits<double>::epsilon())
                      possible_vf = abs_vf;
                    else
                      possible_vf = -abs_vf;
                    assert(abs_vf <= maxvel + std::numeric_limits<double>::epsilon());
                }
                else
                {
                    double abs_v0 = sqrt(vf * vf - 2.0 * boost::math::sign(d_acc) *  dist * maxacc);
                    if (v0 - abs_v0 < std::numeric_limits<double>::epsilon() && abs_v0 - vf < std::numeric_limits<double>::epsilon())
                        possible_v0 = abs_v0;
                    else
                        possible_v0 = -abs_v0;
                    assert(abs_v0 <= maxvel + std::numeric_limits<double>::epsilon());
                }
                return false;
            }
            else
            {
                // we can accelerate in time; just compute how long it takes
                t = (vf - v0) / maxacc;
                return true;
            }
        }
    }
}

bool ompl::geometric::TimeParameterization::computeFastTimeParametrizationPart(const PathGeometric &path,
                                                                               const base::StateSpace::SubstateLocation &sloc, 
                                                                               double maxVel, double maxAcc, 
                                                                               std::vector<double> &times, 
                                                                               std::vector<double> &velocities,
                                                                               double startVel, double endVel,
                                                                               unsigned int maxSteps) const
{
    //  This implementation greatly benefited from discussions with Kenneth Anderson (http://sites.google.com/site/kennethaanderson/)
    if (path.getStateCount() == 0)
    {
        times.clear();
        velocities.clear();
        return true;
    }
    if (path.getStateCount() == 1)
    {
        times.resize(1);
        velocities.resize(1);
        times[0] = 0.0;
        velocities[0] = 0.0;
        return true;
    }

    const base::StateSpacePtr &ss = path.getSpaceInformation()->getStateSpace();
    
    // compute the lengths of the segments along the path
    std::vector<double> L(path.getStateCount() - 1);
    for (std::size_t i = 0 ; i < L.size() ; ++i)
        L[i] = sloc.space->distance(ss->getSubstateAtLocation(path.getState(i), sloc), ss->getSubstateAtLocation(path.getState(i + 1), sloc));

    // the time for the first state is 0
    times.resize(path.getStateCount());
    times[0] = 0.0;

    // the velocity is maximum everywhere, except at endpoints
    velocities.resize(path.getStateCount());
    std::fill(velocities.begin(), velocities.end(), maxVel);
    velocities.front() = std::min(maxVel, startVel);
    velocities.back() = std::min(maxVel, endVel);

    // we should check if some of the veolcities should be set to a negative value.
    // this would be done 
    
    // compute the cosine of the angle between consecutive segments
    // and scale the maximum desired velocity by that value
    // this has the effect of stopping at very sharp turns
    // and ignoring straight lines
    for (std::size_t i = 1 ; i < L.size() ; ++i)
    {
        double a = L[i-1];
        if (a < std::numeric_limits<double>::epsilon() * 2.0)
            velocities[i] = 0.0;
        else
        {
            double b = L[i];
            double c = sloc.space->distance(ss->getSubstateAtLocation(path.getState(i - 1), sloc), ss->getSubstateAtLocation(path.getState(i + 1), sloc));
            double acosValue = (a*a + b*b - c*c) / (2.0*a*b);
            velocities[i] *= std::min(1.0, fabs(acosValue));
        }
    }
    
    bool valid = true;
    bool change = true;
    unsigned int steps = 0;
    while (change && steps <= maxSteps)
    {
        ++steps;
        change = false;
        valid = true;
        
        for (std::size_t i = 1 ; i < times.size() ; ++i)
        {
            double possible_v0, possible_vf, t;
            bool ok = getTime(velocities[i-1], velocities[i], L[i-1], maxVel, maxAcc, possible_v0, possible_vf, t, true);
            if (ok)
                times[i] = t;
            else
            {
              if (fabs(velocities[i]) > fabs(possible_vf))
                {
                    velocities[i] = possible_vf;
                    change = true;
                }
                valid = false;
            }
        }
        

        if (change || !valid)
        {
          for (int i = L.size() - 1 ; i >= 0 ; --i)
            {
                double possible_v0, possible_vf, t;
                if (!getTime(velocities[i], velocities[i+1], L[i], maxVel, maxAcc, possible_v0, possible_vf, t, false))
                {
                  if (fabs(velocities[i]) > fabs(possible_v0))
                    {
                        velocities[i] = possible_v0;
                        change = true;
                    }
                    valid = false;
                }
            }
        }
        
    }  
    return valid;
}
