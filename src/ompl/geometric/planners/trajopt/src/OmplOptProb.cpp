/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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

/* Authors: Bryce Willey */

#include "ompl/geometric/planners/trajopt/OmplOptProb.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include <boost/format.hpp>
#include "ompl/trajopt/typedefs.hpp"

ompl::geometric::OmplOptProb::OmplOptProb(int nSteps, ompl::base::SpaceInformationPtr &si) :
    si_(si)
{
    // Get the space information and nSteps_ to make the array of variables.
    int dof = si_->getStateDimension();
    // Get the bounds of the state space (assume SE2 for now)
    // TODO: eventually use StateSpace magic to see if it is bounded or not.
    ompl::base::StateSpacePtr ss = si_->getStateSpace();
    double max_extent = ss->getMaximumExtent();
    std::vector<double> low;
    std::vector<double> high;
    if (max_extent == std::numeric_limits<double>::infinity()) {
        ompl::base::RealVectorBounds bounds(dof);
        for (int i = 0; i < nSteps; i++) {
            bounds.low[i] = -1 * std::numeric_limits<double>::infinity();
            bounds.high[i] = std::numeric_limits<double>::infinity();
        }
        low = bounds.low;
        high = bounds.high;
    } else {
        // Check for some specific spaces.
        ompl::base::SE2StateSpace *se2;
        ompl::base::RealVectorStateSpace *real;
        if ((se2 = dynamic_cast<ompl::base::SE2StateSpace *>(ss.get()))) {
            ompl::base::RealVectorBounds bounds = se2->getBounds();
            bounds.low.push_back(-1 * std::numeric_limits<double>::infinity());
            bounds.high.push_back(std::numeric_limits<double>::infinity());
            low = bounds.low;
            high = bounds.high;
        } else if ((real = dynamic_cast<ompl::base::RealVectorStateSpace *>(ss.get()))) {
            ompl::base::RealVectorBounds bounds = real->getBounds();
            low = bounds.low;
            high = bounds.high;
        }
    }
    std::vector<double> vlower, vupper;
    std::vector<std::string> names;
    for (int i = 0; i < nSteps; i++) {
        vlower.insert(vlower.end(), low.data(), low.data() + low.size());
        vupper.insert(vupper.end(), high.data(), high.data() + high.size());
        for (int j = 0; j < dof; j++) {
            names.push_back( (boost::format("j_%i_%i")%i%j).str());
        }
    }
    sco::VarVector trajvarvec = createVariables(names, vlower, vupper);
    traj_vars_ = trajopt::VarArray(nSteps, dof, trajvarvec.data());
}
