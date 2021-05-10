/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, University of Oxford
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
 *   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Marlin Strub */

#include "ompl/geometric/planners/informedtrees/ABITstar.h"

namespace ompl
{
    namespace geometric
    {
        ABITstar::ABITstar(const ompl::base::SpaceInformationPtr &si, const std::string &name /*= "ABITstar"*/)
          : ompl::geometric::BITstar(si, name)
        {
            // Enable cascading rewirings.
            enableCascadingRewirings(true);

            // Set the default initial inflation factor to very high.
            setInitialInflationFactor(1000000.0);

            // Set the default inflation factor parameter to something reasonable.
            setInflationScalingParameter(10.0);

            // Set the default truncation factor parameter to something reasonable.
            setTruncationScalingParameter(5.0);

            // Declare the planner parameters.
            Planner::declareParam<double>("initial_inflation_factor", this, &ABITstar::setInitialInflationFactor,
                                          &ABITstar::getInitialInflationFactor, "1.0:0.01:1000000.0");
            Planner::declareParam<double>("inflation_scaling_parameter", this, &ABITstar::setInflationScalingParameter,
                                          &ABITstar::getInflationScalingParameter, "1.0:0.01:1000000.0");
            Planner::declareParam<double>("truncation_scaling_parameter", this, &ABITstar::setTruncationScalingParameter,
                                          &ABITstar::getTruncationScalingParameter, "1.0:0.01:1000000.0");
        }

        void ABITstar::setInitialInflationFactor(double factor)
        {
            BITstar::setInitialInflationFactor(factor);
        }

        void ABITstar::setInflationScalingParameter(double factor)
        {
            BITstar::setInflationScalingParameter(factor);
        }

        void ABITstar::setTruncationScalingParameter(double factor)
        {
            BITstar::setTruncationScalingParameter(factor);
        }

        double ABITstar::getInitialInflationFactor() const
        {
            return BITstar::getInitialInflationFactor();
        }

        double ABITstar::getInflationScalingParameter() const
        {
            return BITstar::getInflationScalingParameter();
        }

        double ABITstar::getTruncationScalingParameter() const
        {
            return BITstar::getTruncationScalingParameter();
        }

        double ABITstar::getCurrentInflationFactor() const
        {
            return BITstar::getCurrentInflationFactor();
        }

        double ABITstar::getCurrentTruncationFactor() const
        {
            return BITstar::getCurrentTruncationFactor();
        }

    }  // namespace geometric

}  // namespace ompl
