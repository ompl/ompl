/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Ryan Luna */

#ifndef OMPL_GEOMETRIC_CONSTRAINED_SIMPLE_SETUP_
#define OMPL_GEOMETRIC_CONSTRAINED_SIMPLE_SETUP_

#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/ConstrainedSpaceInformation.h"

namespace ompl
{

    namespace geometric
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ConstrainedSimpleSetup);
        /// @endcond

        /** \class ompl::geometric::ConstrainedSimpleSetupPtr
            \brief A boost shared pointer wrapper for ompl::geometric::ConstrainedSimpleSetup */

        /** \brief Create the set of classes typically needed to solve a
            constrained geometric problem */
        class ConstrainedSimpleSetup : public SimpleSetup
        {
        public:

            /** \brief Constructor needs the state space used for planning. */
            explicit
            ConstrainedSimpleSetup(const base::StateSpacePtr &space) : SimpleSetup(space)
            {
                csi_.reset(new base::ConstrainedSpaceInformation(space));
                //si_ = boost::static_pointer_cast<base::SpaceInformation>(csi_);
                si_ = csi_;
                pdef_.reset(new base::ProblemDefinition(si_));
                psk_.reset(new PathSimplifier(si_));
                params_.include(si_->params());
            }

            virtual ~ConstrainedSimpleSetup(void)
            {
            }

            /** \brief Get the current instance of the constrained space information */
            const base::ConstrainedSpaceInformationPtr& getConstrainedSpaceInformation(void) const
            {
                return csi_;
            }

        protected:

            /// The created constrained space information
            base::ConstrainedSpaceInformationPtr     csi_;
        };
    }

}
#endif
