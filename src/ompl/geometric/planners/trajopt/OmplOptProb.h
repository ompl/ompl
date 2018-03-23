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

#ifndef OMPL_GEOMETRIC_PLANNERS_TRAJOPT_OMPLOPTPROB_
#define OMPL_GEOMETRIC_PLANNERS_TRAJOPT_OMPLOPTPROB_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/trajopt/modeling.h"
#include "ompl/trajopt/solver_interface.h"
#include "ompl/trajopt/typedefs.h"

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(OmplOptProb);

        class OmplOptProb : public sco::OptProb
        {
        public:
            // TODO: do we need this constructor?
            OmplOptProb()
            {
            }
            OmplOptProb(int nSteps, ompl::base::SpaceInformationPtr &si);

            void SetInitTraj(const sco::TrajArray &x)
            {
                init_traj_ = x;
                initTrajIsSet = true;
            }

            bool InitTrajIsSet()
            {
                return initTrajIsSet;
            }

            sco::TrajArray GetInitTraj()
            {
                return init_traj_;
            }

            sco::VarArray GetVars()
            {
                return traj_vars_;
            }

            sco::VarVector GetVarRow(int i)
            {
                return traj_vars_.row(i);
            }

            sco::Var &GetVar(int i, int j)
            {
                return traj_vars_.at(i, j);
            }

            sco::VarArray traj_vars_;

        private:
            bool initTrajIsSet = false;
            ompl::base::SpaceInformationPtr si_;
            sco::TrajArray init_traj_;
        };
    }
}

#endif
