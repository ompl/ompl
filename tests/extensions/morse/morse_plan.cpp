/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Caleb Voss */

// Tests the OMPL MORSE extension without invoking MORSE

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"
#include "ompl/extensions/morse/MorseSimpleSetup.h"
#include "ompl/extensions/morse/MorseGoal.h"

#include <vector>

using namespace ompl;

class MyEnvironment : public base::MorseEnvironment
{
public:
    MyEnvironment(const unsigned int rigidBodies, const unsigned int controlDim,
        const std::vector<double> &controlBounds, const std::vector<double> &positionBounds,
        const std::vector<double> &linvelBounds, const std::vector<double> &angvelBounds)
        : base::MorseEnvironment(controlDim, controlBounds, rigidBodies, positionBounds, linvelBounds, angvelBounds,
            1.0/60, 30, 90)
    {
    }
    void readState(base::State *state)
    {
        // load fake state data into state
        base::MorseStateSpace::StateType *mstate = state->as<base::MorseStateSpace::StateType>();
        for (unsigned int i = 0; i < 4*rigidBodies_; i+=4)
        {
            double *pos = mstate->as<base::RealVectorStateSpace::StateType>(i)->values;
            double *lin = mstate->as<base::RealVectorStateSpace::StateType>(i+1)->values;
            double *ang = mstate->as<base::RealVectorStateSpace::StateType>(i+2)->values;
            for (unsigned int j = 0; j < 3; j++)
            {
                pos[j] = 1.0;
                lin[j] = 1.0;
                ang[j] = 1.0;
            }
            base::SO3StateSpace::StateType *quat = mstate->as<base::SO3StateSpace::StateType>(i+3);
            quat->w = 1.0;
            quat->x = 0.0;
            quat->y = 0.0;
            quat->z = 0.0;
        }

    }
    void writeState(const base::State *)
    {
        // nothing to do
    }
    void applyControl(const std::vector<double> &)
    {
        // nothing to do
    }
    void worldStep(double)
    {
        // nothing to do
    }
};

class MyGoal : public base::MorseGoal
{
public:
    MyGoal(base::SpaceInformationPtr si)
        : base::MorseGoal(si), c(0)
    {
    }
    bool isSatisfied_Py(const base::State *) const
    {
        // goal is "reached" the 10th time this is called
        distance_ = 10-(c++);
        if (distance_ == 0)
            return true;
        return false;
    }
private:
    mutable unsigned int c;
};

int main()
{
    // Control Bounds: velocity <= 10 m/s, turning angle <= ~pi/3
    std::vector<double> cbounds(4);
    cbounds[0] = -10;
    cbounds[1] = 10;
    cbounds[2] = -1;
    cbounds[3] = 1;
    // Position Bounds: stay inside 200x200x200 m cube at origin
    std::vector<double> pbounds(6);
    pbounds[0] = -100;
    pbounds[1] = 100;
    pbounds[2] = -100;
    pbounds[3] = 100;
    pbounds[4] = -100;
    pbounds[5] = 100;
    // Linear Velocity Bounds: velocity in any axis <= 10 m/s
    std::vector<double> lbounds(6);
    lbounds[0] = -10;
    lbounds[1] = 10;
    lbounds[2] = -10;
    lbounds[3] = 10;
    lbounds[4] = -10;
    lbounds[5] = 10;
    // Angular Velocity Bounds: rotation <= ~1 rps on every axis
    std::vector<double> abounds(6);
    abounds[0] = -6;
    abounds[1] = 6;
    abounds[2] = -6;
    abounds[3] = 6;
    abounds[4] = -6;
    abounds[5] = 6;
    auto env(std::make_shared<MyEnvironment>(2, 2, cbounds, pbounds, lbounds, abounds));

    auto ss(std::make_shared<control::MorseSimpleSetup>(env));

    auto g(std::make_shared<MyGoal>(ss->getSpaceInformation()));

    ss->setGoal(g);

    ss->control::SimpleSetup::solve(1.0);

    return 0;
}

