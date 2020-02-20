/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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
*   * Neither the name of Rice University nor the names of its
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

#ifndef BOUNDED_PLANAR_MANIPULATOR_STATE_SPACE_H_
#define BOUNDED_PLANAR_MANIPULATOR_STATE_SPACE_H_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "PlanarManipulator.h" // i don't like this.  Need for cartesian interpolator.  Should probably move that code here...

// State space for a planar manipulator with bounded revolute joints
class BoundedPlanarManipulatorStateSpace : public ompl::base::RealVectorStateSpace
{
    public:
        BoundedPlanarManipulatorStateSpace(const PlanarManipulator* manip);

        virtual ~BoundedPlanarManipulatorStateSpace();

        virtual bool isMetricSpace() const;

        virtual bool hasSymmetricDistance() const;

        virtual bool hasSymmetricInterpolate() const;

        virtual double distance(const ompl::base::State* s1, const ompl::base::State* s2) const;

        virtual void interpolate(const ompl::base::State* from, const ompl::base::State* to, const double t, ompl::base::State* state) const;

        bool usingCartesianInterpolator() const;
        void useCartesianInterpolator(bool useIt);

        bool usingCartesianJointDistanceFn() const;
        void useCartesianJointDistance(bool useIt);

        bool usingHybridCartesianInterpolator() const;
        void useHybridCartesianInterpolator(bool useIt);

    protected:
        void linearInterpolate(const double& from, const double& to, const double& t, double& out) const;

        const PlanarManipulator* manip_;
        bool cartInterpolator_;
        bool cartDistance_;
        bool hybridCartInterpolator_;
};

#endif