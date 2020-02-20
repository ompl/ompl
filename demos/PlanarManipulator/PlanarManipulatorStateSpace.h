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

#ifndef PLANAR_MANIPULATOR_STATE_SPACE_H_
#define PLANAR_MANIPULATOR_STATE_SPACE_H_

#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>

#include "PlanarManipulator.h"  // i don't like this.  Need for cartesian interpolator.  Should probably move that code here...

class PlanarManipulatorStateSampler : public ompl::base::StateSampler
{
public:
    PlanarManipulatorStateSampler(const ompl::base::StateSpace *space);
    virtual void sampleUniform(ompl::base::State *state);
    virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance);
    virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev);
};

namespace ompl
{
    namespace base
    {
        enum PlanarManipulatorStateSpaceType
        {
            STATE_SPACE_PLANAR_MANIPULATOR = STATE_SPACE_TYPE_COUNT + 42,  // 42, because 7 8 9
        };
    }
}  // namespace ompl

// State space for a planar manipulator with revolute joints (no joint limits)
class PlanarManipulatorStateSpace : public ompl::base::StateSpace
{
public:
    class StateType : public ompl::base::State
    {
    public:
        StateType() : State()
        {
        }

        double operator[](unsigned int i) const
        {
            return values[i];
        }

        double &operator[](unsigned int i)
        {
            return values[i];
        }

        double *values;
    };

    PlanarManipulatorStateSpace(const PlanarManipulator *manip);

    virtual ~PlanarManipulatorStateSpace();

    virtual bool isMetricSpace() const;

    virtual bool hasSymmetricDistance() const;

    virtual bool hasSymmetricInterpolate() const;

    virtual unsigned int getDimension() const;

    virtual double getMaximumExtent() const;

    virtual double getMeasure() const;

    virtual void enforceBounds(ompl::base::State *state) const;

    virtual bool satisfiesBounds(const ompl::base::State *state) const;

    virtual void copyState(ompl::base::State *dest, const ompl::base::State *src) const;

    virtual double distance(const ompl::base::State *s1, const ompl::base::State *s2) const;

    virtual bool equalStates(const ompl::base::State *s1, const ompl::base::State *s2) const;

    virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t,
                             ompl::base::State *state) const;

    virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const;

    virtual ompl::base::State *allocState() const;

    virtual void freeState(ompl::base::State *state) const;

    virtual void registerProjections();

    virtual double *getValueAddressAtIndex(ompl::base::State *state, const unsigned int index) const;

    virtual unsigned int getSerializationLength() const;

    virtual void serialize(void *serialization, const ompl::base::State *state) const;

    virtual void deserialize(ompl::base::State *state, const void *serialization) const;

    virtual void printState(const ompl::base::State *state, std::ostream &out) const;

    virtual void printSettings(std::ostream &out) const;

    bool usingCartesianInterpolator() const;
    void useCartesianInterpolator(bool useIt);

protected:
    void linearInterpolate(const double &from, const double &to, const double &t, double &out) const;

    const PlanarManipulator *manip_;
    unsigned int dimension_;
    bool cartInterpolator_;
};

#endif
