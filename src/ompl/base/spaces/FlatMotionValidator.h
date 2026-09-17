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

/* Author: Clayton Ramsey */

#ifndef OMPL_BASE_SPACES_FLAT_MOTION_VALIDATOR_
#define OMPL_BASE_SPACES_FLAT_MOTION_VALIDATOR_

#include "ompl/base/MotionValidator.h"
#include "ompl/base/spaces/FlatStateSpace.h"

namespace ompl::base
{
    /// @cond IGNORE
    OMPL_CLASS_FORWARD(FlatMotionValidator);
    /// @endcond

    /** \brief Checks motions in an ompl::base::FlatStateSpace, solving the steering once per edge.

        Sampling an edge through ompl::base::FlatStateSpace::interpolate solves the steering afresh for
        every sample, so an edge split into a hundred pieces pays for a hundred solves.
        This validator solves the edge once, holds the motion for as long as it takes to walk it, and
        hands that motion back on every sample.

        ompl::base::SpaceInformation installs this validator for a flat state space by default, so a
        caller gets it without asking.
        Verdicts match ompl::base::DiscreteMotionValidator exactly, because both sample the same edge at
        the same fractions of its duration.
    */
    class FlatMotionValidator : public MotionValidator
    {
    public:
        /** \brief Validate motions in the flat state space \e si plans in, which \e si has to carry. */
        FlatMotionValidator(SpaceInformation *si) : MotionValidator(si)
        {
            defaultSettings();
        }

        /** \brief Validate motions in the flat state space \e si plans in, which \e si has to carry. */
        FlatMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
        {
            defaultSettings();
        }

        ~FlatMotionValidator() override = default;

        /** \brief Whether every sample along the motion from \e s1 to \e s2 is valid, assuming \e s1
            already is.

            Samples land coarse to fine, halving the spacing on each pass, so a blocked edge comes back
            rejected after a handful of checks rather than after a walk from one end.
        */
        bool checkMotion(const State *s1, const State *s2) const override;

        /** \brief Whether every sample along the motion from \e s1 to \e s2 is valid, assuming \e s1
            already is, recording where the motion stopped being valid.

            On a false result \e lastValid.second comes out holding the fraction of the duration the
            motion stayed valid through, and \e lastValid.first, when it isn't null, comes out holding the
            state at that fraction.
            Samples land in order from \e s1 onward, so the fraction is the one just before the first
            invalid sample.
        */
        bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

    protected:
        /** \brief Take the flat state space out of the space information, throwing when it holds another
            kind of space. */
        void defaultSettings();

        /** \brief The flat state space the motions run through. */
        FlatStateSpace *stateSpace_{nullptr};
    };
}  // namespace ompl::base

#endif
