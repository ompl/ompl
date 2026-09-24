/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
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

/* Author: Clayton W. Ramsey */

#ifndef OMPL_BASE_SPACES_FLAT_TRAJECTORY_
#define OMPL_BASE_SPACES_FLAT_TRAJECTORY_

#include "ompl/base/spaces/FlatStateSpace.h"
#include "ompl/geometric/PathGeometric.h"

#include <vector>

namespace ompl::base
{
    /// @cond IGNORE
    OMPL_CLASS_FORWARD(FlatTrajectory);
    /// @endcond

    /** \brief A sequence of polynomial curves joined end to end, parameterized by time.

        The PathGeometric objects returned by geometric planners don't include the polynomial interpolations between
       states, so this class performs the necessary interpolation to produce dynamically valid paths.

        Time runs from zero through \ref duration across the whole run rather than restarting per segment,
        and \ref evaluate takes a time on that clock.
    */
    class FlatTrajectory
    {
    public:
        /** \brief An empty trajectory of no duration. */
        FlatTrajectory() = default;

        /** \brief The trajectory following \e motions in the order given.

            Every motion needs the same output dimension.
        */
        explicit FlatTrajectory(std::vector<FlatMotion> motions);

        /** \brief The trajectory steering through \e states in order, under \e space.

            Consecutive states that coincide have no polynomial between them and contribute no motion, so
            the trajectory can come out shorter than the list it was built from.
        */
        FlatTrajectory(const FlatStateSpace *space, const std::vector<const State *> &states);

        /** \brief The trajectory steering through the states of \e path.

            \e path has to run through an ompl::base::FlatStateSpace.
        */
        explicit FlatTrajectory(const geometric::PathGeometric &path);

        /** \brief The number of motions strung together. */
        std::size_t size() const
        {
            return motions_.size();
        }

        /** \brief The motions strung together. */
        const std::vector<FlatMotion> &motions() const
        {
            return motions_;
        }

        /** \brief Motion \e index, counting from zero. */
        const FlatMotion &motion(std::size_t index) const;

        /** \brief Calculate the time that the motion at \e index starts. */
        double startTime(std::size_t index) const;

        /** \brief The durations of every motion added up. */
        double duration() const;

        /** \brief The number of flat output dimensions the run moves through, which is zero for an empty
            trajectory. */
        unsigned int outputDimension() const;

        /** \brief Write derivative level \e level of the flat output at time \e t into \e out, which needs
            one entry per output dimension.

            Times outside zero through \ref duration clamp to the nearer end.
            This allocates nothing.
        */
        void evaluate(double t, unsigned int level, Eigen::Ref<Eigen::VectorXd> out) const;

        /** \brief The flat output at time \e t. */
        Eigen::VectorXd evaluate(double t) const;

        /** \brief Write the flat state at time \e t into \e state, which \e space has to have allocated. */
        void toState(const FlatStateSpace *space, double t, State *state) const;

    private:
        /** \brief The motions strung together. */
        std::vector<FlatMotion> motions_;

        /** \brief The time each motion starts at, with the duration of the whole run on the end, so this
            always holds one more entry than \ref motions_. */
        std::vector<double> starts_{0.};
    };
}  // namespace ompl::base

#endif
