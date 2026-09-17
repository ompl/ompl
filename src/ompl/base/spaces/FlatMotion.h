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

#ifndef OMPL_BASE_SPACES_FLAT_MOTION_
#define OMPL_BASE_SPACES_FLAT_MOTION_

#include "ompl/util/ClassForward.h"

#include <Eigen/Core>

#include <optional>

namespace ompl::base
{
    /** \brief The polynomial curve joining one pair of flat states of a differentially flat system.

        A motion spans a single edge, the way a straight line spans one in a Euclidean state space, and a
        solved path strings many of them end to end.

        The curve holds one polynomial in time per flat output dimension.
        Accordingly, the curve is represented as a matrix of coefficients.
        Row \e i of the coefficient matrix carries the coefficient of \f$t^i\f$ for every dimension and
        column \e j carries the polynomial for dimension \e j.
        The curve is meaningful for times from zero through \ref duration.

        A flat state is the flat output stacked with its derivatives, written as a matrix whose row \e i
        holds derivative level \e i, so a motion reproduces the flat state at time \e t by evaluating
        itself and its first few derivatives there.
    */
    class FlatMotion
    {
    public:
        /** \brief Construct an empty motion carrying no coefficients and zero duration. */
        FlatMotion() = default;

        /** \brief Construct a motion from an ascending-power coefficient matrix and a duration.

            \param coefficients row \e i holds the coefficient of \f$t^i\f$ for every output dimension, so
            the matrix has degree plus one rows and one column per dimension
            \param duration the length of the time interval the curve covers, which has to be positive
        */
        FlatMotion(Eigen::MatrixXd coefficients, double duration);

        /** \brief The ascending-power coefficient matrix. */
        const Eigen::MatrixXd &coefficients() const
        {
            return coefficients_;
        }

        /** \brief The length of the time interval the curve covers. */
        double duration() const
        {
            return duration_;
        }

        /** \brief The highest power of time carrying a coefficient, which is zero for an empty
            motion. */
        unsigned int degree() const
        {
            return coefficients_.rows() < 2 ? 0u : static_cast<unsigned int>(coefficients_.rows()) - 1u;
        }

        /** \brief The number of flat output dimensions the curve moves through. */
        unsigned int outputDimension() const
        {
            return static_cast<unsigned int>(coefficients_.cols());
        }

        /** \brief The flat output at time \e t, evaluated by Horner's method without clamping \e t to the
            duration. */
        Eigen::VectorXd evaluate(double t) const;

        /** \brief The derivative with respect to time, over the same duration.

            The derivative of a constant curve is the zero curve of the same dimension rather than an empty
            one, so repeated differentiation always yields a usable motion.
        */
        FlatMotion derivative() const;

        /** \brief The antiderivative with respect to time whose value at time zero is zero, over the same
            duration. */
        FlatMotion integral() const;

        /** \brief The cost of this motion, computed from control effort and duration.

            The computed cost is the sum of the curve's control effort and \e rho times the duration.
            The control effort is the integral over the duration of the squared Euclidean norm of the flat
            output differentiated \e derivativeLevel times.

            Minimizing this cost value produces minimum-effort steers.
            Increasing \e rho buys a shorter motion with more effort.
        */
        double cost(unsigned int derivativeLevel, double rho) const;

        /** \brief The fastest the flat output moves anywhere along the motion.

            This is the largest Euclidean norm the first derivative reaches between time zero and the
            duration.

            Dividing a distance in the flat output space by this speed gives the longest time step that
            keeps consecutive samples within that distance of each other.
        */
        double peakSpeed() const;

    private:
        /** \brief Ascending-power coefficients, one row per power and one column per output dimension. */
        Eigen::MatrixXd coefficients_;

        /** \brief The length of the time interval the curve covers. */
        double duration_{0.};
    };

    /// @cond IGNORE
    OMPL_CLASS_FORWARD(FlatSteering);
    /// @endcond

    /** \brief A steering structure that constructs motions between two flat states.

        Steering is directed, so the motion from \e from to \e to is not the reverse of the motion from
        \e to to \e from.
    */
    class FlatSteering
    {
    public:
        /** \brief Construct a steering generator for states of order \e order.
            The order has to be 2. */
        explicit FlatSteering(unsigned int order);

        virtual ~FlatSteering() = default;

        /** \brief The number of derivative levels a flat state carries, counting the flat output itself. */
        unsigned int getOrder() const
        {
            return order_;
        }

        /** \brief Set the time penalty that \ref cost charges per unit duration, which has to be positive.
            It defaults to 5. */
        void setRho(double rho);

        /** \brief The time penalty that \ref cost charges per unit duration. */
        double getRho() const
        {
            return rho_;
        }

        /** \brief Calculates the cost of traversing \e motion. */
        double cost(const FlatMotion &motion) const;

        /** \brief Calculates the motion from \e from to \e to, or nothing when no motion joins them. */
        virtual std::optional<FlatMotion> steer(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                const Eigen::Ref<const Eigen::MatrixXd> &to) const = 0;

    protected:
        /** \brief Throw unless both flat states carry one row per derivative level of this steering and
            share a column count of at least one. */
        void checkFlatStates(const Eigen::Ref<const Eigen::MatrixXd> &from,
                             const Eigen::Ref<const Eigen::MatrixXd> &to) const;

        /** \brief The number of derivative levels a flat state carries. */
        unsigned int order_;

        /** \brief The time penalty charged per unit duration. */
        double rho_{5.};
    };

    /// @cond IGNORE
    OMPL_CLASS_FORWARD(FixedDurationSteering);
    /// @endcond

    /** \brief Steering over a duration the caller picks.

        This steering generates motions that minimize control effort among all curves of a fixed duration.
    */
    class FixedDurationSteering : public FlatSteering
    {
    public:
        /** \brief Steer between flat states holding \e order derivative levels over \e duration.
            The order has to be 2 and the duration has to be positive. */
        FixedDurationSteering(unsigned int order, double duration);

        /** \brief Set the duration every steer takes, which has to be positive. */
        void setDuration(double duration);

        /** \brief The duration every steer takes. */
        double getDuration() const
        {
            return duration_;
        }

        std::optional<FlatMotion> steer(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                        const Eigen::Ref<const Eigen::MatrixXd> &to) const override;

    private:
        /** \brief The duration every steer takes. */
        double duration_;
    };

    /// @cond IGNORE
    OMPL_CLASS_FORWARD(MinimumEffortSteering);
    /// @endcond

    /** \brief Steering over the duration that minimizes control effort plus the time penalty.

        The duration is the smallest positive one at which the cost bottoms out, read off the roots of the
        derivative of the cost with respect to duration.
        Effort alone shrinks as the duration grows without bound, so the time penalty keeps the duration finite.
        Flat states that already coincide have no such duration and steering between them reports failure.
    */
    class MinimumEffortSteering : public FlatSteering
    {
    public:
        /** \brief Steer between flat states holding \e order derivative levels, counting the flat output
            itself.
            The order has to be 2. */
        explicit MinimumEffortSteering(unsigned int order);

        /** \brief The smallest positive duration at which \ref cost bottoms out over motions from
            \e from to \e to, or nothing when there's none. */
        std::optional<double> optimalDuration(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                              const Eigen::Ref<const Eigen::MatrixXd> &to) const;

        std::optional<FlatMotion> steer(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                        const Eigen::Ref<const Eigen::MatrixXd> &to) const override;
    };
}  // namespace ompl::base

#endif
