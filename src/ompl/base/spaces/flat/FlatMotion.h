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

#ifndef OMPL_BASE_SPACES_FLAT_MOTION_
#define OMPL_BASE_SPACES_FLAT_MOTION_

#include "ompl/util/ClassForward.h"

#include <Eigen/Core>

#include <boost/container/small_vector.hpp>

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

        /** \brief The flat output at time \e t, without clamping \e t to the duration. */
        Eigen::VectorXd evaluate(double t) const;

        /** \brief Write derivative level \e derivativeLevel of the flat output at time \e t into \e out,
            which needs one entry per output dimension.

            Levels above the degree come out zero.
            This allocates nothing, so a caller walking a motion sample by sample can hand back the same
            buffer every time rather than paying for a vector per sample.
        */
        void evaluate(double t, unsigned int derivativeLevel, Eigen::Ref<Eigen::VectorXd> out) const;

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

            Minimizing this cost value produces minimum-effort motions.
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

        Over a fixed duration, a steering of order \e k joins two flat states with the curve that fixes
        derivative levels 0 through \e k - 1 at both ends and spends the least control effort, meaning the
        integral of the squared norm of derivative level \e k.
        That curve is a polynomial of degree 2 \e k - 1 in every output dimension.
    */
    class FlatSteering
    {
    public:
        /** \brief Construct a steering generator for states of order \e order, which has to be at least 1. */
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

        /** \brief The \ref cost of the motion that \ref steer makes from \e from to \e to, or nothing
            when no motion joins them.

            By default this builds the motion with \ref steer and returns its cost.
            A subclass that knows the cost in closed form should override this to skip building the motion.
        */
        virtual std::optional<double> steeringCost(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                   const Eigen::Ref<const Eigen::MatrixXd> &to) const;

    protected:
        /** \brief Throw unless both flat states carry one row per derivative level of this steering and
            share a column count of at least one. */
        void checkFlatStates(const Eigen::Ref<const Eigen::MatrixXd> &from,
                             const Eigen::Ref<const Eigen::MatrixXd> &to) const;

        /** \brief Calculate the coefficients of the least-effort polynomial from \e from to \e to over \e duration, one
            column per axis.

            Below is a summary of the calculation procedure for these coefficients:

            At order \e k, the Taylor expansion at \e from fixes the lowest \e k coefficients.
            Knowing the bottom \e k coefficients, we can calculate residuals to find higher coefficients.
            Evaluating the residuals, we see a linear system with one equation per derivative level at \e to and one
            unknown per top coefficient.
            Scaling top coefficient \e j by \f$T^{k + j}\f$ and residual \e l by \f$T^l\f$,
            where \e T is the duration, writes the system in the time \f$\sigma = t / T\f$ scaled to run from 0 to 1.
            The entry in row \e l and column \e j of its matrix is then derivative level \e l of
            \f$\sigma^{k + j}\f$ at \f$\sigma = 1\f$.
            That matrix is the same for every duration, and its inverse sits in \ref endDerivativesInverse_.
        */
        Eigen::MatrixXd coefficientsOver(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                         const Eigen::Ref<const Eigen::MatrixXd> &to, double duration) const;

        /** \brief The number of derivative levels a flat state carries. */
        unsigned int order_;

        /** \brief The time penalty charged per unit duration. */
        double rho_{5.};

        /** \brief The inverse of the matrix of the linear system \ref coefficientsOver solves for the top
            \e k coefficients of a steer.

            In the scaled time \f$\sigma\f$ that \ref coefficientsOver describes, the entry in row \e l and
            column \e j of that matrix is derivative level \e l of \f$\sigma^{k + j}\f$ at \f$\sigma = 1\f$.
            Multiplying the scaled residuals at the far end by this inverse gives the scaled top
            coefficients.
            Each entry comes from a closed form and is correctly rounded.
        */
        Eigen::MatrixXd endDerivativesInverse_;

        /** \brief The quadratic form giving the control effort of a steer, scaled to a duration of 1, in
            terms of the shortfall its top \e k coefficients make up at the far end.

            This is the inverse of the controllability Gramian of a chain of \e k integrators over a duration
            of 1, and its entries are integers.
        */
        Eigen::MatrixXd effortForm_;
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
            The order has to be at least 1 and the duration has to be positive. */
        FixedDurationSteering(unsigned int order, double duration);

        /** \brief Set the duration every motion takes, which has to be positive. */
        void setDuration(double duration);

        /** \brief The duration every motion takes. */
        double getDuration() const
        {
            return duration_;
        }

        std::optional<FlatMotion> steer(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                        const Eigen::Ref<const Eigen::MatrixXd> &to) const override;

        std::optional<double> steeringCost(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                           const Eigen::Ref<const Eigen::MatrixXd> &to) const override;

    private:
        /** \brief The duration every motion takes. */
        double duration_;
    };

    /// @cond IGNORE
    OMPL_CLASS_FORWARD(MinimumEffortSteering);
    /// @endcond

    /** \brief Steering over the duration that minimizes control effort plus the time penalty.

        The duration is the positive duration at which control effort plus the time penalty is lowest.
        Effort alone shrinks as the duration grows without bound, so the time penalty keeps the duration finite.
        Flat states that already sit still at the same flat output have no such duration and steering
        between them reports failure.
    */
    class MinimumEffortSteering : public FlatSteering
    {
    public:
        /** \brief Steer between flat states holding \e order derivative levels, counting the flat output
            itself.
            The order has to be at least 1. */
        explicit MinimumEffortSteering(unsigned int order);

        /** \brief The duration at which steering between two flat states costs least, and that cost. */
        struct OptimalDuration
        {
            /** \brief The duration of the motion. */
            double duration;

            /** \brief The cost of steering over \e duration. */
            double cost;
        };

        /** \brief The positive duration at which \ref cost is lowest over motions from \e from to \e to,
            together with that cost, or nothing when there's no such duration.

            \ref steeringCost reports the cost alone and \ref steer builds the motion over the duration.
        */
        std::optional<OptimalDuration> optimalDuration(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                       const Eigen::Ref<const Eigen::MatrixXd> &to) const;

        /** \brief The least-effort motion from \e from to \e to over \e duration, which has to be positive.

            Providing a duration found by \ref optimalDuration allows the motion generator to find a steering without
            recalculating a duration.
        */
        FlatMotion motion(const Eigen::Ref<const Eigen::MatrixXd> &from, const Eigen::Ref<const Eigen::MatrixXd> &to,
                          double duration) const;

        /** \brief A duration at which the cheapest number of extra periods for one wrapping coordinate changes. */
        struct WindingChange
        {
            /** \brief The duration from which the new number of extra periods is cheapest. */
            double time;

            /** \brief The number of whole periods added to the flat output of \e to from this duration on. */
            double winding;
        };

        /** \brief The durations at which the cheapest way to reach \e to along wrapping coordinate \e axis
            changes, in order of time, over motions from \e from to \e to lasting up to \e horizon.

            A coordinate wrapping with period \e period reaches the same place in the flat output space
            after any whole number of extra periods, such as an angle turning one more full turn.
            Longer motions can afford more extra periods, so the cheapest number changes as the duration
            grows.
            Each change holds from its time until the next change, and zero extra periods holds before the
            first one.
        */
        boost::container::small_vector<WindingChange, 4> windingChanges(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                                        const Eigen::Ref<const Eigen::MatrixXd> &to,
                                                                        unsigned int axis, double period,
                                                                        double horizon) const;

        std::optional<FlatMotion> steer(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                        const Eigen::Ref<const Eigen::MatrixXd> &to) const override;

        std::optional<double> steeringCost(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                           const Eigen::Ref<const Eigen::MatrixXd> &to) const override;
    };
}  // namespace ompl::base

#endif
