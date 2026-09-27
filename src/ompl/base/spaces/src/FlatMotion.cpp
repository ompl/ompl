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

#include "ompl/base/spaces/FlatMotion.h"

#include "ompl/util/Exception.h"

#include <Eigen/Eigenvalues>

#include <boost/container/small_vector.hpp>
#include <boost/math/constants/constants.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace
{
    /** \brief Ascending-power polynomial coefficients, held without touching the heap up to degree four. */
    using Coefficients = boost::container::small_vector<double, 5>;

    /** \brief Real roots of a polynomial, held without touching the heap up to the four. */
    using Roots = boost::container::small_vector<double, 4>;

    /** \brief A polynomial in one variable, carrying one coefficient per power of that variable. */
    class Polynomial
    {
    public:
        /** \brief The value of a polynomial and its derivative at one point. */
        struct Evaluation
        {
            /** \brief The value of the polynomial. */
            double value;

            /** \brief The value of the derivative. */
            double slope;
        };

        /** \brief Construct the polynomial whose ascending-power coefficients are \e coefficients, so
            entry \e i is the coefficient of \f$t^i\f$. */
        explicit Polynomial(Coefficients coefficients) : coefficients_(std::move(coefficients))
        {
        }

        /** \brief Construct the polynomial whose ascending-power coefficients are \e coefficients. */
        Polynomial(std::initializer_list<double> coefficients) : coefficients_(coefficients)
        {
        }

        /** \brief The value at \e t, evaluated by Horner's method. */
        double operator()(double t) const
        {
            double value = 0.;
            for (Eigen::Index i = size() - 1; i >= 0; --i)
                value = value * t + coefficients_[i];
            return value;
        }

        /** \brief The value and derivative at \e t, evaluated together by Horner's method so the derivative
            never needs a polynomial of its own. */
        Evaluation evaluate(double t) const
        {
            Evaluation result{0., 0.};
            for (Eigen::Index i = size() - 1; i >= 0; --i)
            {
                result.slope = result.slope * t + result.value;
                result.value = result.value * t + coefficients_[i];
            }
            return result;
        }

        /** \brief The derivative with respect to the variable.

            The derivative of a constant is the zero polynomial carrying one coefficient rather than an
            empty one, so repeated differentiation always yields a usable polynomial.
        */
        Polynomial derivative() const
        {
            Coefficients slope(std::max<Eigen::Index>(size() - 1, 1), 0.);
            for (Eigen::Index i = 1; i < size(); ++i)
                slope[i - 1] = static_cast<double>(i) * coefficients_[i];
            return Polynomial(std::move(slope));
        }

        /** \brief The real roots.

            Degrees up to four go through closed forms and anything higher through the eigenvalues of the
            companion matrix.
            Every candidate gets refined and then checked against the polynomial, and the companion matrix
            takes over whenever a closed form hands back something the polynomial doesn't vanish at, which
            is how an ill-conditioned resolvent stays out of the answer.
            A closed form that comes back empty hands over too, because a quartic whose resolvent declines
            to split it looks from the outside like one with no roots at all.
        */
        Roots roots() const
        {
            const Eigen::Index degree = leadingDegree();
            if (degree < 1)
                return {};

            Roots closed;
            switch (degree)
            {
                case 1:
                    closed = {-coefficients_[0] / coefficients_[1]};
                    break;
                case 2:
                    closed = quadraticRoots(coefficients_[2], coefficients_[1], coefficients_[0]);
                    break;
                case 3:
                    closed = cubicRoots(coefficients_[3], coefficients_[2], coefficients_[1], coefficients_[0]);
                    break;
                case 4:
                    closed = quarticRoots(coefficients_[4], coefficients_[3], coefficients_[2], coefficients_[1],
                                          coefficients_[0]);
                    break;
                default:
                    break;
            }

            if (!closed.empty())
            {
                for (double &root : closed)
                    root = newton(root);
                if (std::all_of(closed.begin(), closed.end(), [this](double root) { return vanishesAt(root); }))
                    return closed;
            }

            Roots refined = companionRoots(degree);
            for (double &root : refined)
                root = newton(root);
            return refined;
        }

        /** \brief Every strictly positive point at which the polynomial crosses from negative to positive.

            The polynomial this runs on is the derivative of a cost, so an upward crossing is a local
            minimum of that cost and a root the polynomial only touches is not.
            Demanding the crossing also throws out the spurious roots that a repeated root at zero
            produces.
        */
        Roots upwardCrossings() const
        {
            Roots crossings;
            for (double root : roots())
                if (root > 0. && evaluate(root).slope > 0.)
                    crossings.push_back(root);
            return crossings;
        }

    private:
        /** \brief The number of coefficients. */
        Eigen::Index size() const
        {
            return static_cast<Eigen::Index>(coefficients_.size());
        }

        /** \brief The coefficients as a vector for Eigen to work on. */
        Eigen::Map<const Eigen::VectorXd> vector() const
        {
            return {coefficients_.data(), size()};
        }

        /** \brief The index of the highest-power coefficient large enough next to the rest to lead, or
            zero when nothing is left to solve.

            A coefficient far below the largest one is the residue of a cancellation rather than a leading
            term, and handing the root finder the degree it implies asks for roots that aren't there.
        */
        Eigen::Index leadingDegree() const
        {
            if (size() < 2)
                return 0;

            const double scale = vector().cwiseAbs().maxCoeff();
            if (scale == 0.)
                return 0;

            Eigen::Index degree = size() - 1;
            while (degree > 0 && std::abs(coefficients_[degree]) <= 1e-12 * scale)
                --degree;
            return degree;
        }

        /** \brief Whether the polynomial vanishes at \e root to the precision its coefficients support. */
        bool vanishesAt(double root) const
        {
            double value = 0.;
            double scale = 0.;
            for (Eigen::Index i = size() - 1; i >= 0; --i)
            {
                value = value * root + coefficients_[i];
                scale = scale * std::abs(root) + std::abs(coefficients_[i]);
            }
            return std::abs(value) <= 1e-9 * scale;
        }

        /** \brief Refine \e root with Newton's method, keeping it only while the residual shrinks. */
        double newton(double root) const
        {
            // Companion matrix eigenvalues carry a small backward error against the matrix rather than
            // against the polynomial, so without this the roots above degree four come back wrong often
            // enough to matter.
            // The step budget stops at three because a shrinking residual stops shrinking by then for all
            // but a few candidates in a thousand.
            const int budget = 3;

            double best = root;
            Evaluation atBest = evaluate(best);
            for (int step = 0; step < budget; ++step)
            {
                if (atBest.slope == 0.)
                    break;
                const double candidate = best - atBest.value / atBest.slope;
                const Evaluation atCandidate = evaluate(candidate);
                if (!std::isfinite(candidate) || std::abs(atCandidate.value) >= std::abs(atBest.value))
                    break;
                best = candidate;
                atBest = atCandidate;
            }
            return best;
        }

        /** \brief The roots read off the eigenvalues of the companion matrix of the polynomial truncated
            to \e degree, which has to be the index of a leading coefficient.
            Roots with an imaginary part are dropped. */
        Roots companionRoots(Eigen::Index degree) const
        {
            Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);
            companion.col(degree - 1) = -vector().head(degree) / coefficients_[degree];
            if (degree > 1)
                companion.block(1, 0, degree - 1, degree - 1).setIdentity();

            const Eigen::EigenSolver<Eigen::MatrixXd> solver(companion, false);
            const auto &values = solver.eigenvalues();

            Roots roots;
            for (Eigen::Index i = 0; i < values.size(); ++i)
            {
                const std::complex<double> &value = values[i];
                if (std::abs(value.imag()) <= 1e-8 * (1. + std::abs(value.real())))
                    roots.push_back(value.real());
            }
            return roots;
        }

        /** \brief The roots of \f$b t^2 + c t + d\f$, where \e b is nonzero. */
        static Roots quadraticRoots(double b, double c, double d)
        {
            const double discriminant = c * c - 4. * b * d;
            if (discriminant < 0.)
                return {};

            const double spread = std::sqrt(discriminant);
            return {(-c - spread) / (2. * b), (-c + spread) / (2. * b)};
        }

        /** \brief The roots of \f$a t^3 + b t^2 + c t + d\f$, where \e a is nonzero.

            Cardano's method, which always hands back at least one root because a cubic always has one.
        */
        static Roots cubicRoots(double a, double b, double c, double d)
        {
            const double a2 = b / a;
            const double a1 = c / a;
            const double a0 = d / a;

            const double q = (3. * a1 - a2 * a2) / 9.;
            const double r = (9. * a1 * a2 - 27. * a0 - 2. * a2 * a2 * a2) / 54.;
            const double discriminant = q * q * q + r * r;
            const double shift = a2 / 3.;

            // A cubic with a repeated root sits at a discriminant of zero, which rounding turns into a
            // small number of either sign, so the repeated case gets a band rather than an exact
            // comparison.
            // Landing in the band by mistake costs precision on the repeated root and rounding out of it
            // loses that root altogether.
            const double band = 1e-12 * std::max(std::abs(q * q * q), r * r);

            if (discriminant > band)
            {
                const double spread = std::sqrt(discriminant);
                return {std::cbrt(r + spread) + std::cbrt(r - spread) - shift};
            }
            if (discriminant >= -band)
            {
                const double s = std::cbrt(r);
                return {2. * s - shift, -s - shift};
            }

            const double theta = std::acos(r / std::sqrt(-q * q * q));
            const double radius = 2. * std::sqrt(-q);
            const double third = boost::math::constants::two_pi<double>() / 3.;
            return {radius * std::cos(theta / 3.) - shift, radius * std::cos(theta / 3. + third) - shift,
                    radius * std::cos(theta / 3. + 2. * third) - shift};
        }

        /** \brief The roots of \f$a t^4 + b t^3 + c t^2 + d t + e\f$, where \e a is nonzero, or none
            when no resolvent root leaves a factorization to read them off.

            Ferrari's method, which splits the quartic into two quadratics once a root of its resolvent
            cubic is in hand.
            Whichever resolvent root splits the quartic widest gets used, because a narrow split loses the
            outer pair of roots to cancellation and a resolvent cubic with a repeated root offers a split
            that is no split at all.
        */
        static Roots quarticRoots(double a, double b, double c, double d, double e)
        {
            const double a3 = b / a;
            const double a2 = c / a;
            const double a1 = d / a;
            const double a0 = e / a;

            double resolvent = 0.;
            double square = -1.;
            for (double candidate : cubicRoots(1., -a2, a1 * a3 - 4. * a0, 4. * a2 * a0 - a1 * a1 - a3 * a3 * a0))
            {
                const double split = a3 * a3 / 4. - a2 + candidate;
                if (split > square)
                {
                    resolvent = candidate;
                    square = split;
                }
            }
            if (square < 0.)
                return {};

            const double root = std::sqrt(square);
            double upper, lower;
            if (root != 0.)
            {
                const double base = 0.75 * a3 * a3 - square - 2. * a2;
                const double offset = 0.25 * (4. * a3 * a2 - 8. * a1 - a3 * a3 * a3) / root;
                upper = std::sqrt(base + offset);
                lower = std::sqrt(base - offset);
            }
            else
            {
                const double base = 0.75 * a3 * a3 - 2. * a2;
                const double offset = 2. * std::sqrt(resolvent * resolvent - 4. * a0);
                upper = std::sqrt(base + offset);
                lower = std::sqrt(base - offset);
            }

            Roots roots;
            const double shift = a3 / 4.;
            if (!std::isnan(upper))
            {
                roots.push_back(-shift + (root + upper) / 2.);
                roots.push_back(-shift + (root - upper) / 2.);
            }
            if (!std::isnan(lower))
            {
                roots.push_back(-shift - (root - lower) / 2.);
                roots.push_back(-shift - (root + lower) / 2.);
            }
            return roots;
        }

        /** \brief Ascending-power coefficients, one per power of the variable. */
        Coefficients coefficients_;
    };

    /** \brief The duration-independent terms in the cost of the optimal fixed-duration steer between two
        flat states of order 2.

        Over duration T that cost is
            J(T) = 4 (|v0|^2 + v0 . vf + |vf|^2) / T - 12 offset . (v0 + vf) / T^2 + 12 |offset|^2 / T^3
                   + rho T,
        where offset is the change in flat output.
    */
    struct EffortTerms
    {
        /** \brief Construct the terms for steering from \e from to \e to. */
        EffortTerms(const Eigen::Ref<const Eigen::MatrixXd> &from, const Eigen::Ref<const Eigen::MatrixXd> &to)
        {
            const auto v0 = from.row(1);
            const auto vf = to.row(1);
            const auto offset = to.row(0) - from.row(0);

            speedTerm = 4. * (v0.squaredNorm() + vf.squaredNorm() + v0.dot(vf));
            crossTerm = 12. * (v0 + vf).dot(offset);
            offsetTerm = 12. * offset.squaredNorm();
        }

        /** \brief The cost J(T) of steering over \e duration with time penalty \e rho. */
        double cost(double duration, double rho) const
        {
            const double T = duration;
            return offsetTerm / (T * T * T) - crossTerm / (T * T) + speedTerm / T + rho * T;
        }

        /** \brief The coefficient of 1 / T. */
        double speedTerm;

        /** \brief The negated coefficient of 1 / T^2. */
        double crossTerm;

        /** \brief The coefficient of 1 / T^3. */
        double offsetTerm;
    };
}  // namespace

namespace ompl::base
{
    FlatMotion::FlatMotion(Eigen::MatrixXd coefficients, double duration)
      : coefficients_(std::move(coefficients)), duration_(duration)
    {
        if (coefficients_.rows() < 1 || coefficients_.cols() < 1)
            throw Exception("FlatMotion needs at least one coefficient and one output dimension");
        if (!(duration_ > 0.))
            throw Exception("FlatMotion needs a positive duration");
    }

    Eigen::VectorXd FlatMotion::evaluate(double t) const
    {
        Eigen::VectorXd value(coefficients_.cols());
        evaluate(t, 0u, value);
        return value;
    }

    void FlatMotion::evaluate(double t, unsigned int derivativeLevel, Eigen::Ref<Eigen::VectorXd> out) const
    {
        out.setZero();

        const auto level = static_cast<Eigen::Index>(derivativeLevel);
        if (level >= coefficients_.rows())
            return;

        // Horner over the rows at or above the level.
        // Differentiating row i down to the level leaves the falling factorial i (i - 1) ... (i - level
        // + 1) in front of it, so weighting each row by that evaluates the derivative without building
        // its polynomial first.
        for (Eigen::Index i = coefficients_.rows() - 1; i >= level; --i)
        {
            double weight = 1.;
            for (Eigen::Index k = 0; k < level; ++k)
                weight *= static_cast<double>(i - k);
            out = out * t + weight * coefficients_.row(i).transpose();
        }
    }

    FlatMotion FlatMotion::derivative() const
    {
        FlatMotion result;
        result.duration_ = duration_;
        result.coefficients_ =
            Eigen::MatrixXd::Zero(std::max<Eigen::Index>(coefficients_.rows() - 1, 1), coefficients_.cols());
        for (Eigen::Index i = 1; i < coefficients_.rows(); ++i)
            result.coefficients_.row(i - 1) = static_cast<double>(i) * coefficients_.row(i);
        return result;
    }

    FlatMotion FlatMotion::integral() const
    {
        FlatMotion result;
        result.duration_ = duration_;
        result.coefficients_ = Eigen::MatrixXd::Zero(coefficients_.rows() + 1, coefficients_.cols());
        for (Eigen::Index i = 0; i < coefficients_.rows(); ++i)
            result.coefficients_.row(i + 1) = coefficients_.row(i) / static_cast<double>(i + 1);
        return result;
    }

    double FlatMotion::cost(unsigned int derivativeLevel, double rho) const
    {
        // Integrate the squared norm of the derivative at the level over the duration.
        // Differentiating row i down to the level scales it by the falling factorial i (i - 1) ... (i -
        // level + 1) and leaves it at power i - level, so the integral sums over pairs of rows at or above
        // the level without building the derivative.
        // Each off-diagonal pair counts twice.
        const auto level = static_cast<Eigen::Index>(derivativeLevel);
        double effort = 0.;
        for (Eigen::Index i = level; i < coefficients_.rows(); ++i)
        {
            double weightI = 1.;
            for (Eigen::Index k = 0; k < level; ++k)
                weightI *= static_cast<double>(i - k);

            double weightJ = weightI;
            double durationPower = std::pow(duration_, static_cast<double>(2 * (i - level) + 1));
            for (Eigen::Index j = i; j < coefficients_.rows(); ++j)
            {
                const double power = static_cast<double>(i + j - 2 * level + 1);
                const double term =
                    weightI * weightJ * coefficients_.row(i).dot(coefficients_.row(j)) * durationPower / power;

                // double-count off-diagonals
                effort += i == j ? term : 2. * term;

                weightJ *= static_cast<double>(j + 1) / static_cast<double>(j + 1 - level);
                durationPower *= duration_;
            }
        }
        return effort + rho * duration_;
    }

    double FlatMotion::peakSpeed() const
    {
        const Eigen::Index rows = coefficients_.rows();
        Coefficients coefficients(std::max<Eigen::Index>(2 * rows - 3, 1), 0.);
        for (Eigen::Index i = 1; i < rows; ++i)
        {
            const double weight = static_cast<double>(i);
            coefficients[2 * (i - 1)] += weight * weight * coefficients_.row(i).squaredNorm();
            for (Eigen::Index j = i + 1; j < rows; ++j)
                coefficients[i + j - 2] +=
                    2. * weight * static_cast<double>(j) * coefficients_.row(i).dot(coefficients_.row(j));
        }

        const Polynomial v_squared(std::move(coefficients));
        double peak = std::max(v_squared(0.), v_squared(duration_));
        for (double t : v_squared.derivative().roots())
            if (t > 0. && t < duration_)
                peak = std::max(peak, v_squared(t));
        return std::sqrt(std::max(peak, 0.));
    }

    FlatSteering::FlatSteering(unsigned int order) : order_(order)
    {
        if (order != 2u)
            throw Exception("FlatSteering only supports order 2");
    }

    void FlatSteering::setRho(double rho)
    {
        if (!(rho > 0.))
            throw Exception("FlatSteering needs a positive rho");
        rho_ = rho;
    }

    double FlatSteering::cost(const FlatMotion &motion) const
    {
        return motion.cost(order_, rho_);
    }

    std::optional<double> FlatSteering::steeringCost(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                     const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        const std::optional<FlatMotion> motion = steer(from, to);
        if (!motion.has_value())
            return {};
        return cost(*motion);
    }

    void FlatSteering::checkFlatStates(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                       const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        if (from.rows() != static_cast<Eigen::Index>(order_) || to.rows() != static_cast<Eigen::Index>(order_))
            throw Exception("FlatSteering needs flat states with one row per derivative level");
        if (from.cols() != to.cols() || from.cols() < 1)
            throw Exception("FlatSteering needs flat states agreeing on the output dimension");
    }

    FixedDurationSteering::FixedDurationSteering(unsigned int order, double duration) : FlatSteering(order)
    {
        setDuration(duration);
    }

    void FixedDurationSteering::setDuration(double duration)
    {
        if (!(duration > 0.))
            throw Exception("FixedDurationSteering needs a positive duration");
        duration_ = duration;
    }

    std::optional<FlatMotion> FixedDurationSteering::steer(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                           const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        checkFlatStates(from, to);

        const double T = duration_;
        const auto y0 = from.row(0);
        const auto v0 = from.row(1);
        const auto yf = to.row(0);
        const auto vf = to.row(1);

        const Eigen::RowVectorXd offset = yf - y0 - v0 * T;
        const Eigen::RowVectorXd change = vf - v0;

        Eigen::MatrixXd coefficients(4, from.cols());
        coefficients.row(0) = y0;
        coefficients.row(1) = v0;
        coefficients.row(2) = 3. * offset / (T * T) - change / T;
        coefficients.row(3) = -2. * offset / (T * T * T) + change / (T * T);
        return FlatMotion(std::move(coefficients), T);
    }

    std::optional<double> FixedDurationSteering::steeringCost(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                              const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        checkFlatStates(from, to);
        return EffortTerms(from, to).cost(duration_, rho_);
    }

    MinimumEffortSteering::MinimumEffortSteering(unsigned int order) : FlatSteering(order)
    {
    }

    std::optional<double> MinimumEffortSteering::optimalDuration(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                                 const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        const std::optional<PricedDuration> cheapest = cheapestDuration(from, to);
        if (!cheapest.has_value())
            return {};
        return cheapest->duration;
    }

    std::optional<double> MinimumEffortSteering::steeringCost(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                              const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        const std::optional<PricedDuration> cheapest = cheapestDuration(from, to);
        if (!cheapest.has_value())
            return {};
        return cheapest->cost;
    }

    std::optional<MinimumEffortSteering::PricedDuration> MinimumEffortSteering::cheapestDuration(
        const Eigen::Ref<const Eigen::MatrixXd> &from, const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        checkFlatStates(from, to);

        // Clearing T^4 out of dJ/dT leaves this quartic.
        const EffortTerms terms(from, to);
        const Polynomial quartic = {-3. * terms.offsetTerm, 2. * terms.crossTerm, -terms.speedTerm, 0., rho_};

        // The quartic can cross upward twice, which puts two local minima on J, so each crossing gets
        // priced and the cheapest one wins.
        // Taking the first crossing instead would sometimes charge several times what the motion needs to
        // cost, and an optimizing planner would steer through it believing the price.
        std::optional<PricedDuration> best;
        for (double t : quartic.upwardCrossings())
        {
            const double cost = terms.cost(t, rho_);
            if (!best.has_value() || cost < best->cost)
                best = PricedDuration{t, cost};
        }

        return best;
    }

    std::optional<FlatMotion> MinimumEffortSteering::steer(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                           const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        const std::optional<double> duration = optimalDuration(from, to);
        if (!duration.has_value())
            return {};
        return FixedDurationSteering(order_, *duration).steer(from, to);
    }
}  // namespace ompl::base
