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

/* Author: Clayton W. Ramsey */

#include "ompl/base/spaces/FlatMotion.h"

#include "ompl/util/Exception.h"

#include <Eigen/Eigenvalues>

#include <boost/math/constants/constants.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

/** Helper functions for manipulating polynomials. */
namespace
{
    /** \brief Evaluate the polynomial whose ascending-power coefficients are \e c at \e t. */
    double evaluatePolynomial(const Eigen::VectorXd &c, double t)
    {
        double value = 0.;
        for (Eigen::Index i = c.size() - 1; i >= 0; --i)
            value = value * t + c[i];
        return value;
    }

    /** \brief The ascending-power coefficients of the derivative of the polynomial whose ascending-power
        coefficients are \e c. */
    Eigen::VectorXd differentiatePolynomial(const Eigen::VectorXd &c)
    {
        Eigen::VectorXd slope = Eigen::VectorXd::Zero(std::max<Eigen::Index>(c.size() - 1, 1));
        for (Eigen::Index i = 1; i < c.size(); ++i)
            slope[i - 1] = static_cast<double>(i) * c[i];
        return slope;
    }

    /** \brief Refine \e root with Newton's method, keeping it only while the residual shrinks.

        \e slope has to be the derivative of the polynomial whose ascending-power coefficients are \e c.
    */
    double newton(const Eigen::VectorXd &c, const Eigen::VectorXd &slope, double root)
    {
        // Companion matrix eigenvalues carry a small backward error against the matrix rather than
        // against the polynomial, so without this the roots above degree four come back wrong often
        // enough to matter.
        // The step budget stops at three because a shrinking residual stops shrinking by then for all
        // but a few candidates in a thousand.
        const int budget = 3;

        double best = root;
        double bestResidual = std::abs(evaluatePolynomial(c, root));
        for (int step = 0; step < budget; ++step)
        {
            const double denominator = evaluatePolynomial(slope, best);
            if (denominator == 0.)
                break;
            const double candidate = best - evaluatePolynomial(c, best) / denominator;
            const double residual = std::abs(evaluatePolynomial(c, candidate));
            if (!std::isfinite(candidate) || residual >= bestResidual)
                break;
            best = candidate;
            bestResidual = residual;
        }
        return best;
    }

    /** \brief The roots of \f$b t^2 + c t + d\f$, where \e b is nonzero. */
    std::vector<double> quadraticRoots(double b, double c, double d)
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
    std::vector<double> cubicRoots(double a, double b, double c, double d)
    {
        const double a2 = b / a;
        const double a1 = c / a;
        const double a0 = d / a;

        const double q = (3. * a1 - a2 * a2) / 9.;
        const double r = (9. * a1 * a2 - 27. * a0 - 2. * a2 * a2 * a2) / 54.;
        const double discriminant = q * q * q + r * r;
        const double shift = a2 / 3.;

        // A cubic with a repeated root sits at a discriminant of zero, which rounding turns into a small
        // number of either sign, so the repeated case gets a band rather than an exact comparison.
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

    /** \brief The roots of \f$a t^4 + b t^3 + c t^2 + d t + e\f$, where \e a is nonzero, or nothing when
        no resolvent root leaves a factorization to read them off.

        Ferrari's method, which splits the quartic into two quadratics once a root of its resolvent cubic
        is in hand.
        Whichever resolvent root splits the quartic widest gets used, because a narrow split loses the outer
        pair of roots to cancellation and a resolvent cubic with a repeated root offers a split that is no
        split at all.
    */
    std::optional<std::vector<double>> quarticRoots(double a, double b, double c, double d, double e)
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
            return std::nullopt;

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

        std::vector<double> roots;
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

    /** \brief Whether the polynomial whose ascending-power coefficients are \e c vanishes at \e root to
        the precision its coefficients support. */
    bool vanishesAt(const Eigen::VectorXd &c, double root)
    {
        double value = 0.;
        double scale = 0.;
        for (Eigen::Index i = c.size() - 1; i >= 0; --i)
        {
            value = value * root + c[i];
            scale = scale * std::abs(root) + std::abs(c[i]);
        }
        return std::abs(value) <= 1e-9 * scale;
    }

    /** \brief The roots of the polynomial whose ascending-power coefficients are \e c and whose leading
        coefficient sits at index \e degree, read off the eigenvalues of its companion matrix.
        Roots with an imaginary part are dropped. */
    std::vector<double> companionRoots(const Eigen::VectorXd &c, Eigen::Index degree)
    {
        Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);
        companion.col(degree - 1) = -c.head(degree) / c[degree];
        if (degree > 1)
            companion.block(1, 0, degree - 1, degree - 1).setIdentity();

        const Eigen::EigenSolver<Eigen::MatrixXd> solver(companion, false);
        const auto &values = solver.eigenvalues();

        std::vector<double> roots;
        for (Eigen::Index i = 0; i < values.size(); ++i)
        {
            const std::complex<double> &value = values[i];
            if (std::abs(value.imag()) <= 1e-8 * (1. + std::abs(value.real())))
                roots.push_back(value.real());
        }
        return roots;
    }

    /** \brief The roots of the polynomial whose ascending-power coefficients are \e c.

        Degrees up to four go through closed forms and anything higher through the eigenvalues of the
        companion matrix.
        Every candidate gets refined and then checked against the polynomial, and the companion matrix
        takes over whenever a closed form hands back something the polynomial doesn't vanish at, which is
        how an ill-conditioned resolvent stays out of the answer.
        A closed form that comes back empty hands over too, because a quartic whose resolvent declines to
        split it looks from the outside like one with no roots at all.
    */
    std::vector<double> polynomialRoots(const Eigen::VectorXd &c)
    {
        std::vector<double> roots;
        if (c.size() < 2)
            return roots;

        const double scale = c.cwiseAbs().maxCoeff();
        if (scale == 0.)
            return roots;

        Eigen::Index degree = c.size() - 1;
        while (degree > 0 && std::abs(c[degree]) <= 1e-12 * scale)
            --degree;
        if (degree < 1)
            return roots;

        std::optional<std::vector<double>> closed;
        switch (degree)
        {
            case 1:
                closed = std::vector<double>{-c[0] / c[1]};
                break;
            case 2:
                closed = quadraticRoots(c[2], c[1], c[0]);
                break;
            case 3:
                closed = cubicRoots(c[3], c[2], c[1], c[0]);
                break;
            case 4:
                closed = quarticRoots(c[4], c[3], c[2], c[1], c[0]);
                break;
            default:
                break;
        }

        const Eigen::VectorXd slope = differentiatePolynomial(c);
        if (closed.has_value() && !closed->empty())
        {
            for (double &root : *closed)
                root = newton(c, slope, root);
            if (std::all_of(closed->begin(), closed->end(), [&c](double root) { return vanishesAt(c, root); }))
                return std::move(*closed);
        }

        roots = companionRoots(c, degree);
        for (double &root : roots)
            root = newton(c, slope, root);
        return roots;
    }

    /** \brief Every strictly positive point at which the polynomial with ascending-power coefficients
        \e c crosses from negative to positive.

        The polynomial here is the derivative of a cost, so an upward crossing is a local minimum of that
        cost and a root the polynomial only touches is not.
        Demanding the crossing also throws out the spurious roots that a repeated root at zero produces.
    */
    std::vector<double> upwardCrossings(const Eigen::VectorXd &c)
    {
        const Eigen::VectorXd slope = differentiatePolynomial(c);
        std::vector<double> crossings;
        for (double root : polynomialRoots(c))
            if (root > 0. && evaluatePolynomial(slope, root) > 0.)
                crossings.push_back(root);
        return crossings;
    }
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
        FlatMotion level = *this;
        for (unsigned int i = 0; i < derivativeLevel; ++i)
            level = level.derivative();

        const Eigen::MatrixXd gram = level.coefficients_ * level.coefficients_.transpose();
        double effort = 0.;
        for (Eigen::Index i = 0; i < gram.rows(); ++i)
            for (Eigen::Index j = 0; j < gram.cols(); ++j)
            {
                const double power = static_cast<double>(i + j + 1);
                effort += gram(i, j) * std::pow(duration_, power) / power;
            }
        return effort + rho * duration_;
    }

    double FlatMotion::peakSpeed() const
    {
        const Eigen::MatrixXd velocity = derivative().coefficients_;
        const Eigen::MatrixXd gram = velocity * velocity.transpose();

        Eigen::VectorXd squared = Eigen::VectorXd::Zero(2 * gram.rows() - 1);
        for (Eigen::Index i = 0; i < gram.rows(); ++i)
            for (Eigen::Index j = 0; j < gram.cols(); ++j)
                squared[i + j] += gram(i, j);

        double peak = std::max(evaluatePolynomial(squared, 0.), evaluatePolynomial(squared, duration_));
        if (squared.size() > 1)
        {
            Eigen::VectorXd slope(squared.size() - 1);
            for (Eigen::Index i = 1; i < squared.size(); ++i)
                slope[i - 1] = static_cast<double>(i) * squared[i];

            for (double t : polynomialRoots(slope))
                if (t > 0. && t < duration_)
                    peak = std::max(peak, evaluatePolynomial(squared, t));
        }
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

    MinimumEffortSteering::MinimumEffortSteering(unsigned int order) : FlatSteering(order)
    {
    }

    std::optional<double> MinimumEffortSteering::optimalDuration(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                                 const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        checkFlatStates(from, to);

        const auto v0 = from.row(1);
        const auto vf = to.row(1);
        const Eigen::RowVectorXd offset = to.row(0) - from.row(0);

        // The cost of the optimal fixed-duration steer is
        //     J(T) = 4 (|v0|^2 + v0 . vf + |vf|^2) / T - 12 offset . (v0 + vf) / T^2 + 12 |offset|^2 / T^3
        //            + rho T,
        // and clearing T^4 out of dJ/dT leaves this quartic.
        const double speedTerm = 4. * (v0.squaredNorm() + vf.squaredNorm() + v0.dot(vf));
        const double crossTerm = 12. * (v0 + vf).dot(offset);
        const double offsetTerm = 12. * offset.squaredNorm();

        Eigen::VectorXd quartic(5);
        quartic[0] = -3. * offsetTerm;
        quartic[1] = 2. * crossTerm;
        quartic[2] = -speedTerm;
        quartic[3] = 0.;
        quartic[4] = rho_;

        // The quartic can cross upward twice, which puts two local minima on J, so each crossing gets
        // priced and the cheapest one wins.
        // Taking the first crossing instead would sometimes charge several times what the motion needs to
        // cost, and an optimizing planner would steer through it believing the price.
        std::optional<double> best;
        double bestCost = std::numeric_limits<double>::infinity();
        for (double duration : upwardCrossings(quartic))
        {
            const double cost = offsetTerm / (duration * duration * duration) - crossTerm / (duration * duration) +
                                speedTerm / duration + rho_ * duration;
            if (cost < bestCost)
            {
                bestCost = cost;
                best = duration;
            }
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
