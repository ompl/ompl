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

#include "ompl/base/spaces/flat/FlatMotion.h"

#include "ompl/util/Exception.h"

#include <Eigen/Eigenvalues>

#include <boost/config.hpp>
#include <boost/container/small_vector.hpp>
#include <boost/math/constants/constants.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <type_traits>

namespace
{
    /** \brief Ascending-power polynomial coefficients, held without touching the heap up to degree four. */
    using Coefficients = boost::container::small_vector<double, 5>;

    /** \brief Real roots of a polynomial, held without touching the heap up to degree four. */
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

        /** \brief Construct the zero polynomial carrying \e size coefficients, for the caller to fill in
            through \ref operator[].

            Filling the polynomial in place can sometimes help avoid calling memcpy.
        */
        explicit Polynomial(Eigen::Index size) : coefficients_(static_cast<std::size_t>(size), 0.)
        {
        }

        /** \brief Construct the polynomial whose ascending-power coefficients are \e coefficients. */
        Polynomial(std::initializer_list<double> coefficients) : coefficients_(coefficients)
        {
        }

        /** \brief The coefficient of \f$t^i\f$. */
        double &operator[](Eigen::Index i)
        {
            return coefficients_[static_cast<std::size_t>(i)];
        }

        /** \brief The coefficient of \f$t^i\f$. */
        double operator[](Eigen::Index i) const
        {
            return coefficients_[static_cast<std::size_t>(i)];
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
            Polynomial slope(std::max<Eigen::Index>(size() - 1, 1));
            for (Eigen::Index i = 1; i < size(); ++i)
                slope[i - 1] = static_cast<double>(i) * coefficients_[i];
            return slope;
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

            // Everything lands in one named result so the compiler can build it in the caller's storage.
            Roots roots = closedFormRoots(degree);
            for (double &root : roots)
                root = newton(root);
            if (roots.empty() ||
                !std::all_of(roots.begin(), roots.end(), [this](double root) { return vanishesAt(root); }))
            {
                roots = companionRoots(degree);
                for (double &root : roots)
                    root = newton(root);
            }
            return roots;
        }

        /** \brief Every strictly positive point at which the polynomial crosses from negative to positive.

            The polynomial this runs on is the derivative of a cost, so an upward crossing is a local
            minimum of that cost and a root the polynomial only touches is not.
            Demanding the crossing also throws out the spurious roots that a repeated root at zero
            produces.
        */
        Roots upwardCrossings() const
        {
            Roots crossings = roots();
            crossings.erase(std::remove_if(crossings.begin(), crossings.end(),
                                           [this](double root) { return !(root > 0. && evaluate(root).slope > 0.); }),
                            crossings.end());
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

        /** \brief The roots the closed form for \e degree finds, or none for a degree above four.

            \e degree has to be the index of a leading coefficient.
        */
        Roots closedFormRoots(Eigen::Index degree) const
        {
            switch (degree)
            {
                case 1:
                    return {-coefficients_[0] / coefficients_[1]};
                case 2:
                    return quadraticRoots(coefficients_[2], coefficients_[1], coefficients_[0]);
                case 3:
                    return cubicRoots(coefficients_[3], coefficients_[2], coefficients_[1], coefficients_[0]);
                case 4:
                    return quarticRoots(coefficients_[4], coefficients_[3], coefficients_[2], coefficients_[1],
                                        coefficients_[0]);
                default:
                    return {};
            }
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

    /** \brief Scratch room for a small matrix at any order other than 2. */
    using Scratch = boost::container::small_vector<double, 25>;

    /** \brief Run \e kernel with the order as a compile-time constant at 2, and as 0 at
        every other order, which tells the kernel to read the order at runtime.

        Unrolling the loops over derivative levels at order 2 makes steering there cost what the closed forms
        it replaced did.
    */
    template <class Kernel>
    decltype(auto) withOrder(Eigen::Index order, Kernel &&kernel)
    {
        if (order == 2)
            return kernel(std::integral_constant<int, 2>());
        return kernel(std::integral_constant<int, 0>());
    }

    /** \brief Room for \e size values, on the stack when \e Size is positive and in \ref Scratch
        otherwise. */
    template <int Size>
    auto allocBuffer(Eigen::Index size)
    {
        if constexpr (Size > 0)
            return std::array<double, static_cast<std::size_t>(Size)>();
        else
            return Scratch(static_cast<std::size_t>(size));
    }

    /** \brief The falling factorial i (i - 1) ... (i - count + 1), which differentiating \f$t^i\f$ \e count
        times leaves in front of it. */
    double fallingFactorial(Eigen::Index i, Eigen::Index count)
    {
        double product = 1.;
        for (Eigen::Index j = 0; j < count; ++j)
            product *= static_cast<double>(i - j);
        return product;
    }

    /** \brief The binomial coefficient C(n, r), which is exact while it stays below 2^53, since every partial
        product is itself a binomial coefficient. */
    double binomial(Eigen::Index n, Eigen::Index r)
    {
        double result = 1.;
        for (Eigen::Index i = 1; i <= r; ++i)
            result = result * static_cast<double>(n - r + i) / static_cast<double>(i);
        return result;
    }

    /** \brief \e base to the power \e exponent, by repeated multiplication. */
    double power(double base, Eigen::Index exponent)
    {
        double result = 1.;
        for (Eigen::Index i = 0; i < exponent; ++i)
            result *= base;
        return result;
    }

    /** \brief Write into \e shortfall what the Taylor expansion at \e from leaves undone at \e to along
        coordinate \e axis, as polynomials in the duration T.

        Entry \e l + \e k \e p holds the coefficient of \f$T^p\f$ in
            T^l yf^(l) - sum over i from l through k - 1 of y0^(i) T^i / (i - l)!,
        for \e l and \e p from 0 through \e k - 1.
        That polynomial is T^l times whatever of derivative level \e l at \e to the Taylor expansion at
        \e from leaves over, and scaling level \e l by T^l writes the shortfall in time scaled by the
        duration.
        Entries with \e p below \e l are zero.
    */
    template <int Order>
    void writeShortfall(const Eigen::Ref<const Eigen::MatrixXd> &from, const Eigen::Ref<const Eigen::MatrixXd> &to,
                        Eigen::Index axis, double *shortfall)
    {
        const Eigen::Index k = Order > 0 ? Order : from.rows();
        for (Eigen::Index l = 0; l < k; ++l)
        {
            for (Eigen::Index p = 0; p < l; ++p)
                shortfall[l + k * p] = 0.;
            shortfall[l + k * l] = to(l, axis);
            double factorial = 1.;
            for (Eigen::Index i = l; i < k; ++i)
            {
                if (i > l)
                {
                    factorial *= static_cast<double>(i - l);
                    shortfall[l + k * i] = 0.;
                }
                shortfall[l + k * i] -= from(i, axis) / factorial;
            }
        }
    }

    /** \brief The least control effort of a steer between two flat states of order \e k over a duration T,
        which comes to Q(T) / T^(2k - 1) for a polynomial Q of degree 2k - 2.

        With the shortfall at the far end written as polynomials s(T) by \ref writeShortfall, Q sums
        s(T)' W s(T) over every axis, where W is the effort form of the steering.
    */
    class Effort
    {
    public:
        /** \brief Construct the effort of steering from \e from to \e to under the effort form \e form.

            Inlining this into \ref ompl::base::MinimumEffortSteering::optimalDuration made steering at
            order 2 about 10 percent slower.
        */
        BOOST_NOINLINE Effort(const Eigen::MatrixXd &form, const Eigen::Ref<const Eigen::MatrixXd> &from,
                              const Eigen::Ref<const Eigen::MatrixXd> &to)
          : order_(from.rows()), numerator_(2 * from.rows() - 1)
        {
            withOrder(order_, [&](auto order) { accumulate<decltype(order)::value>(form.data(), from, to); });
        }

        /** \brief The cost J(T) of steering over \e duration with time penalty \e rho. */
        double cost(double duration, double rho) const
        {
            return numerator_(duration) / power(duration, 2 * order_ - 1) + rho * duration;
        }

        /** \brief T^(2k) times the derivative of \ref cost with respect to T, which is a polynomial of
            degree 2k and so has the same positive roots and signs. */
        Polynomial slope(double rho) const
        {
            // T Q'(T) + (1 - 2k) Q(T) + rho T^(2k), taken a power at a time.
            Polynomial result(2 * order_ + 1);
            for (Eigen::Index m = 0; m < 2 * order_ - 1; ++m)
                result[m] = static_cast<double>(m + 1 - 2 * order_) * numerator_[m];
            result[2 * order_] = rho;
            return result;
        }

    private:
        /** \brief Add the effort along every axis into \ref numerator_, reading the effort form from the
            column-major \e form. */
        template <int Order>
        void accumulate(const double *form, const Eigen::Ref<const Eigen::MatrixXd> &from,
                        const Eigen::Ref<const Eigen::MatrixXd> &to)
        {
            const Eigen::Index k = Order > 0 ? Order : order_;
            auto shortfall = allocBuffer<Order * Order>(k * k);
            auto weighted = allocBuffer<Order * Order>(k * k);

            // Local copies tell the compiler that the numerator and the form never overlap, and they keep
            // the sum out of memory until the end.
            auto local = allocBuffer<Order * Order>(k * k);
            std::copy(form, form + k * k, local.begin());
            auto sums = allocBuffer<(Order > 0 ? 2 * Order - 1 : 0)>(2 * k - 1);
            std::fill(sums.begin(), sums.end(), 0.);
            for (Eigen::Index axis = 0; axis < from.cols(); ++axis)
            {
                writeShortfall<Order>(from, to, axis, shortfall.data());

                // Row l of the shortfall vanishes below power l, which the loops skip.
                for (Eigen::Index p = 0; p < k; ++p)
                    for (Eigen::Index l = 0; l < k; ++l)
                    {
                        double sum = 0.;
                        for (Eigen::Index m = 0; m <= p; ++m)
                            sum += local[l + k * m] * shortfall[m + k * p];
                        weighted[l + k * p] = sum;
                    }

                for (Eigen::Index p = 0; p < k; ++p)
                    for (Eigen::Index q = 0; q < k; ++q)
                    {
                        double sum = 0.;
                        for (Eigen::Index l = 0; l <= p; ++l)
                            sum += shortfall[l + k * p] * weighted[l + k * q];
                        sums[p + q] += sum;
                    }
            }
            for (Eigen::Index m = 0; m < 2 * k - 1; ++m)
                numerator_[m] = sums[m];
        }

        /** \brief The number of derivative levels in each flat state. */
        Eigen::Index order_;

        /** \brief The polynomial Q. */
        Polynomial numerator_;
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
        Polynomial v_squared(std::max<Eigen::Index>(2 * rows - 3, 1));
        for (Eigen::Index i = 1; i < rows; ++i)
        {
            const double weight = static_cast<double>(i);
            v_squared[2 * (i - 1)] += weight * weight * coefficients_.row(i).squaredNorm();
            for (Eigen::Index j = i + 1; j < rows; ++j)
                v_squared[i + j - 2] +=
                    2. * weight * static_cast<double>(j) * coefficients_.row(i).dot(coefficients_.row(j));
        }

        double peak = std::max(v_squared(0.), v_squared(duration_));
        for (double t : v_squared.derivative().roots())
            if (t > 0. && t < duration_)
                peak = std::max(peak, v_squared(t));
        return std::sqrt(std::max(peak, 0.));
    }

    FlatSteering::FlatSteering(unsigned int order) : order_(order)
    {
        if (order < 1u)
            throw Exception("FlatSteering needs an order of at least 1");

        // In time scaled by the duration, the top k coefficients c_k through c_(2k - 1) of a steer meet
        // derivative level l at the far end through the matrix M_lj = (k + j)! / (k + j - l)!, which doesn't
        // depend on the duration.
        // Writing M_lj = l! C(k + j, l) and splitting C(k + j, l) by Vandermonde's identity factors M into
        // diag(l!) times a lower triangular Toeplitz matrix of C(k, l - r) times an upper triangular Pascal
        // matrix of C(j, r).
        // Both triangular factors invert in closed form, the Toeplitz one through the series of
        // (1 + x)^-k, which leaves
        //     (M^-1)_jl = (-1)^(j + l) / l! sum over r from max(j, l) through k - 1 of C(r, j) C(k - 1 + r - l, r - l).
        // The sum is an integer, so each entry rounds once, when it divides by l!.
        const auto k = static_cast<Eigen::Index>(order);
        endDerivativesInverse_.resize(k, k);
        for (Eigen::Index j = 0; j < k; ++j)
            for (Eigen::Index l = 0; l < k; ++l)
            {
                double sum = 0.;
                for (Eigen::Index r = std::max(j, l); r < k; ++r)
                    sum += binomial(r, j) * binomial(k - 1 + r - l, r - l);
                const double sign = (j + l) % 2 == 0 ? 1. : -1.;
                endDerivativesInverse_(j, l) = sign * sum / fallingFactorial(l, l);
            }

        // The effort form is the inverse of the controllability Gramian of a chain of k integrators over a
        // duration of 1, and reindexing that Gramian by i = k - 1 - l makes it F H F, where H is the
        // Hilbert matrix and F = diag(1 / i!).
        // So the form is diag(i!) H^-1 diag(i!), whose entries are integers, and the Hilbert inverse has
        // the closed form
        //     (H^-1)_ij = (-1)^(i + j) (i + j + 1) C(k + i, k - j - 1) C(k + j, k - i - 1) C(i + j, i)^2.
        // Every factor is an integer, so through order 8, where every entry stays below 2^53, this is exact.
        effortForm_.resize(k, k);
        for (Eigen::Index l = 0; l < k; ++l)
        {
            for (Eigen::Index m = 0; m < k; ++m)
            {
                const Eigen::Index i = k - 1 - l;
                const Eigen::Index j = k - 1 - m;
                const double sign = (i + j) % 2 == 0 ? 1. : -1.;
                const double center = binomial(i + j, i);
                effortForm_(l, m) = sign * fallingFactorial(i, i) * fallingFactorial(j, j) *
                                    static_cast<double>(i + j + 1) * binomial(k + i, k - j - 1) *
                                    binomial(k + j, k - i - 1) * center * center;
            }
        }
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

    Eigen::MatrixXd FlatSteering::coefficientsOver(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                   const Eigen::Ref<const Eigen::MatrixXd> &to, double duration) const
    {
        Eigen::MatrixXd coefficients(2 * static_cast<Eigen::Index>(order_), from.cols());
        withOrder(order_,
                  [&](auto order)
                  {
                      constexpr int Order = decltype(order)::value;
                      const Eigen::Index k = Order > 0 ? Order : static_cast<Eigen::Index>(order_);
                      const double T = duration;

                      // Scaling the top half back out of time scaled by T divides power k + a by T^(k + a).
                      auto scales = allocBuffer<Order>(k);
                      scales[0] = 1. / power(T, k);
                      for (Eigen::Index a = 1; a < k; ++a)
                          scales[a] = scales[a - 1] / T;

                      auto shortfall = allocBuffer<Order * Order>(k * k);
                      auto shortfallAt = allocBuffer<Order>(k);
                      for (Eigen::Index axis = 0; axis < from.cols(); ++axis)
                      {
                          // The Taylor expansion at from fixes the bottom half.
                          double factorial = 1.;
                          for (Eigen::Index i = 0; i < k; ++i)
                          {
                              if (i > 0)
                                  factorial *= static_cast<double>(i);
                              coefficients(i, axis) = from(i, axis) / factorial;
                          }

                          writeShortfall<Order>(from, to, axis, shortfall.data());
                          for (Eigen::Index l = 0; l < k; ++l)
                          {
                              double value = 0.;
                              for (Eigen::Index p = k - 1; p >= l; --p)
                                  value = value * T + shortfall[l + k * p];
                              shortfallAt[l] = value * power(T, l);
                          }

                          for (Eigen::Index a = 0; a < k; ++a)
                          {
                              double scaled = 0.;
                              for (Eigen::Index l = 0; l < k; ++l)
                                  scaled += endDerivativesInverse_(a, l) * shortfallAt[l];
                              coefficients(k + a, axis) = scaled * scales[a];
                          }
                      }
                  });
        return coefficients;
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
        return FlatMotion(coefficientsOver(from, to, duration_), duration_);
    }

    std::optional<double> FixedDurationSteering::steeringCost(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                              const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        checkFlatStates(from, to);
        return Effort(effortForm_, from, to).cost(duration_, rho_);
    }

    MinimumEffortSteering::MinimumEffortSteering(unsigned int order) : FlatSteering(order)
    {
    }

    std::optional<double> MinimumEffortSteering::steeringCost(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                              const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        const std::optional<OptimalDuration> cheapest = optimalDuration(from, to);
        if (!cheapest.has_value())
            return {};
        return cheapest->cost;
    }

    std::optional<MinimumEffortSteering::OptimalDuration> MinimumEffortSteering::optimalDuration(
        const Eigen::Ref<const Eigen::MatrixXd> &from, const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        checkFlatStates(from, to);

        // Clearing T^(2k) out of dJ/dT leaves a polynomial of degree 2k, which is a quartic at order 2.
        const Effort effort(effortForm_, from, to);
        const Polynomial slope = effort.slope(rho_);

        // The slope can cross upward more than once, which puts several local minima on J, so each crossing
        // gets priced and the cheapest one wins.
        // Taking the first crossing instead would sometimes charge several times what the motion needs to
        // cost, and an optimizing planner would steer through it believing the price.
        std::optional<OptimalDuration> best;
        for (double t : slope.upwardCrossings())
        {
            const double cost = effort.cost(t, rho_);
            if (!best.has_value() || cost < best->cost)
                best = OptimalDuration{t, cost};
        }

        return best;
    }

    std::optional<FlatMotion> MinimumEffortSteering::steer(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                                           const Eigen::Ref<const Eigen::MatrixXd> &to) const
    {
        const std::optional<OptimalDuration> cheapest = optimalDuration(from, to);
        if (!cheapest.has_value())
            return {};
        return motion(from, to, cheapest->duration);
    }

    FlatMotion MinimumEffortSteering::motion(const Eigen::Ref<const Eigen::MatrixXd> &from,
                                             const Eigen::Ref<const Eigen::MatrixXd> &to, double duration) const
    {
        checkFlatStates(from, to);
        if (!(duration > 0.))
            throw Exception("MinimumEffortSteering needs a positive duration");
        return FlatMotion(coefficientsOver(from, to, duration), duration);
    }

    boost::container::small_vector<MinimumEffortSteering::WindingChange, 4> MinimumEffortSteering::windingChanges(
        const Eigen::Ref<const Eigen::MatrixXd> &from, const Eigen::Ref<const Eigen::MatrixXd> &to, unsigned int axis,
        double period, double horizon) const
    {
        checkFlatStates(from, to);

        boost::container::small_vector<WindingChange, 4> changes;
        if (!(period > 0.) || !(horizon > 0.) || !std::isfinite(horizon))
            return changes;

        // The displacement enters only the constant term of row 0 of the shortfall, so over a fixed duration
        // the effort is least where row 0 settles at minus the sum of form(0, m) times row m over form(0, 0).
        // That minimum, measured from the flat output of to in periods, is the ideal winding, and it puts
        // the halfway points between whole windings at the half integers.
        const auto k = static_cast<Eigen::Index>(order_);
        // Every division here sits on the path to the first crossing, so the reciprocals get taken up front
        // where they don't wait on anything.
        const double inversePeriod = 1. / period;
        const double inverseLead = 1. / effortForm_(0, 0);
        Polynomial idealWinding(k);
        withOrder(k,
                  [&](auto order)
                  {
                      constexpr int Order = decltype(order)::value;
                      auto shortfall = allocBuffer<Order * Order>(k * k);
                      writeShortfall<Order>(from, to, axis, shortfall.data());
                      for (Eigen::Index p = 0; p < k; ++p)
                      {
                          double settled = 0.;
                          for (Eigen::Index m = 1; m <= p; ++m)
                              settled -= effortForm_(0, m) * shortfall[m + k * p];
                          idealWinding[p] = (settled * inverseLead - shortfall[k * p]) * inversePeriod;
                      }
                  });

        // At order 2 the ideal winding grows linearly with the duration, so it only ever moves one way.
        // Each time it passes halfway between two whole windings, the nearest winding steps by one in that
        // direction.
        if (k == 2)
        {
            double winding = std::round(idealWinding[0]);
            if (winding != 0.)
                changes.push_back(WindingChange{0., winding});

            const double direction = idealWinding[1] > 0. ? 1. : -1.;
            if (!(idealWinding[1] != 0.))
                return changes;

            // A line crosses the halfway points at evenly spaced times.
            // Each time comes from the first one rather than from adding up spacings, so rounding doesn't
            // build up, and a velocity of NaN makes every time NaN, which fails the loop condition and ends
            // the walk rather than spinning forever.
            const double pace = 1. / idealWinding[1];
            const double first = (winding + 0.5 * direction - idealWinding[0]) * pace;
            const double spacing = std::abs(pace);
            unsigned long crossings = 0;
            for (double time = first; time < horizon; time = first + static_cast<double>(crossings) * spacing)
            {
                winding += direction;
                changes.push_back(WindingChange{time, winding});
                ++crossings;
            }
            return changes;
        }

        double low = std::min(idealWinding(0.), idealWinding(horizon));
        double high = std::max(idealWinding(0.), idealWinding(horizon));
        for (double t : idealWinding.derivative().roots())
            if (t > 0. && t < horizon)
            {
                low = std::min(low, idealWinding(t));
                high = std::max(high, idealWinding(t));
            }
        // A velocity of NaN ends the search rather than spinning forever.
        if (!std::isfinite(low) || !std::isfinite(high))
            return changes;

        Roots crossings;
        for (double halfway = std::ceil(low - 0.5) + 0.5; halfway < high; halfway += 1.)
        {
            Polynomial shifted = idealWinding;
            shifted[0] -= halfway;
            for (double t : shifted.roots())
                if (t > 0. && t < horizon)
                    crossings.push_back(t);
        }
        std::sort(crossings.begin(), crossings.end());
        crossings.erase(std::unique(crossings.begin(), crossings.end()), crossings.end());
        crossings.push_back(horizon);

        // The nearest winding stays constant between two neighboring crossings, so rounding the ideal winding
        // at the midpoint between them gives the winding for that whole interval.
        // Counting crossings doesn't work, since the ideal winding can touch a halfway point and turn back
        // without changing the nearest winding.
        double winding = 0.;
        double start = 0.;
        for (double end : crossings)
        {
            const double nearest = std::round(idealWinding(0.5 * (start + end)));
            if (nearest != winding)
            {
                changes.push_back(WindingChange{start, nearest});
                winding = nearest;
            }
            start = end;
        }
        return changes;
    }
}  // namespace ompl::base
