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

/* Author: Mark Moll */

#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <boost/math/constants/constants.hpp>

using namespace ompl::base;

namespace
{
    const double twopi = 2. * boost::math::constants::pi<double>();
    const double DUBINS_EPS = 1e-6;
    const double DUBINS_ZERO = -1e-7;

    enum DubinsClass
    {
        A11 = 0,
        A12 = 1,
        A13 = 2,
        A14 = 3,
        A21 = 4,
        A22 = 5,
        A23 = 6,
        A24 = 7,
        A31 = 8,
        A32 = 9,
        A33 = 10,
        A34 = 11,
        A41 = 12,
        A42 = 13,
        A43 = 14,
        A44 = 15
    };

    inline double mod2pi(double x)
    {
        if (x < 0 && x > DUBINS_ZERO)
            return 0;
        double xm = x - twopi * floor(x / twopi);
        if (twopi - xm < .5 * DUBINS_EPS)
            xm = 0.;
        return xm;
    }

    inline double t_lsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.0));
        const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0, p);
        return mod2pi(-alpha + theta);  // t
    }

    inline double p_lsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_lsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.0));
        const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0, p);
        return mod2pi(-beta + theta);  // q
    }

    inline double t_rsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.0));
        const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2.0, p);
        return mod2pi(alpha - theta);  // t
    }

    inline double p_rsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_rsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.));
        const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2.0, p);
        return mod2pi(beta - theta);  // q
    }

    inline double t_rsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(ca - cb, d - sa + sb);
        return mod2pi(alpha - theta);  // t
    }

    inline double p_rsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_rsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(ca - cb, d - sa + sb);
        return mod2pi(-beta + theta);  // q
    }

    inline double t_lsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(cb - ca, d + sa - sb);
        return mod2pi(-alpha + theta);  // t
    }

    inline double p_lsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_lsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(cb - ca, d + sa - sb);
        return mod2pi(beta - theta);  // q
    }

    inline double s_12(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (q_rsl(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_13(double d, double alpha, double beta)
    {  // t_rsr - pi
        return t_rsr(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_14_1(double d, double alpha, double beta)
    {
        return t_rsr(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_21(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (t_rsl(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_22_1(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (t_rsl(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_22_2(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (q_rsl(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_24(double d, double alpha, double beta)
    {
        return q_rsr(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_31(double d, double alpha, double beta)
    {
        return q_lsl(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_33_1(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (t_lsr(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_33_2(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (q_lsr(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_34(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (t_lsr(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_41_1(double d, double alpha, double beta)
    {
        return t_lsl(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_41_2(double d, double alpha, double beta)
    {
        return q_lsl(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_42(double d, double alpha, double beta)
    {
        return t_lsl(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_43(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (q_lsr(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    DubinsStateSpace::DubinsPath dubinsLSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(cb - ca, d + sa - sb);
            double t = mod2pi(-alpha + theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(beta - theta);
            assert(fabs(p * cos(alpha + t) - sa + sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[0], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsRSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(-beta + theta);
            assert(fabs(p * cos(alpha - t) + sa - sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[1], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsRSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
        if (tmp >= DUBINS_ZERO)
        {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
            double t = mod2pi(alpha - theta);
            double q = mod2pi(beta - theta);
            assert(fabs(p * cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[2], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsLSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
        if (tmp >= DUBINS_ZERO)
        {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
            double t = mod2pi(-alpha + theta);
            double q = mod2pi(-beta + theta);
            assert(fabs(p * cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[3], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsRLR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = twopi - acos(tmp);
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta + .5 * p);
            double q = mod2pi(alpha - beta - t + p);
            assert(fabs(2. * sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) < 2 * DUBINS_EPS);
            assert(fabs(-2. * cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t + p - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[4], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsLRL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = twopi - acos(tmp);
            double theta = atan2(-ca + cb, d + sa - sb);
            double t = mod2pi(-alpha + theta + .5 * p);
            double q = mod2pi(beta - alpha - t + p);
            assert(fabs(-2. * sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) < 2 * DUBINS_EPS);
            assert(fabs(2. * cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t - p + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[5], t, p, q);
        }
        return {};
    }

    bool is_longpath_case(double d, double alpha, double beta)
    {
        return (std::abs(std::sin(alpha)) + std::abs(std::sin(beta)) +
                std::sqrt(4 - std::pow(std::cos(alpha) + std::cos(beta), 2)) - d) < 0;
    }

    DubinsStateSpace::DubinsPath dubins_exhaustive(const double d, const double alpha, const double beta)
    {
        if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
            return {DubinsStateSpace::dubinsPathType[0], 0, d, 0};

        DubinsStateSpace::DubinsPath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
        double len, minLength = path.length();

        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRSL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLSR(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRLR(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLRL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
            path = tmp;
        return path;
    }

    DubinsClass getDubinsClass(const double alpha, const double beta)
    {
        int row(0), column(0);
        if (0 <= alpha && alpha <= boost::math::constants::half_pi<double>())
        {
            row = 1;
        }
        else if (boost::math::constants::half_pi<double>() < alpha && alpha <= boost::math::constants::pi<double>())
        {
            row = 2;
        }
        else if (boost::math::constants::pi<double>() < alpha && alpha <= 3 * boost::math::constants::half_pi<double>())
        {
            row = 3;
        }
        else if (3 * boost::math::constants::half_pi<double>() < alpha && alpha <= twopi)
        {
            row = 4;
        }

        if (0 <= beta && beta <= boost::math::constants::half_pi<double>())
        {
            column = 1;
        }
        else if (boost::math::constants::half_pi<double>() < beta && beta <= boost::math::constants::pi<double>())
        {
            column = 2;
        }
        else if (boost::math::constants::pi<double>() < beta && beta <= 3 * boost::math::constants::half_pi<double>())
        {
            column = 3;
        }
        else if (3 * boost::math::constants::half_pi<double>() < beta &&
                 beta <= 2.0 * boost::math::constants::pi<double>())
        {
            column = 4;
        }

        assert(row >= 1 && row <= 4 &&
               "alpha is not in the range of [0,2pi] in classifyPath(double alpha, double beta).");
        assert(column >= 1 && column <= 4 &&
               "beta is not in the range of [0,2pi] in classifyPath(double alpha, double beta).");
        assert((column - 1) + 4 * (row - 1) >= 0 && (column - 1) + 4 * (row - 1) <= 15 &&
               "class is not in range [0,15].");
        return (DubinsClass)((column - 1) + 4 * (row - 1));
    }
}  // namespace

DubinsStateSpace::DubinsPath dubins_classification(const double d, const double alpha, const double beta)
{
    if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
        return {DubinsStateSpace::dubinsPathType[0], 0, d, 0};
    // Dubins set classification scheme
    // Shkel, Andrei M., and Vladimir Lumelsky. "Classification of the Dubins set."
    //   Robotics and Autonomous Systems 34.4 (2001): 179-202.
    // Lim, Jaeyoung, et al. "Circling Back: Dubins set Classification Revisited."
    //   Workshop on Energy Efficient Aerial Robotic Systems, International Conference on Robotics and Automation 2023.
    //   2023.
    DubinsStateSpace::DubinsPath path;
    auto dubins_class = getDubinsClass(alpha, beta);
    switch (dubins_class)
    {
        case DubinsClass::A11:
        {
            path = dubinsRSL(d, alpha, beta);
            break;
        }
        case DubinsClass::A12:
        {
            if (s_13(d, alpha, beta) < 0.0)
            {
                path = (s_12(d, alpha, beta) < 0.0) ? dubinsRSR(d, alpha, beta) : dubinsRSL(d, alpha, beta);
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
                DubinsStateSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                if (path.length() > tmp.length())
                {
                    path = tmp;
                }
            }
            break;
        }
        case DubinsClass::A13:
        {
            if (s_13(d, alpha, beta) < 0.0)
            {
                path = dubinsRSR(d, alpha, beta);
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A14:
        {
            if (s_14_1(d, alpha, beta) > 0.0)
            {
                path = dubinsLSR(d, alpha, beta);
            }
            else if (s_24(d, alpha, beta) > 0.0)
            {
                path = dubinsRSL(d, alpha, beta);
            }
            else
            {
                path = dubinsRSR(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A21:
        {
            if (s_31(d, alpha, beta) < 0.0)
            {
                if (s_21(d, alpha, beta) < 0.0)
                {
                    path = dubinsLSL(d, alpha, beta);
                }
                else
                {
                    path = dubinsRSL(d, alpha, beta);
                }
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
                DubinsStateSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                if (path.length() > tmp.length())
                {
                    path = tmp;
                }
            }
            break;
        }
        case DubinsClass::A22:
        {
            if (alpha > beta)
            {
                path = (s_22_1(d, alpha, beta) < 0.0) ? dubinsLSL(d, alpha, beta) : dubinsRSL(d, alpha, beta);
            }
            else
            {
                path = (s_22_2(d, alpha, beta) < 0.0) ? dubinsRSR(d, alpha, beta) : dubinsRSL(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A23:
        {
            path = dubinsRSR(d, alpha, beta);
            break;
        }
        case DubinsClass::A24:
        {
            if (s_24(d, alpha, beta) < 0.0)
            {
                path = dubinsRSR(d, alpha, beta);
            }
            else
            {
                path = dubinsRSL(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A31:
        {
            if (s_31(d, alpha, beta) < 0.0)
            {
                path = dubinsLSL(d, alpha, beta);
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A32:
        {
            path = dubinsLSL(d, alpha, beta);
            break;
        }
        case DubinsClass::A33:
        {
            if (alpha < beta)
            {
                if (s_33_1(d, alpha, beta) < 0.0)
                {
                    path = dubinsRSR(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                }
            }
            else
            {
                if (s_33_2(d, alpha, beta) < 0.0)
                {
                    path = dubinsLSL(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                }
            }
            break;
        }
        case DubinsClass::A34:
        {
            if (s_24(d, alpha, beta) < 0.0)
            {
                if (s_34(d, alpha, beta) < 0.0)
                {
                    path = dubinsRSR(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                }
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
                DubinsStateSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                if (path.length() > tmp.length())
                {
                    path = tmp;
                }
            }
            break;
        }
        case DubinsClass::A41:
        {
            if (s_41_1(d, alpha, beta) > 0.0)
            {
                path = dubinsRSL(d, alpha, beta);
            }
            else if (s_41_2(d, alpha, beta) > 0.0)
            {
                path = dubinsLSR(d, alpha, beta);
            }
            else
            {
                path = dubinsLSL(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A42:
        {
            if (s_42(d, alpha, beta) < 0.0)
            {
                path = dubinsLSL(d, alpha, beta);
            }
            else
            {
                path = dubinsRSL(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A43:
        {
            if (s_42(d, alpha, beta) < 0.0)
            {
                if (s_43(d, alpha, beta) < 0.0)
                {
                    path = dubinsLSL(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                }
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
                DubinsStateSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                if (path.length() > tmp.length())
                {
                    path = tmp;
                }
            }
            break;
        }
        case DubinsClass::A44:
        {
            path = dubinsLSR(d, alpha, beta);
            break;
        }
    }
    return path;
}

DubinsStateSpace::DubinsPath dubins(double d, double alpha, double beta)
{
    if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
        return {DubinsStateSpace::dubinsPathType[0], 0, d, 0};
    alpha = mod2pi(alpha);
    beta = mod2pi(beta);
    return is_longpath_case(d, alpha, beta) ? ::dubins_classification(d, alpha, beta) :
                                              ::dubins_exhaustive(d, alpha, beta);
}

const ompl::base::DubinsStateSpace::DubinsPathSegmentType ompl::base::DubinsStateSpace::dubinsPathType[6][3] = {
    {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT},  {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT},
    {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT}, {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT},
    {DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT},    {DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT}};

double ompl::base::DubinsStateSpace::distance(const State *state1, const State *state2) const
{
    if (isSymmetric_)
        return rho_ * std::min(dubins(state1, state2).length(), dubins(state2, state1).length());
    return rho_ * dubins(state1, state2).length();
}

void ompl::base::DubinsStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    bool firstTime = true;
    DubinsPath path;
    interpolate(from, to, t, firstTime, path, state);
}

void ompl::base::DubinsStateSpace::interpolate(const State *from, const State *to, const double t, bool &firstTime,
                                               DubinsPath &path, State *state) const
{
    if (firstTime)
    {
        if (t >= 1.)
        {
            if (to != state)
                copyState(state, to);
            return;
        }
        if (t <= 0.)
        {
            if (from != state)
                copyState(state, from);
            return;
        }

        path = dubins(from, to);
        if (isSymmetric_)
        {
            DubinsPath path2(dubins(to, from));
            if (path2.length() < path.length())
            {
                path2.reverse_ = true;
                path = path2;
            }
        }
        firstTime = false;
    }
    interpolate(from, path, t, state);
}

void ompl::base::DubinsStateSpace::interpolate(const State *from, const DubinsPath &path, double t, State *state) const
{
    auto *s = allocState()->as<StateType>();
    double seg = t * path.length(), phi, v;

    s->setXY(0., 0.);
    s->setYaw(from->as<StateType>()->getYaw());
    if (!path.reverse_)
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[i]);
            phi = s->getYaw();
            seg -= v;
            switch (path.type_[i])
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + sin(phi + v) - sin(phi), s->getY() - cos(phi + v) + cos(phi));
                    s->setYaw(phi + v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - sin(phi - v) + sin(phi), s->getY() + cos(phi - v) - cos(phi));
                    s->setYaw(phi - v);
                    break;
                case DUBINS_STRAIGHT:
                    s->setXY(s->getX() + v * cos(phi), s->getY() + v * sin(phi));
                    break;
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[2 - i]);
            phi = s->getYaw();
            seg -= v;
            switch (path.type_[2 - i])
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + sin(phi - v) - sin(phi), s->getY() - cos(phi - v) + cos(phi));
                    s->setYaw(phi - v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - sin(phi + v) + sin(phi), s->getY() + cos(phi + v) - cos(phi));
                    s->setYaw(phi + v);
                    break;
                case DUBINS_STRAIGHT:
                    s->setXY(s->getX() - v * cos(phi), s->getY() - v * sin(phi));
                    break;
            }
        }
    }
    state->as<StateType>()->setX(s->getX() * rho_ + from->as<StateType>()->getX());
    state->as<StateType>()->setY(s->getY() * rho_ + from->as<StateType>()->getY());
    getSubspace(1)->enforceBounds(s->as<SO2StateSpace::StateType>(1));
    state->as<StateType>()->setYaw(s->getYaw());
    freeState(s);
}

unsigned int ompl::base::DubinsStateSpace::validSegmentCount(const State *state1, const State *state2) const
{
    return StateSpace::validSegmentCount(state1, state2);
}

ompl::base::DubinsStateSpace::DubinsPath ompl::base::DubinsStateSpace::dubins(const State *state1,
                                                                              const State *state2) const
{
    const auto *s1 = static_cast<const StateType *>(state1);
    const auto *s2 = static_cast<const StateType *>(state2);
    double x1 = s1->getX(), y1 = s1->getY(), th1 = s1->getYaw();
    double x2 = s2->getX(), y2 = s2->getY(), th2 = s2->getYaw();
    double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx * dx + dy * dy) / rho_, th = atan2(dy, dx);
    double alpha = mod2pi(th1 - th), beta = mod2pi(th2 - th);
    return ::dubins(d, alpha, beta);
}

void ompl::base::DubinsMotionValidator::defaultSettings()
{
    stateSpace_ = dynamic_cast<DubinsStateSpace *>(si_->getStateSpace().get());
    if (stateSpace_ == nullptr)
        throw Exception("No state space for motion validator");
}

bool ompl::base::DubinsMotionValidator::checkMotion(const State *s1, const State *s2,
                                                    std::pair<State *, double> &lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true, firstTime = true;
    DubinsStateSpace::DubinsPath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, firstTime, path, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool ompl::base::DubinsMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
        return false;

    bool result = true, firstTime = true;
    DubinsStateSpace::DubinsPath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2)
    {
        pos.emplace(1, nd - 1);

        /* temporary storage for the checked state */
        State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, firstTime, path, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.emplace(x.first, mid - 1);
            if (x.second > mid)
                pos.emplace(mid + 1, x.second);
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}
