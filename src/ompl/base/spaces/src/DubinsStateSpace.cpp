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
#include "ompl/util/Exception.h"
#include <boost/math/constants/constants.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/unordered_map.hpp>
#include <boost/bimap.hpp>
#include <boost/bimap/list_of.hpp>
#include <boost/bimap/set_of.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/function.hpp>

#define DUBINS_EPS 1e-5

using namespace ompl::base;

namespace dubins
{
    const double twopi = 2. * boost::math::constants::pi<double>();
    inline double mod2pi(double x)
    {
        return x - twopi * floor(x / twopi);
    }

    DubinsStateSpace::DubinsPath dubinsLSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d*d - 2.*(ca*cb +sa*sb - d*(sa - sb));
        if (tmp >= 0.)
        {
            double theta = atan2(cb - ca, d + sa - sb);
            double t = mod2pi(-alpha + theta);
            double p = sqrt(tmp);
            double q = mod2pi(beta - theta);
            assert(fabs(p*cos(alpha + t) - sa + sb - d) < DUBINS_EPS);
            assert(fabs(p*sin(alpha + t) + ca - cb) < DUBINS_EPS);
            assert(mod2pi(alpha + t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[0], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    DubinsStateSpace::DubinsPath dubinsRSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d*d - 2.*(ca*cb + sa*sb - d*(sb - sa));
        if (tmp >= 0.)
        {
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta);
            double p = sqrt(tmp);
            double q = mod2pi(-beta + theta);
            assert(fabs(p*cos(alpha - t) + sa - sb - d) < DUBINS_EPS);
            assert(fabs(p*sin(alpha - t) - ca + cb) < DUBINS_EPS);
            assert(mod2pi(alpha - t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[1], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    DubinsStateSpace::DubinsPath dubinsRSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = d * d - 2. + 2. * (ca*cb + sa*sb - d * (sa + sb));
        if (tmp >= 0.)
        {
            double p = sqrt(tmp);
            double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
            double t = mod2pi(alpha - theta);
            double q = mod2pi(beta - theta);
            assert(fabs(p*cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) < DUBINS_EPS);
            assert(fabs(p*sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) < DUBINS_EPS);
            assert(mod2pi(alpha - t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[2], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    DubinsStateSpace::DubinsPath dubinsLSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = -2. + d * d + 2. * (ca*cb + sa*sb + d * (sa + sb));
        if (tmp >= 0.)
        {
            double p = sqrt(tmp);
            double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
            double t = mod2pi(-alpha + theta);
            double q = mod2pi(-beta + theta);
            assert(fabs(p*cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) < DUBINS_EPS);
            assert(fabs(p*sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) < DUBINS_EPS);
            assert(mod2pi(alpha + t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[3], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    DubinsStateSpace::DubinsPath dubinsRLR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d  + 2. * (ca*cb + sa*sb + d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = acos(tmp);
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta + .5 * p);
            double q = mod2pi(alpha - beta - t + p);
            assert(fabs( 2.*sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) < DUBINS_EPS);
            assert(fabs(-2.*cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) < DUBINS_EPS);
            assert(mod2pi(alpha - t + p - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[4], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    DubinsStateSpace::DubinsPath dubinsLRL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d  + 2. * (ca*cb + sa*sb - d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = acos(tmp);
            double theta = atan2(-ca + cb, d + sa - sb);
            double t = mod2pi(-alpha + theta + .5 * p);
            double q = mod2pi(beta - alpha - t + p);
            assert(fabs(-2.*sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) < DUBINS_EPS);
            assert(fabs( 2.*cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) < DUBINS_EPS);
            assert(mod2pi(alpha + t - p + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[5], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    struct TripleDouble
    {
        TripleDouble(double d, double alpha, double beta) : tuple_(d, alpha, beta)
        {
        }
        boost::tuple<double,double,double> tuple_;
    };
    struct TripleDoubleHash : public std::unary_function<TripleDouble, std::size_t>
    {
        std::size_t operator()(TripleDouble const& t) const
        {
            return hasher_(100.*t.tuple_.get<0>() + 10.*t.tuple_.get<1>() + t.tuple_.get<0>());
        }

        boost::hash<double> hasher_;
    };
    struct TripleDoubleEqual : public std::binary_function<TripleDouble, TripleDouble, bool>
    {
        bool operator()(TripleDouble const& t0, TripleDouble const& t1) const
        {
            return t0.tuple_.get<0>()==t1.tuple_.get<0>() &&
                   t0.tuple_.get<1>()==t1.tuple_.get<1>() &&
                   t0.tuple_.get<2>()==t1.tuple_.get<2>();
        }
    };
    DubinsStateSpace::DubinsPath dubins(const TripleDouble t)
    {
        double d = t.tuple_.get<0>(), alpha = t.tuple_.get<1>(), beta = t.tuple_.get<2>();
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
        {
            minLength = len;
            path = tmp;
        }
        return path;
    }


    // code taken from http://www.bottlenose.demon.co.uk/article/lru.htm
    template <typename K, typename V>
    class LRUCache
    {
    public:

        typedef K key_type;
        typedef V value_type;

        typedef boost::bimaps::bimap<
            boost::bimaps::unordered_set_of<key_type, TripleDoubleHash, TripleDoubleEqual>,
            boost::bimaps::list_of<value_type>
            > container_type;

        // Constuctor specifies the cached function and
        // the maximum number of records to be stored.
        LRUCache(const boost::function<value_type(const key_type&)>& f, size_t c)
            : fn_(f), capacity_(c)
        {
            assert(capacity_!=0);
        }

        // Obtain value of the cached function for k
        value_type operator()(const key_type& k) {
            // Attempt to find existing record
            const typename container_type::left_iterator it =container_.left.find(k);
            if (it==container_.left.end())
            {
                const value_type v=fn_(k);
                insert(k,v);
                return v;
            }
            else
            {
                container_.right.relocate(container_.right.end(), container_.project_right(it));
                return it->second;
            }
        }

    private:
        void insert(const key_type& k,const value_type& v) {
            assert(container_.size() <= capacity_);
            if (container_.size() == capacity_)
                // by purging the least-recently-used element
                container_.right.erase(container_.right.begin());

            // Create a new record from the key and the value
            // bimap's list_view defaults to inserting this at
            // the list tail (considered most-recently-used).
            container_.insert(typename container_type::value_type(k,v));
        }

        const boost::function<value_type(const key_type&)> fn_;
        const size_t capacity_;
        container_type container_;
    };
}

const ompl::base::DubinsStateSpace::DubinsPathSegmentType
ompl::base::DubinsStateSpace::dubinsPathType[6][3] = {
    { DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT },
    { DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT },
    { DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT },
    { DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT },
    { DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT },
    { DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT }
};

double ompl::base::DubinsStateSpace::distance(const State *state1, const State *state2) const
{
    return rho_ * std::min(DubinsPath(dubins(state1, state2)).length(),
        DubinsPath(dubins(state2, state1)).length());
}

void ompl::base::DubinsStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    if (t>=1.)
    {
        copyState(state, to);
        return;
    }
    copyState(state, from);
    if (t<=0.)
        return;

    StateType *s  = state->as<StateType>();
    DubinsPath path1(dubins(from, to)), path2(dubins(to, from));
    double len1 = path1.length(), len2 = path2.length(), seg, phi, v;

    s->setXY(0., 0.);
    s->setYaw(from->as<StateType>()->getYaw());
    if (len1 < len2)
    {
        seg = t * len1;
        for (unsigned int i=0; i<3 && seg>0; ++i)
        {
            v = std::min(seg, path1.length_[i]);
            phi = s->getYaw();
            seg -= v;
            switch(path1.type_[i])
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + sin(phi+v) - sin(phi), s->getY() - cos(phi+v) + cos(phi));
                    s->setYaw(phi+v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - sin(phi-v) + sin(phi), s->getY() + cos(phi-v) - cos(phi));
                    s->setYaw(phi-v);
                    break;
                case DUBINS_STRAIGHT:
                    s->setXY(s->getX() + v * cos(phi), s->getY() + v * sin(phi));
                    break;
            }
        }
    }
    else
    {
        seg = t * len2;
        for (unsigned int i=0; i<3 && seg>0; ++i)
        {
            v = std::min(seg, path2.length_[2-i]);
            phi = s->getYaw();
            seg -= v;
            switch(path2.type_[2-i])
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + sin(phi-v) - sin(phi), s->getY() - cos(phi-v) + cos(phi));
                    s->setYaw(phi-v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - sin(phi+v) + sin(phi), s->getY() + cos(phi+v) - cos(phi));
                    s->setYaw(phi+v);
                    break;
                case DUBINS_STRAIGHT:
                    s->setXY(s->getX() - v * cos(phi), s->getY() - v * sin(phi));
                    break;
            }
        }
    }
    s->setX(s->getX() * rho_ + from->as<StateType>()->getX());
    s->setY(s->getY() * rho_ + from->as<StateType>()->getY());
    getSubSpace(1)->enforceBounds(s->as<SO2StateSpace::StateType>(1));
}

ompl::base::DubinsStateSpace::DubinsPath ompl::base::DubinsStateSpace::dubins(const State *state1, const State *state2) const
{
    static ::dubins::LRUCache< ::dubins::TripleDouble, DubinsPath> cache(::dubins::dubins, cacheSize_);
    const StateType *s1 = static_cast<const StateType*>(state1);
    const StateType *s2 = static_cast<const StateType*>(state2);
    double x1 = s1->getX(), y1 = s1->getY(), th1 = s1->getYaw();
    double x2 = s2->getX(), y2 = s2->getY(), th2 = s2->getYaw();
    double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx*dx + dy*dy) / rho_, th = atan2(dy, dx);
    double alpha = ::dubins::mod2pi(th1 - th), beta = ::dubins::mod2pi(th2 - th);
    ::dubins::TripleDouble t(d,alpha,beta);
    return cache(t);
}

