/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Rice University
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

#ifndef OMPL_UTIL_HASH_
#define OMPL_UTIL_HASH_

#include <functional>
#include <type_traits>
#include <utility>
#include <vector>
#include <boost/functional/hash.hpp>

namespace ompl
{
    // copied from <boost/functional/hash.hpp>
    template <class T>
    inline void hash_combine(std::size_t &seed, const T &v)
    {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
}  // namespace ompl

namespace std
{
    template <class U, class V>
    struct hash<std::pair<U, V>>
    {
        using argument_type = std::pair<U, V>;
        using result_type = std::size_t;
        result_type operator()(argument_type const &p) const
        {
            result_type h = std::hash<std::remove_cv_t<U>>()(p.first);
            ompl::hash_combine(h, p.second);
            return h;
        }
    };

    template <class T>
    struct hash<std::vector<T>>
    {
        using argument_type = std::vector<T>;
        using result_type = std::size_t;
        result_type operator()(argument_type const &v) const
        {
            result_type h = 0;
            boost::hash_range(h, v.begin(), v.end());
            return h;
        }
    };
}  // namespace std

#endif
