/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_SQRT_APPROX_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_SQRT_APPROX_

#include "ompl/datastructures/NearestNeighborsLinear.h"
#include <algorithm>
#include <cmath>

namespace ompl
{
    /** \brief A nearest neighbors datastructure that uses linear
        search. The linear search is done over sqrt(n) elements
        only. (Every sqrt(n) elements are skipped).

        \li Search for nearest neighbor is O(sqrt(n)).
        \li Search for k-nearest neighbors is  O(n log(k)).
        \li Search for neighbors within a range is O(n log(n)).
        \li Adding an element to the datastructure is O(1).
        \li Removing an element from the datastructure O(n).
    */
    template <typename _T>
    class NearestNeighborsSqrtApprox : public NearestNeighborsLinear<_T>
    {
    public:
        NearestNeighborsSqrtApprox() = default;

        ~NearestNeighborsSqrtApprox() override = default;

        void clear() override
        {
            NearestNeighborsLinear<_T>::clear();
            checks_ = 0;
            offset_ = 0;
        }

        void add(const _T &data) override
        {
            NearestNeighborsLinear<_T>::add(data);
            updateCheckCount();
        }

        void add(const std::vector<_T> &data) override
        {
            NearestNeighborsLinear<_T>::add(data);
            updateCheckCount();
        }

        bool remove(const _T &data) override
        {
            bool result = NearestNeighborsLinear<_T>::remove(data);
            if (result)
                updateCheckCount();
            return result;
        }

        _T nearest(const _T &data) const override
        {
            const std::size_t n = NearestNeighborsLinear<_T>::data_.size();
            std::size_t pos = n;

            if (checks_ > 0 && n > 0)
            {
                double dmin = 0.0;
                for (std::size_t j = 0; j < checks_; ++j)
                {
                    std::size_t i = (j * checks_ + offset_) % n;

                    double distance = NearestNeighbors<_T>::distFun_(NearestNeighborsLinear<_T>::data_[i], data);
                    if (pos == n || dmin > distance)
                    {
                        pos = i;
                        dmin = distance;
                    }
                }
                offset_ = (offset_ + 1) % checks_;
            }
            if (pos != n)
                return NearestNeighborsLinear<_T>::data_[pos];

            throw Exception("No elements found in nearest neighbors data structure");
        }

    protected:
        /** \brief The maximum number of checks to perform when searching for a nearest neighbor */
        inline void updateCheckCount()
        {
            checks_ = 1 + (std::size_t)floor(sqrt((double)NearestNeighborsLinear<_T>::data_.size()));
        }

        /** \brief The number of checks to be performed when looking for a nearest neighbor */
        std::size_t checks_{0};

        /** \brief The offset to start checking at (between 0 and \e checks_) */
        mutable std::size_t offset_{0};
    };
}

#endif
