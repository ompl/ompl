/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: James D. Marble */

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_CONNECTION_STRATEGY_
#define OMPL_GEOMETRIC_PLANNERS_PRM_CONNECTION_STRATEGY_

#include "ompl/datastructures/NearestNeighbors.h"
#include <functional>
#include <memory>
#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <utility>
#include <vector>

namespace ompl
{
    namespace geometric
    {
        /**
         * Attempt to connect to the k nearest neighbors.
         */
        template <class Milestone>
        class KStrategy
        {
        public:
            /** \brief Constructor takes the maximum number of nearest neighbors to return (\e k) and the
                nearest neighbors datastruture to use (\e nn) */
            KStrategy(const unsigned int k, std::shared_ptr<NearestNeighbors<Milestone>> nn) : k_(k), nn_(std::move(nn))
            {
                neighbors_.reserve(k_);
            }

            virtual ~KStrategy() = default;

            /** \brief Set the nearest neighbors datastructure to use */
            void setNearestNeighbors(const std::shared_ptr<NearestNeighbors<Milestone>> &nn)
            {
                nn_ = nn;
            }

            /** \brief Given a milestone \e m, find the number of nearest
                neighbors connection attempts that should be made from it,
                according to the connection strategy */
            const std::vector<Milestone> &operator()(const Milestone &m)
            {
                nn_->nearestK(m, k_, neighbors_);
                return neighbors_;
            }

            unsigned int getNumNeighbors() const
            {
                return k_;
            }
        protected:
            /** \brief Maximum number of nearest neighbors to attempt to connect new milestones to */
            unsigned int k_;

            /** \brief Nearest neighbors data structure */
            std::shared_ptr<NearestNeighbors<Milestone>> nn_;

            /** \brief Scratch space for storing k-nearest neighbors */
            std::vector<Milestone> neighbors_;
        };

        /**
         * \brief Make the minimal number of connections required to ensure
         * asymptotic optimality.
         *
         * This connection strategy attempts to connect a milestone to its
         * k-nearest neighbors where k is a function of the number of milestones
         * that have already been added to the roadmap (n).
         *
         * k(n) = kPRMConstant * log(n)
         *
         * where
         *
         * kPRMConstant > kStarPRMConstant = e(1 + 1/d)
         *
         * and d is the number of dimensions in the state space. Note that
         * kPRMConstant = 2e is a valid choice for any problem instance and so,
         * if d is not provided, this value is used.
         *
         * The user must provide a function that returns the value of n.
         *
         * @par External documentation
         * S. Karaman and E. Frazzoli
         * Sampling-based algorithms for optimal motion planning,
         * <em>Int. Journal of Robotics Research</em> Volume 30, Number 7, June 2010
         */
        template <class Milestone>
        class KStarStrategy : public KStrategy<Milestone>
        {
        public:
            using NumNeighborsFn = std::function<unsigned int()>;
            /**
             * \brief Constructor
             *
             * \param n a function that returns the number of milestones that have already been added to the roadmap
             * \param nn the nearest neighbors datastruture to use
             * \param d the dimensionality of the state space.
             * The default is 1, which will make kPRMConstant=2e which
             * is valid for all problem instances.
             */
            KStarStrategy(const NumNeighborsFn &n, const std::shared_ptr<NearestNeighbors<Milestone>> &nn,
                          const unsigned int d = 1)
              : KStrategy<Milestone>(n(), nn)
              , n_(n)
              , kPRMConstant_(boost::math::constants::e<double>() + (boost::math::constants::e<double>() / (double)d))
            {
            }

            const std::vector<Milestone> &operator()(const Milestone &m)
            {
                KStrategy<Milestone>::k_ = static_cast<unsigned int>(ceil(kPRMConstant_ * log((double)n_())));
                return static_cast<KStrategy<Milestone> &>(*this)(m);
            }

        protected:
            /** \brief Function returning the number of milestones added to the roadmap so far */
            const NumNeighborsFn n_;
            const double kPRMConstant_;
        };

        /**
         * \brief Return at most k neighbors, as long as they are also within a specified bound.
         */
        template <class Milestone>
        class KBoundedStrategy : public KStrategy<Milestone>
        {
        public:
            /**
             * \brief Constructor
             *
             * \param k the maximum number of nearest neighbors to return
             * \param bound the maximum distance for any nearest neighbor to be returned
             * \param nn the nearest neighbors datastruture to use
             */
            KBoundedStrategy(const unsigned int k, const double bound,
                             const std::shared_ptr<NearestNeighbors<Milestone>> &nn)
              : KStrategy<Milestone>(k, nn), bound_(bound)
            {
            }

            const auto &operator()(const Milestone &m)
            {
                auto &result = KStrategy<Milestone>::neighbors_;
                KStrategy<Milestone>::nn_->nearestK(m, KStrategy<Milestone>::k_, result);
                if (result.empty())
                    return result;
                const auto &dist = KStrategy<Milestone>::nn_->getDistanceFunction();
                if (!KStrategy<Milestone>::nn_->reportsSortedResults())
                    std::sort(result.begin(), result.end(), dist);
                auto newCount = result.size();
                while (newCount > 0 && dist(result[newCount - 1], m) > bound_)
                    --newCount;
                result.resize(newCount);
                return result;
            }

        protected:
            /** \brief The maximum distance at which nearby milestones are reported */
            const double bound_;
        };
    }
}

#endif
