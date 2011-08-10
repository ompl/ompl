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
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>
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
            KStrategy(const unsigned int k,
                      const boost::shared_ptr< NearestNeighbors<Milestone> > &nn) :
                k_(k), nn_(nn)
            {
                neighbors_.reserve(k_);
            }

            virtual ~KStrategy(void)
            {
            }

            /** \brief Set the nearest neighbors datastructure to use */
            void setNearestNeighbors(const boost::shared_ptr< NearestNeighbors<Milestone> > &nn)
            {
                nn_ = nn;
            }

            /** \brief Given a milestone \e m, find the nearest
                neighbors attempts of connection should be made to,
                according to the connection strategy */
            std::vector<Milestone>& operator()(const Milestone& m)
            {
                nn_->nearestK(m, k_, neighbors_);
                return neighbors_;
            }

        protected:

            /** \brief Maximum number of nearest neighbors to attempt to connect new milestones to */
            unsigned int                                     k_;

            /** \brief Nearest neighbors data structure */
            boost::shared_ptr< NearestNeighbors<Milestone> > nn_;

            /** \brief Scratch space for storing k-nearest neighbors */
            std::vector<Milestone>                           neighbors_;
        };

        /**
         * Make the minimal number of connections required to ensure
         * asymptotic optimality.
         *
         * @par External documentation
         * S. Karaman and E. Frazzoli
         * Incremental Sampling-based Algorithms for Optimal Motion Planning,
         * <em>Int. Journal of Robotics Research</em> (Submitted 2010)
         */
        template <class Milestone>
        class KStarStrategy : public KStrategy<Milestone>
        {
        public:

            /** \brief Constructor takes a function that retunrs the
                maximum number of nearest neighbors to return at a
                particular time (\e n) and the nearest neighbors
                datastruture to use (\e nn) */
            KStarStrategy(const boost::function<unsigned int()>& n,
                          const boost::shared_ptr< NearestNeighbors<Milestone> > &nn) :
                KStrategy<Milestone>(n(), nn), n_(n)
            {
            }

            std::vector<Milestone>& operator()(const Milestone& m)
            {
                KStrategy<Milestone>::k_ = ceil(2.0 * boost::math::constants::euler<double>() * log((double)n_()));
                return static_cast<KStrategy<Milestone>&>(*this)(m);
            }

        protected:

            /** \brief Function returning the number of milestones added so far */
            const boost::function<unsigned int()> n_;

        };

    }

}

#endif
