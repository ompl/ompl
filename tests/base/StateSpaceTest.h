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

/* Author: Ioan Sucan */

#include <boost/test/unit_test.hpp>
#include "ompl/base/StateSpace.h"
#include "ompl/base/ScopedState.h"
#include "ompl/util/RandomNumbers.h"
#include <limits>

// define a convenience macro
#ifndef BOOST_OMPL_EXPECT_NEAR
#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)
#endif

namespace ompl
{

    /** \brief Encapsulate basic tests for state spaces. This class
        should be used for every state space included with ompl, to
        ensure basic functionality works. */
    class StateSpaceTest
    {
    public:

        /** \brief Construct a testing setup for state space \e
            space. When samples need to be taken to ensure certain
            functionality works, \e n samples are to be drawn. For
            distance comparisons, use an error margin of \e eps. */
        StateSpaceTest(const base::StateSpacePtr &space, int n = 1000, double eps = std::numeric_limits<double>::epsilon() * 10.0) :
            space_(space), n_(n), eps_(eps)
        {
        }

        ~StateSpaceTest()
        {
        }

        /** \brief Test that distances are always positive */
        void testDistance()
        {
            base::ScopedState<> s1(space_);
            base::ScopedState<> s2(space_);

            for (int i = 0 ; i < n_ ; ++i)
            {
                s1.random();
                BOOST_OMPL_EXPECT_NEAR(s1.distance(s1), 0.0, eps_);
                s2.random();
                if (s1 != s2)
                {
                    double d12 = s1.distance(s2);
                    BOOST_CHECK(d12 > 0.0);
                    double d21 = s2.distance(s1);
                    BOOST_OMPL_EXPECT_NEAR(d12, d21, eps_);
                }
            }
        }

        /** \brief Test that interpolation works as expected and also test triangle inequality */
        void testInterpolation()
        {
            base::ScopedState<> s1(space_);
            base::ScopedState<> s2(space_);
            base::ScopedState<> s3(space_);

            for (int i = 0 ; i < n_ ; ++i)
            {
                s1.random(); s2.random(); s3.random();

                space_->interpolate(s1.get(), s2.get(), 0.0, s3.get());
                BOOST_OMPL_EXPECT_NEAR(s1.distance(s3), 0.0, eps_);

                space_->interpolate(s1.get(), s2.get(), 1.0, s3.get());
                BOOST_OMPL_EXPECT_NEAR(s2.distance(s3), 0.0, eps_);

                space_->interpolate(s1.get(), s2.get(), 0.5, s3.get());
                BOOST_OMPL_EXPECT_NEAR(s1.distance(s3) + s3.distance(s2), s1.distance(s2), eps_);

                space_->interpolate(s3.get(), s2.get(), 0.5, s3.get());
                space_->interpolate(s1.get(), s2.get(), 0.75, s2.get());
                BOOST_OMPL_EXPECT_NEAR(s2.distance(s3), 0.0, eps_);
            }
        }
        
        /** \brief Test that states are correctly cloned*/
        void testCloneState()
        {
            base::ScopedState<> source(space_);
            source.random();
            const base::State* clonedState = space_->cloneState(source.get());
            //Make sure the cloned state is actually a new state in memory.
            BOOST_CHECK(clonedState != source.get());
            //Make sure the states are the same.
            BOOST_CHECK(space_->equalStates(clonedState, source.get()));
        }

        /** \brief Call all tests for the state space */
        void test()
        {
            testDistance();
            testInterpolation();
            testCloneState();
        }

    private:

        base::StateSpacePtr           space_;
        RNG                           rng_;

        int                           n_;
        double                        eps_;
    };
}
