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

#define BOOST_TEST_MODULE "Heap"
#include <boost/test/unit_test.hpp>
#include "ompl/datastructures/BinaryHeap.h"

using namespace ompl;

BOOST_AUTO_TEST_CASE(Simple)
{
    BinaryHeap<int> h;
    BOOST_CHECK(h.empty());
    h.insert(2);
    BOOST_CHECK(h.size() == 1);
    BinaryHeap<int>::Element *e2 = h.insert(3);
    BinaryHeap<int>::Element *e3 = h.insert(1);
    BOOST_CHECK(h.size() == 3);

    BOOST_CHECK(h.top() == e3);
    h.insert(9);
    h.insert(-2);
    h.insert(5);
    h.remove(e3);
    BOOST_CHECK(h.size() == 5);
    BOOST_CHECK(h.top()->data == -2);
    e2->data = -5;
    h.update(e2);
    BOOST_CHECK(h.top()->data == -5);

    std::vector<int> s;
    h.getContent(s);
    h.sort(s);
    BOOST_CHECK(s.size() == 5);

    BOOST_CHECK_EQUAL(-5, s[0]);
    BOOST_CHECK_EQUAL(-2, s[1]);
    BOOST_CHECK_EQUAL(2, s[2]);
    BOOST_CHECK_EQUAL(5, s[3]);
    BOOST_CHECK_EQUAL(9, s[4]);
    h.clear();
    BOOST_CHECK(h.empty());
    BOOST_CHECK(h.empty());
    h.insert(2);
    BinaryHeap<int>::Element *eY = h.insert(2);
    h.insert(2);
    BinaryHeap<int>::Element *eX = h.insert(1);
    BOOST_CHECK(h.top()->data == 1);
    h.remove(eY);
    BOOST_CHECK(h.top()->data == 1);
    h.remove(eX);
    BOOST_CHECK(h.top()->data == 2);
    h.insert(-1);
    BOOST_CHECK(h.top()->data == -1);
}
