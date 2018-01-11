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

#define BOOST_TEST_MODULE "Grid"
#include <boost/test/unit_test.hpp>

#include "ompl/datastructures/Grid.h"
#include "ompl/datastructures/GridN.h"

using namespace ompl;

BOOST_AUTO_TEST_CASE(Grid_Simple)
{
    Grid<int> g(2);

    BOOST_CHECK_EQUAL((unsigned int)2, g.getDimension());

    Grid<int>::Coord coord(2);
    coord[0] = 1;
    coord[1] = 0;
    BOOST_CHECK_EQUAL(g.has(coord), false);
    Grid<int>::Cell *cell1 = g.createCell(coord);
    BOOST_CHECK(cell1 != nullptr);
    cell1->data = 1;
    g.add(cell1);
    BOOST_CHECK(g.has(coord));

    coord[1] = 1;
    BOOST_CHECK_EQUAL(g.has(coord), false);
    Grid<int>::Cell *cell2 = g.createCell(coord);
    BOOST_CHECK(cell2 != nullptr);
    cell2->data = 2;
    g.add(cell2);
    BOOST_CHECK(g.has(coord));

    Grid<int>::CellArray ca;
    g.neighbors(cell2, ca);
    BOOST_CHECK_EQUAL((unsigned int)1, ca.size());
    BOOST_CHECK_EQUAL(ca[0], cell1);

    coord[0] = 0;
    BOOST_CHECK_EQUAL(g.has(coord), false);
    ca.clear();
    g.neighbors(coord, ca);
    BOOST_CHECK_EQUAL((unsigned int)1, ca.size());
    BOOST_CHECK_EQUAL(ca[0], cell2);

    Grid<int>::Cell *cell3 = g.createCell(coord);
    BOOST_CHECK(cell3 != nullptr);
    cell3->data = 3;
    g.add(cell3);
    BOOST_CHECK(g.has(coord));

    BOOST_CHECK_EQUAL((unsigned int)3, g.size());
    int sum = 0;
    for (const auto & it : g)
        sum += it.second->data;
    BOOST_CHECK_EQUAL(6, sum);

    g.remove(cell2);
    g.destroyCell(cell2);

    BOOST_CHECK_EQUAL((unsigned int)2, g.size());
    sum = 0;
    for (const auto & it : g)
        sum += it.second->data;
    BOOST_CHECK_EQUAL(4, sum);
}

BOOST_AUTO_TEST_CASE(GridN_Simple)
{
    GridN<int> g(0);
    g.setDimension(2);

    BOOST_CHECK_EQUAL((unsigned int)2, g.getDimension());

    GridN<int>::Coord coord(2);
    coord[0] = 1;
    coord[1] = 0;
    BOOST_CHECK_EQUAL(g.has(coord), false);
    auto *cell1 = dynamic_cast<GridN<int>::Cell*>(g.createCell(coord));
    BOOST_CHECK(cell1 != nullptr);
    BOOST_CHECK(cell1->neighbors == 0);
    cell1->data = 1;
    g.add(cell1);
    BOOST_CHECK(g.has(coord));

    coord[1] = 1;
    BOOST_CHECK_EQUAL(g.has(coord), false);
    auto *cell2 = dynamic_cast<GridN<int>::Cell*>(g.createCell(coord));
    BOOST_CHECK(cell1->neighbors == 1);
    BOOST_CHECK(cell2->neighbors == 1);
    BOOST_CHECK(cell2 != nullptr);
    cell2->data = 2;
    g.add(cell2);
    BOOST_CHECK(g.has(coord));

    GridN<int>::CellArray ca;
    g.neighbors(cell2, ca);
    BOOST_CHECK_EQUAL((unsigned int)1, ca.size());
    BOOST_CHECK_EQUAL(ca[0], cell1);

    coord[0] = 0;
    BOOST_CHECK_EQUAL(g.has(coord), false);
    ca.clear();
    g.neighbors(coord, ca);
    BOOST_CHECK_EQUAL((unsigned int)1, ca.size());
    BOOST_CHECK_EQUAL(ca[0], cell2);

    auto *cell3 = dynamic_cast<GridN<int>::Cell*>(g.createCell(coord));
    BOOST_CHECK(cell3 != nullptr);
    BOOST_CHECK(cell1->neighbors == 1);
    BOOST_CHECK(cell2->neighbors == 2);
    BOOST_CHECK(cell3->neighbors == 1);
    cell3->data = 3;
    g.add(cell3);
    BOOST_CHECK(g.has(coord));

    BOOST_CHECK_EQUAL((unsigned int)3, g.size());
    int sum = 0;
    for (const auto & it : g)
        sum += it.second->data;
    BOOST_CHECK_EQUAL(6, sum);

    coord[0] = 2;
    auto *cell4 = dynamic_cast<GridN<int>::Cell*>(g.createCell(coord));
    BOOST_CHECK(cell4 != nullptr);
    cell4->data = 4;
    g.add(cell4);
    BOOST_CHECK(cell2->neighbors == 3);
    BOOST_CHECK(cell2->border);

    coord[0] = 1;
    coord[1] = 2;
    auto *cell5 = dynamic_cast<GridN<int>::Cell*>(g.createCell(coord));
    BOOST_CHECK(cell5 != nullptr);
    cell5->data = 5;
    g.add(cell5);
    BOOST_CHECK(cell2->neighbors == 4);
    BOOST_CHECK_EQUAL(cell2->border, false);
    BOOST_CHECK(cell1->border);
    BOOST_CHECK(cell3->border);
    BOOST_CHECK(cell4->border);
    BOOST_CHECK(cell5->border);

    BOOST_CHECK(g.getCell(cell5->coord) == cell5);

    g.remove(cell1);
    g.destroyCell(cell1);
    BOOST_CHECK(cell2->border);

    BOOST_CHECK_EQUAL((unsigned int)4, g.size());
    sum = 0;
    for (const auto & it : g)
        sum += it.second->data;
    BOOST_CHECK_EQUAL(14, sum);

    BOOST_CHECK_EQUAL((unsigned int)1, g.components().size());

    coord[0] = 10;
    coord[1] = 2;
    auto *cell6 = dynamic_cast<GridN<int>::Cell*>(g.createCell(coord));
    g.add(cell6);
    BOOST_CHECK_EQUAL((unsigned int)2, g.components().size());
    BOOST_CHECK_EQUAL(g.components()[0].size() + g.components()[1].size(), g.size());
}
