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

/** \author Ioan Sucan */

#include <gtest/gtest.h>
#include "ompl/datastructures/GridB.h"

using namespace ompl;

TEST(GridB, Simple)
{
    GridB<int> g(2); 
    
    EXPECT_EQ((unsigned int)2, g.getDimension());
    
    GridB<int>::Coord coord(2);
    coord[0] = 1;
    coord[1] = 0;
    EXPECT_FALSE(g.has(coord));
    GridB<int>::Cell *cell1 = g.createCell(coord);
    EXPECT_FALSE(cell1 == NULL);
    EXPECT_TRUE(cell1->neighbors == 0);
    cell1->data = 1;
    g.add(cell1);
    EXPECT_TRUE(g.has(coord));
    EXPECT_NEAR(g.fracExternal(), 1.0, 1e-12);
    

    coord[1] = 1;
    EXPECT_FALSE(g.has(coord));
    GridB<int>::Cell *cell2 = g.createCell(coord);
    EXPECT_TRUE(cell1->neighbors == 1);
    EXPECT_TRUE(cell2->neighbors == 1);
    EXPECT_FALSE(cell2 == NULL);    
    cell2->data = 2;
    g.add(cell2);
    EXPECT_TRUE(g.has(coord));
    EXPECT_NEAR(g.fracExternal(), 1.0, 1e-12);

    GridB<int>::CellArray ca;
    g.neighbors(cell2, ca);
    EXPECT_EQ((unsigned int)1, ca.size());
    EXPECT_EQ(ca[0], cell1);
    
    coord[0] = 0;
    EXPECT_FALSE(g.has(coord));
    ca.clear();
    g.neighbors(coord, ca);
    EXPECT_EQ((unsigned int)1, ca.size());
    EXPECT_EQ(ca[0], cell2);
    
    GridB<int>::Cell *cell3 = g.createCell(coord);
    EXPECT_FALSE(cell3 == NULL);
    EXPECT_TRUE(cell1->neighbors == 1);
    EXPECT_TRUE(cell2->neighbors == 2);
    EXPECT_TRUE(cell3->neighbors == 1);
    cell3->data = 3;
    g.add(cell3);
    EXPECT_TRUE(g.has(coord));
    EXPECT_NEAR(g.fracExternal(), 1.0, 1e-12);
    
    EXPECT_EQ((unsigned int)3, g.size());    
    int sum = 0;
    for (GridB<int>::iterator it = g.begin() ; it != g.end() ; ++it)
	sum += it->second->data;
    EXPECT_EQ(6, sum);

    coord[0] = 2;
    GridB<int>::Cell *cell4 = g.createCell(coord);
    EXPECT_FALSE(cell4 == NULL);
    cell4->data = 4;
    g.add(cell4);
    EXPECT_TRUE(cell2->neighbors == 3);
    EXPECT_TRUE(cell2->border);
    EXPECT_NEAR(g.fracExternal(), 1.0, 1e-12);
    
    coord[0] = 1;
    coord[1] = 2;
    GridB<int>::Cell *cell5 = g.createCell(coord);
    EXPECT_FALSE(cell5 == NULL);
    cell5->data = 5;
    g.add(cell5);
    EXPECT_TRUE(cell2->neighbors == 4);
    EXPECT_FALSE(cell2->border);
    EXPECT_TRUE(cell1->border);
    EXPECT_TRUE(cell3->border);
    EXPECT_TRUE(cell4->border);
    EXPECT_TRUE(cell5->border);
    EXPECT_EQ((unsigned int)1, g.countInternal());
    
    EXPECT_EQ(1, g.topExternal()->data);
    EXPECT_EQ(2, g.topInternal()->data);
    
    g.remove(cell1);
    g.destroyCell(cell1);
    EXPECT_TRUE(cell2->border);

    EXPECT_EQ((unsigned int)0, g.countInternal());
    EXPECT_EQ(2, g.topExternal()->data);

    EXPECT_EQ((unsigned int)4, g.size());    
    sum = 0;
    for (GridB<int>::iterator it = g.begin() ; it != g.end() ; ++it)
	sum += it->second->data;
    EXPECT_EQ(14, sum);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
