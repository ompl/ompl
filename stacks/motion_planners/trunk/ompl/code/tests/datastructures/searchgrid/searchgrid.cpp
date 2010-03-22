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

#include "ompl/datastructures/SearchGrid.h"

using namespace ompl;

TEST(SearchGrid, Simple)
{
    SearchGrid::Coord M(2);
    M[0] = 10;
    M[1] = 10;
    SearchGrid *grid = new SearchGrid2D(M);

    grid->setAllCells(0.1);


    SearchGrid::Coord c(2);
    c[0] = 1;
    c[1] = 1;
    
    EXPECT_EQ(grid->getCell(c), 0.1);
    c[1]++;
    grid->setCellWithDecay(c, 2.0, 0.25, 2);
    c[1]--;
    EXPECT_NEAR(grid->getCell(c), 0.5, 1e-9);
    
    SearchGrid::Coord s(2);
    SearchGrid::Coord g(2);
    s[0] = s[1] = 0;
    g[0] = g[1] = 7;
    std::vector<SearchGrid::Coord> p;
    grid->shortestPath(s, g, p);
    
    for (unsigned int i = 0 ; i < p.size() ; ++i)
	grid->setCell(p[i], 9);
    
    //    grid->print();
    
    delete grid;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
