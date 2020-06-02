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

#ifndef OMPL_TEST_ENVIRONMENT_2D_
#define OMPL_TEST_ENVIRONMENT_2D_

#include <fstream>
#include <vector>
#include <string>

static const int T_FREE     = 0;
static const int T_OBSTACLE = 1;
static const int T_PATH     = 2;
static const int T_ERROR    = 3;

/** \brief Representation of a 2D environment */
struct Environment2D
{
    std::pair<int, int>             start;
    std::pair<int, int>             goal;
    int                             width{0};
    int                             height{0};
    std::vector< std::vector<int> > grid;
};

/** Load a grid representing the environment */
static void loadEnvironment(const char *filename, Environment2D &env)
{
    std::fstream in(filename);
    while (in.good() && !in.eof())
    {
        std::string name;
        in >> name;
        if (name == "discretization(cells):")
        {
            in >> env.width >> env.height;
            env.grid.resize(env.width);
            for (unsigned int i = 0 ; i < env.grid.size() ; ++i)
                env.grid[i].resize(env.height, 0);
        }
        else
            if (name == "start(cells):")
                in >> env.start.first >> env.start.second;
            else
                if (name == "end(cells):")
                    in >> env.goal.first >> env.goal.second;
                else
                    if (name == "environment:")
                    {
                        for (unsigned int i = 0 ; i < env.grid.size() ; ++i)
                            for (unsigned int j = 0 ; j < env.grid[i].size() ; ++j)
                                in >> env.grid[i][j];
                    }
    }
    in.close();
}

static void printEnvironment(std::ostream &out, const Environment2D &env)
{

    for (unsigned int i = 0 ; i < env.grid.size() ; ++i)
    {
        for (unsigned int j = 0 ; j < env.grid[i].size() ; ++j)
        {
            switch (env.grid[i][j])
            {
            case T_PATH:
                out << "* ";
                break;
            case T_ERROR:
                out << "X ";
                break;
            case T_FREE:
            case T_OBSTACLE:
            default:
                out << env.grid[i][j] << " ";
                break;
            }
        }
        out << std::endl;
    }
}

#endif
