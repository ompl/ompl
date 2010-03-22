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

/** \author Ioan Sucan  */

#include "ompl/datastructures/SearchGrid.h"
#include <ros/console.h>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <sstream>

ompl::SearchGrid::SearchGrid(const Coord &maxC) : m_maxC(maxC)
{
    m_stateCount = 1;
    m_maxCAux.resize(m_maxC.size());
    for (int i = m_maxC.size() - 1 ; i >= 0 ; --i)
    {
	if (m_maxC[i] <= 0)
	    ROS_ERROR("Number of cells in each dimension of search grid must be positive");
	m_maxCAux[i] = m_stateCount;
	m_stateCount *= m_maxC[i];
    }
}

ompl::SearchGrid::~SearchGrid(void)
{
}

void ompl::SearchGrid::stateToCell(const int state, Coord &cell) const
{
    cell.resize(m_maxC.size());
    int st = state;
    for (unsigned int i = 0 ; i < cell.size() ; ++i)
    {
	cell[i] = st / m_maxCAux[i];
	st = st % m_maxCAux[i];
    }    
}

void ompl::SearchGrid::cellToState(const Coord &cell, int &state) const
{
    state = 0;
    for (unsigned int i = 0 ; i < cell.size() ; ++i)
	state += cell[i] * m_maxCAux[i];
}

void ompl::SearchGrid::setCellWithDecay(const Coord &cell, const double val,
					const double decay, const unsigned int steps)
{
    int state;
    cellToState(cell, state);
    setCellWithDecay(state, val, decay, steps);
}

void ompl::SearchGrid::setCellWithDecay(const int state, const double val,
					const double decay, const unsigned int steps)
{
    setState(state, val);
    if (steps > 0)
    {
	std::map<int, int> seen;
	seen[state] = 1;
	
	std::vector<int> nbh;
	getNeighbors(state, nbh);
	
	double v = val * decay;
	for (unsigned int i = 0 ; i < nbh.size() ; ++i)
	    setCellWithDecayAux(seen, nbh[i], v, decay, steps - 1);
    }
}

void ompl::SearchGrid::setCellWithDecayAux(std::map<int, int> &seen, const int state, const double val,
					   const double decay, const unsigned int steps)
{
    setState(state, val);
    seen[state] = 1;
    
    if (steps > 0)
    {
	std::vector<int> nbh;
	getNeighbors(state, nbh);
	
	double v = val * decay;
	for (unsigned int i = 0 ; i < nbh.size() ; ++i)
	    if (seen.find(nbh[i]) == seen.end())
		setCellWithDecayAux(seen, nbh[i], v, decay, steps - 1);
    }
}

ompl::SearchGrid2D::SearchGrid2D(const Coord &maxC) : SearchGrid(maxC)
{
    assert(maxC.size() == 2);
    m_cells.resize(m_stateCount);
}

ompl::SearchGrid2D::~SearchGrid2D(void)
{
}

void ompl::SearchGrid2D::setAllCells(const double val)
{
    std::fill(m_cells.begin(), m_cells.end(), val);
}

void ompl::SearchGrid2D::setCell(const Coord &cell, const double val)
{
    m_cells[cell[0] * m_maxC[1] + cell[1]] = val;
}	

void ompl::SearchGrid2D::setState(const int state, const double val)
{
    m_cells[state] = val;
}	

double ompl::SearchGrid2D::getCell(const Coord &cell) const
{
    return m_cells[cell[0] * m_maxC[1] + cell[1]];
}

double ompl::SearchGrid2D::getState(const int state) const
{
    return m_cells[state];
}

void ompl::SearchGrid2D::getNeighbors(const int state, std::vector<int> &neighbors) const
{ 
    Coord nbh;
    stateToCell(state, nbh);

    if (nbh[0] > 0)
	neighbors.push_back(state - m_maxC[1]);

    if (nbh[1] > 0)
	neighbors.push_back(state - 1);

    if (nbh[0] + 1 < m_maxC[0])
	neighbors.push_back(state + m_maxC[1]);
    
    if (nbh[1] + 1 < m_maxC[1])
	neighbors.push_back(state + 1);
}

void ompl::SearchGrid2D::getNeighbors(const Coord &cell, std::vector<Coord> &neighbors) const
{
    Coord nbh = cell;
    
    if (nbh[0] > 0)
    {
	nbh[0]--;
	neighbors.push_back(nbh);
	nbh[0]++;
    }

    if (nbh[1] > 0)
    {
	nbh[1]--;
	neighbors.push_back(nbh);
	nbh[1]++;
    }

    nbh[0]++;
    if (nbh[0] < m_maxC[0])
	neighbors.push_back(nbh);
    nbh[0]--;

    nbh[1]++;
    if (nbh[1] < m_maxC[1])
	neighbors.push_back(nbh);  
}

void ompl::SearchGrid2D::print(const std::string &name, std::ostream &out) const
{
    out << name << " = [" << std::endl;
    for (unsigned int i = 0 ; i < m_maxC[0] ; ++i)
    {
	for (unsigned int j = 0 ; j < m_maxC[1] ; ++j)
	    out << m_cells[i * m_maxC[1] + j] << " ";
	if (i + 1 < m_maxC[0])
	    out << "," << std::endl;
    }
    out << "];" << std::endl;    
}

bool ompl::SearchGrid::shortestPath(const Coord &start, const Coord &goal, std::vector<Coord> &path) const
{
    int s, g;
    cellToState(start, s);
    cellToState(goal, g);
    
    std::vector<double> dist(m_stateCount, INFINITY);
    std::vector<int>    prev(m_stateCount, -1);
    std::vector<int>    seen(m_stateCount, 0);

    dist[s] = 0.0;
    std::vector<int> nbh;
    
    while (true)
    {	
	int u = -1;
	double minDist = INFINITY;    
	for (unsigned int i = 0 ; i < m_stateCount ; ++i)
	{
	    if (seen[i])
		continue;
	    if (minDist > dist[i])
	    {
		minDist = dist[i];
		u = i;
	    }
	}
	
	seen[u] = 1;
	
	if (u < 0 || u == g)
	    break;
	
	nbh.clear();
	getNeighbors(u, nbh);
	
	for (unsigned int i = 0 ; i < nbh.size() ; ++i)
	{
	    double alt = dist[u] + getState(nbh[i]);
	    if (alt < dist[nbh[i]])
	    {
		dist[nbh[i]] = alt;
		prev[nbh[i]] = u;
	    }	    
	}
    }
    
    if (!seen[g])
	return false;
    
    std::vector<int> p;
    while (g != s)
    {
	p.push_back(g);
	g = prev[g];
    }
    p.push_back(g);

    path.resize(p.size());
    for (int i = p.size() - 1 ; i >= 0 ; --i)
	stateToCell(p[i], path[p.size() - i - 1]);
    
    return true;
}

unsigned int ompl::SearchGrid::manhattanDistance(const Coord &a, const Coord &b) const
{
    unsigned int d = 0;
    for (unsigned int i = 0 ; i < a.size() ; ++i)
	d += abs(a[i] - b[i]);
    return d;
}

std::string ompl::SearchGrid::toString(const Coord &c) const
{
    std::stringstream ss;
    for (unsigned int i = 0 ; i < c.size() ; ++i)
    {
	ss << c[i];
	if (i + 1 < c.size())
	    ss << " ";
    }
    return ss.str();
}
