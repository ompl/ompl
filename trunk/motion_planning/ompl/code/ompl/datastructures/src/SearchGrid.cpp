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
#include <cassert>

void ompl::SearchGrid::Mark::setMark(const Coord &cell, int value)
{
    m_hash.insert(std::make_pair(cell, value));
}

bool ompl::SearchGrid::Mark::hasMark(const Coord &cell) const
{
    return m_hash.find(cell) != m_hash.end();
}

int ompl::SearchGrid::Mark::getMark(const Coord &cell, int def) const
{
    CoordHash::const_iterator it = m_hash.find(cell);
    return it != m_hash.end() ? it->second : def;
}

void ompl::SearchGrid::Mark::clear(void)
{
    m_hash.clear();
}

void ompl::SearchGrid::setCellWithDecay(const Coord &cell, const double val,
					const double decay, const unsigned int steps)
{
    setCell(cell, val);
    if (steps > 0)
    {
	Mark seen;
	seen.setMark(cell, 1);
	
	std::vector<Coord> nbh;
	getNeighbors(cell, nbh);
	
	double v = val * decay;
	for (unsigned int i = 0 ; i < nbh.size() ; ++i)
	    setCellWithDecayAux(seen, nbh[i], v, decay, steps - 1);
    }
}

void ompl::SearchGrid::setCellWithDecayAux(Mark &seen, const Coord &cell, const double val,
					   const double decay, const unsigned int steps)
{
    setCell(cell, val);
    seen.setMark(cell, 1);
    
    if (steps > 0)
    {
	std::vector<Coord> nbh;
	getNeighbors(cell, nbh);
	
	double v = val * decay;
	for (unsigned int i = 0 ; i < nbh.size() ; ++i)
	    if (!seen.hasMark(nbh[i]))
		setCellWithDecayAux(seen, nbh[i], v, decay, steps - 1);
    }
}

ompl::SearchGrid2D::SearchGrid2D(const Coord &maxC) : SearchGrid()
{
    assert(maxC.size() == 2);
    m_cells.resize(maxC[0] * maxC[1]);
    assert(m_cells.size() > 0);
    m_maxC = maxC;
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

double ompl::SearchGrid2D::getCell(const Coord &cell) const
{
    return m_cells[cell[0] * m_maxC[1] + cell[1]];
}

void ompl::SearchGrid2D::getNeighbors(const Coord &cell, std::vector<Coord> &neighbors)
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

void ompl::SearchGrid2D::print(std::ostream &out) const
{
    out << "G = [" << std::endl;
    for (int i = 0 ; i < m_maxC[0] ; ++i)
    {
	for (int j = 0 ; j < m_maxC[1] ; ++j)
	    out << m_cells[i * m_maxC[1] + j] << " ";
	if (i + 1 < m_maxC[0])
	    out << "," << std::endl;
    }
    out << "];" << std::endl;    
}
