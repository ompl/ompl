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

/* \author Ioan Sucan */

#ifndef OMPL_DATASTRUCTURES_SEARCH_GRID_
#define OMPL_DATASTRUCTURES_SEARCH_GRID_

#include "ompl/msg/Output.h"
#include <vector>
#include <iostream>
#include <map>

namespace ompl
{

    class SearchGrid
    {
    public:

	/// definition of a coordinate within this grid
	typedef std::vector<unsigned int> Coord;

        SearchGrid(const Coord &maxC);	
	virtual ~SearchGrid(void);
	
	const Coord& getBounds(void) const
	{
	    return m_maxC;
	}
	
	unsigned int getStateCount(void) const
	{
	    return m_stateCount;	    
	}
	
	void stateToCell(const int state, Coord &cell) const;
	void cellToState(const Coord &cell, int &state) const;
	std::string toString(const Coord &c) const;
	
	unsigned int manhattanDistance(const Coord &a, const Coord &b) const;
	
	virtual void setAllCells(const double val) = 0;
	
	virtual void setCell(const Coord &cell, const double val) = 0;
	virtual void setState(const int state, const double val) = 0;
	void setCellWithDecay(const Coord &cell, const double val,
			      const double decay, const unsigned int steps);
	void setCellWithDecay(const int state, const double val,
			      const double decay, const unsigned int steps);
	
	virtual double getCell(const Coord &cell) const = 0;
	virtual double getState(const int state) const = 0;
	
	virtual void getNeighbors(const Coord &cell, std::vector<Coord> &neighbors) const = 0;
	virtual void getNeighbors(const int state, std::vector<int> &neighbors) const = 0;
	
	virtual void print(const std::string &name = "G", std::ostream &out = std::cout) const = 0;
	
	bool shortestPath(const Coord &start, const Coord &goal, std::vector<Coord> &path) const;
	
    protected:
	
	void setCellWithDecayAux(std::map<int, int> &seen, const int state, const double val,
				 const double decay, const unsigned int steps);
	
	unsigned int   m_stateCount;
	Coord          m_maxC;
	Coord          m_maxCAux;

	msg::Interface m_msg;
    };
    
    class SearchGrid2D : public SearchGrid
    {
    public:
	SearchGrid2D(const Coord &maxC);
	virtual ~SearchGrid2D(void);
	
	virtual void setAllCells(const double val);
	virtual void setCell(const Coord &cell, const double val);
	virtual void setState(int state, const double val);
	virtual double getCell(const Coord &cell) const;
	virtual double getState(int state) const;
	virtual void getNeighbors(const Coord &cell, std::vector<Coord> &neighbors) const;
	virtual void getNeighbors(int state, std::vector<int> &neighbors) const;
	virtual void print(const std::string &name = "G", std::ostream &out = std::cout) const;

    private:
	
	std::vector<double> m_cells;
	
    };
    
}

#endif
