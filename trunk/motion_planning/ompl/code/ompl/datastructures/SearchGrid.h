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

#include <vector>
#include <iostream>
#include "ompl/datastructures/Hash.h"

namespace ompl
{

    class SearchGrid
    {
    public:

	/// definition of a coordinate within this grid
	typedef std::vector<int> Coord;

	class Mark
	{
	public:
	    Mark(void)
	    {
	    }
	    
	    ~Mark(void)
	    {
	    }
	    
	    void setMark(const Coord &cell, int value);
	    int  getMark(const Coord &cell, int def) const;
	    bool hasMark(const Coord &cell) const;
	    void clear(void);
	    
	private:
	    
	    /// equality operator for coordinate pointers
	    struct EqualCoord
	    {
		bool operator()(const Coord &c1, const Coord &c2) const
		{
		    return c1 == c2;
		}
	    };
	    
	    /// hash function for coordinates
	    struct HashFunCoord
	    {
		std::size_t operator()(const Coord &s) const
		{ 
		    unsigned long h = 0;
		    for (int i = s.size() - 1; i >= 0; --i)
		    {
			int high = h & 0xf8000000;
			h = h << 5;
			h = h ^ (high >> 27);
			h = h ^ s[i];
		    }		
		    return (std::size_t) h;
		}
	    };
	    
	    /// define the datatype for the used hash structure
	    typedef OMPL_NS_HASH::OMPL_NAME_HASH<Coord, int, HashFunCoord, EqualCoord> CoordHash;
	    
	    CoordHash m_hash;
	};
	
	SearchGrid(void)
	{	    
	}
	
	virtual ~SearchGrid(void)
	{
	}
	
	virtual void setAllCells(const double val) = 0;
	
	virtual void setCell(const Coord &cell, const double val) = 0;
	
	void setCellWithDecay(const Coord &cell, const double val,
			      const double decay, const unsigned int steps);
	
	virtual double getCell(const Coord &cell) const = 0;
	virtual void getNeighbors(const Coord &cell, std::vector<Coord> &neighbors) = 0;
	virtual void print(std::ostream &out = std::cout) const = 0;
	
    protected:
	

	void setCellWithDecayAux(Mark &seen, const Coord &cell, const double val,
				 const double decay, const unsigned int steps);
	
    };
    
    class SearchGrid2D : public SearchGrid
    {
    public:
	SearchGrid2D(const Coord &maxC);
	virtual ~SearchGrid2D(void);
	
	virtual void setAllCells(const double val);
	virtual void setCell(const Coord &cell, const double val);
	virtual double getCell(const Coord &cell) const;
	virtual void getNeighbors(const Coord &cell, std::vector<Coord> &neighbors);
	virtual void print(std::ostream &out = std::cout) const;

    private:
	
	std::vector<double> m_cells;
	Coord               m_maxC;
	
    };
    
}

#endif
