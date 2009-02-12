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

// there should be no inclusion guards for this file
#ifndef OMPL_GRID_INCLUDE
#  error "This file should not be included directly"
#endif

#include <vector>
#include <iostream>
#include <cassert>
#include "ompl/datastructures/Hash.h"

namespace ompl
{

#ifndef OMPL_DATASTRUCTURES_GRID_AUX_
    const unsigned short MAX_GRID_NEIGHBORS = 255;
#endif

    template <typename _T>
    class
#ifdef OMPL_GRID_COUNT_NEIGHBORS
    GridN
#else
    Grid
#endif
    {
    public:
	
	typedef std::vector<int> Coord;
	
	struct Cell
	{
	    _T                  data;
	    Coord               coord;
	    
#ifdef OMPL_GRID_COUNT_NEIGHBORS
	    unsigned short      neighbors;
	    bool                border;
#endif
	    Cell(void)
#ifdef OMPL_GRID_COUNT_NEIGHBORS
		: neighbors(0), border(true)
#endif
	    {
	    }
	    
	    virtual ~Cell(void)
	    {
	    }
	};
	
	typedef std::vector<Cell*> CellArray;	
	
	explicit 
#ifdef OMPL_GRID_COUNT_NEIGHBORS
	    GridN
#else
	    Grid
#endif
	    (unsigned int dimension)
	{
#ifdef OMPL_GRID_COUNT_NEIGHBORS
	    m_hasBounds = false;
	    m_overrideCellNeighborsLimit = false;
#endif
	    setDimension(dimension);
	}

	virtual 
#ifdef OMPL_GRID_COUNT_NEIGHBORS
	    ~GridN
#else
	    ~Grid
#endif
	    (void)
	{
	    freeMemory();
	}
	
	virtual void clear(void)
	{
	    freeMemory();
	}
	
	unsigned int getDimension(void) const
	{
	    return m_dimension;
	}
	
	void setDimension(unsigned int dimension)
	{
	    m_dimension = dimension;
	    m_maxNeighbors = 2 * m_dimension;
	    assert(m_maxNeighbors < MAX_GRID_NEIGHBORS);

#ifdef OMPL_GRID_COUNT_NEIGHBORS
	    if (!m_overrideCellNeighborsLimit)
		m_interiorCellNeighborsLimit = m_maxNeighbors;
#endif
	}

#ifdef OMPL_GRID_COUNT_NEIGHBORS
	void setBounds(const Coord &low, const Coord &up)
	{
	    m_lowBound  = low;
	    m_upBound   = up;
	    m_hasBounds = true;
	}

	void setInteriorCellNeighborLimit(unsigned int count)
	{
	    m_interiorCellNeighborsLimit = count;
	    assert(m_interiorCellNeighborsLimit > 0);
	    m_overrideCellNeighborsLimit = true;
	}
#endif

	bool has(const Coord &coord) const
	{
	    return getCell(coord) != NULL;
	}
	
	Cell* getCell(const Coord &coord) const
	{ 
	    iterator pos = m_hash.find(const_cast<Coord*>(&coord));
	    Cell *c = (pos != m_hash.end()) ? pos->second : NULL;
	    return c;
	}	
	
	void    neighbors(const Cell* cell, CellArray& list) const
	{
	    Coord test = cell->coord;
	    neighbors(test, list);
	}
	
	void    neighbors(const Coord& coord, CellArray& list) const
	{
	    Coord test = coord;
	    neighbors(test, list);
	}
	
	void    neighbors(Coord& coord, CellArray& list) const
	{
	    list.reserve(list.size() + m_maxNeighbors);
	    
	    for (int i = m_dimension - 1 ; i >= 0 ; --i)
	    {
		coord[i]--;
		
		iterator pos = m_hash.find(&coord);
		Cell *cell = (pos != m_hash.end()) ? pos->second : NULL;

		if (cell)
		    list.push_back(cell);
		coord[i] += 2;

		pos = m_hash.find(&coord);
		cell = (pos != m_hash.end()) ? pos->second : NULL;
		
		if (cell)
		    list.push_back(cell);
		coord[i]--;
	    }
	}
	
	virtual Cell* createCell(const Coord& coord, CellArray *nbh = NULL)
	{
	    Cell *cell = new Cell();
	    cell->coord = coord;
	    
#ifdef OMPL_GRID_COUNT_NEIGHBORS
	    CellArray *list = nbh ? nbh : new CellArray();
	    neighbors(cell->coord, *list);
	    
	    for (typename CellArray::iterator cl = list->begin() ; cl != list->end() ; ++cl)
	    {
		Cell* c = *cl;
		c->neighbors++;
		if (c->border && c->neighbors >= m_interiorCellNeighborsLimit)
		    c->border = false;
	    }
	    
	    cell->neighbors = numberOfBoundaryDimensions(cell->coord) + list->size();
	    if (cell->border && cell->neighbors >= m_interiorCellNeighborsLimit)
		cell->border = false;
	    
	    if (!nbh)
		delete list;
#else
	    if (nbh)
		neighbors(cell->coord, *nbh);
#endif	    
	    
	    return cell;
	}

	virtual bool remove(Cell *cell)
	{
	    if (cell)
	    {
#ifdef OMPL_GRID_COUNT_NEIGHBORS
		CellArray *list = new CellArray();
		neighbors(cell->coord, *list);
		for (typename CellArray::iterator cl = list->begin() ; cl != list->end() ; ++cl)
		{
		    Cell* c = *cl;
		    c->neighbors--;
		    if (!c->border && c->neighbors < m_interiorCellNeighborsLimit)
			c->border = true;
		}	  
		delete list;
#endif
		iterator pos = m_hash.find(&cell->coord);
		if (pos != m_hash.end())
		{
		    m_hash.erase(pos);
		    return true;
		}
	    }
	    return false;
	}
	
	virtual void add(Cell *cell)
	{
	    m_hash.insert(std::make_pair(&cell->coord, cell));
	}
	
	virtual void destroyCell(Cell *cell) const
	{
	    delete cell;
	}
	
	void getContent(std::vector<_T> &content) const
	{
	    for (iterator i = m_hash.begin() ; i != m_hash.end() ; i++)
		content.push_back(i->second->data);
	}
	
	void getCoordinates(std::vector<Coord*> &coords) const
	{
	    for (iterator i = m_hash.begin() ; i != m_hash.end() ; i++)
		coords.push_back(i->first);
	}
	
	void getCells(CellArray &cells) const
	{
	    for (iterator i = m_hash.begin() ; i != m_hash.end() ; i++)
		cells.push_back(i->second);
	}
	
	void printCoord(Coord& coord, std::ostream &out = std::cout) const
	{
	    out << "[ ";
	    for (unsigned int i = 0 ; i < m_dimension ; ++i)
		out << coord[i] << " ";
	    out << "]" << std::endl;
	}

	bool empty(void) const
	{
	    return m_hash.empty();
	}
	
	unsigned int size(void) const
	{
	    return m_hash.size();	    
	}

    protected:

#ifdef OMPL_GRID_COUNT_NEIGHBORS
	unsigned int numberOfBoundaryDimensions(const Coord &coord) const
	{
	    unsigned int result = 0;
	    if (m_hasBounds)
	    {
		for (unsigned int i = 0 ; i < m_dimension ; ++i)
		    if (coord[i] == m_lowBound[i] || coord[i] == m_upBound[i])
			result++;
	    }
	    return result;
	}	
#endif

	void freeMemory(void)
	{
	    CellArray content;
	    getCells(content);
	    m_hash.clear();
	    
	    for (unsigned int i = 0 ; i < content.size() ; i++)
		delete content[i];	    
	}

	struct EqualCoordPtr
	{
	    bool operator()(const Coord* const c1, const Coord* const c2) const
	    {
		return *c1 == *c2;
	    }
	};
	
	struct HashFunCoordPtr
	{
	    std::size_t operator()(const Coord* const s) const
	    { 
		unsigned long h = 0;
		for (int i = s->size() - 1; i >= 0; --i)
		{
		    int high = h & 0xf8000000;
		    h = h << 5;
		    h = h ^ (high >> 27);
		    h = h ^ s->at(i);
		}		
		return (std::size_t) h;
	    }
	};

	typedef OMPL_NS_HASH::OMPL_NAME_HASH<Coord*, Cell*, HashFunCoordPtr, EqualCoordPtr> CoordHash;
	
    public:
	
	typedef typename CoordHash::const_iterator iterator;
	
	iterator begin(void) const
	{
	    return m_hash.begin();
	}
	
	iterator end(void) const
	{
	    return m_hash.end();
	}
	
    protected:

	unsigned int     m_dimension;
	unsigned int     m_maxNeighbors;
	
#ifdef OMPL_GRID_COUNT_NEIGHBORS
	bool             m_hasBounds;
	Coord            m_lowBound;
	Coord            m_upBound;
	unsigned int     m_interiorCellNeighborsLimit;
	bool             m_overrideCellNeighborsLimit;
#endif
	
	CoordHash        m_hash;
    };
}

#ifndef OMPL_DATASTRUCTURES_GRID_AUX_
#  define OMPL_DATASTRUCTURES_GRID_AUX_
#endif
