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

/** \Author Ioan Sucan */

#ifndef OMPL_DATASTRUCTURES_GRID_
#define OMPL_DATASTRUCTURES_GRID_

#include <vector>
#include <ext/hash_map>

namespace ompl
{
    
    template <typename _T>
    class Grid
    {
    public:
	
	typedef std::vector<int> Coord;
	typedef Coord*           Coord_t;
	
	ForwardStructDeclaration(Cell);
	
	struct Cell
	{
	    _T                  data;
	    Coord               coord;
	    
	    Cell(void)
	    {
		pthread_rwlock_init(&lock, NULL);
	    }
	    
	    virtual ~Cell(void)
	    {
		pthread_rwlock_destroy(&lock);
	    }
	    
	    inline
	    void rd_lock(void)
	    {
		pthread_rwlock_rdlock(&lock);
	    }

	    inline
	    void wr_lock(void)
	    {
		pthread_rwlock_wrlock(&lock);
	    }
	    
	    inline
	    void unlock(void)
	    {
		pthread_rwlock_unlock(&lock);
	    }

	protected:
	    
	    pthread_rwlock_t lock;
	    
	};
	
	typedef std::vector<Cell_t> CellArray;	
	
	explicit Grid(unsigned int dimension = 0)
	{
	    setDimension(dimension);
	}

	virtual ~Grid(void)
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
	}

	bool has(const Coord &coord) const
	{
	    return getCell(coord) != NULL;
	}
	
	Cell_t getCell(const Coord &coord) const
	{ 
	    pthread_rwlock_rdlock(&m_hashLock);
	    hashIterator pos = m_hash.find(const_cast<const Coord_t>(&coord));
	    Cell_t c = (pos != m_hash.end()) ? pos->second : NULL;
	    pthread_rwlock_unlock(&m_hashLock);
	    return c;
	}	
	
	_T&     get(const Coord& coord) const
	{
	    return getCell(coord)->data;
	}
	
	_T&     get(const Cell_t cell) const
	{
	    return cell->data;
	}
	
	void    neighbors(const Cell_t cell, CellArray& list) const
	{
	    Coord test = cell->coord;
	    neighbors(test, list);
	}
	
	void    neighbors(Coord& coord, CellArray& list) const
	{
	    list.reserve(list.size() + m_maxNeighbors);
	    
	    pthread_rwlock_rdlock(&m_hashLock);

	    for (int i = m_dimension - 1 ; i >= 0 ; --i)
	    {
		coord[i]--;
		
		hashIterator pos = m_hash.find(&coord);
		Cell_t cell = (pos != m_hash.end()) ? pos->second : NULL;

		if (cell)
		    list.push_back(cell);
		coord[i]+=2;

		pos = m_hash.find(&coord);
		cell = (pos != m_hash.end()) ? pos->second : NULL;
		
		if (cell)
		    list.push_back(cell);
		coord[i]--;
	    }
	    pthread_rwlock_unlock(&m_hashLock);
	}
	
	virtual Cell_t add(const Coord& coord, CellArray *nbh = NULL)
	{
	    Cell_t cell = new Cell();
	    cell->coord = coord;
	    
	    if (nbh)
		neighbors(cell->coord, *nbh);
	    
	    return cell;
	}
	
	virtual void finish(Cell_t cell)
	{
	    pthread_rwlock_wrlock(&m_hashLock);
	    m_hash.insert(std::make_pair(&cell->coord, cell));
	    pthread_rwlock_unlock(&m_hashLock);
	}
	
	void getContent(std::vector<_T> &content) const
	{
	    pthread_rwlock_rdlock(&m_hashLock);
	    for (hashIterator i = m_hash.begin() ; i != m_hash.end() ; i++)
		content.push_back(i->second->data);
	    pthread_rwlock_unlock(&m_hashLock);
	}
	
	void getCoordinates(std::vector<Coord_t> &coords) const
	{
	    pthread_rwlock_rdlock(&m_hashLock);
	    for (hashIterator i = m_hash.begin() ; i != m_hash.end() ; i++)
		coords.push_back(i->first);
	    pthread_rwlock_unlock(&m_hashLock);
	}
	
	void getCells(CellArray &cells) const
	{
	    pthread_rwlock_rdlock(&m_hashLock);
	    for (hashIterator i = m_hash.begin() ; i != m_hash.end() ; i++)
		cells.push_back(i->second);
	    pthread_rwlock_unlock(&m_hashLock);
	}
	
	void printCoord(Coord& coord, FILE* out = stdout) const
	{
	    fprintf(out, "[ ");
	    for (unsigned int i = 0 ; i < m_dimension ; ++i)
		fprintf(out, "%d ", coord[i]);
	    fprintf(out, "]\n");
	}
	
	unsigned int size(void) const
	{
	    pthread_rwlock_rdlock(&m_hashLock);
	    const unsigned int sz = m_hash.size();	    
	    pthread_rwlock_unlock(&m_hashLock);
	    return sz;
	}

    protected:


	void freeMemory(void)
	{
	    CellArray content;
	    getCells(content);
	    
	    pthread_rwlock_wrlock(&m_hashLock);
	    m_hash.clear();
	    pthread_rwlock_unlock(&m_hashLock);
	    
	    for (unsigned int i = 0 ; i < content.size() ; i++)
		delete content[i];	    
	}

	struct EqualCoordPtr
	{
	    bool operator()(const Coord* const& c1, const Coord* const& c2) const
	    {
		return *c1 == *c2;
	    }
	};
	
	struct HashFunCoordPtr
	{
	    std::size_t operator()(Coord_t s) const
	    { 
		unsigned long h = 0;
		for (int i = s->size() - 1; i >= 0; --i)
		{
		    int high = h & 0xf8000000;
		    h = h << 5;
		    h = h ^ (high >> 27);
		    h = h ^ (*s)[i];
		}		
		return (std::size_t) h;
	    }
	};

	typedef __gnu_cxx::hash_map<Coord_t, Cell_t, HashFunCoordPtr, EqualCoordPtr> CoordHash;
	typedef typename CoordHash::const_iterator                                   hashIterator;
	
	unsigned int     m_dimension;
	unsigned int     m_maxNeighbors;

	CoordHash        m_hash;
	mutable
	pthread_rwlock_t m_hashLock;
    };
}

#endif
