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

#ifndef OMPL_DATASTRUCTURES_BINARY_HEAP_
#define OMPL_DATASTRUCTURES_BINARY_HEAP_

#include <functional>
#include <vector>
#include <cassert>

namespace ompl
{
    
    /** This class provides an implementation of an updatable
	min-heap. Using it is a bit cumbersome, as it requires keeping
	track of the Element* type, however, it should be as fast as
	it gets with an updatable heap. */
    template <typename _T, 
	      class LessThan = std::less<_T> >
    class BinaryHeap
    {
    public:
	
        class Element
	{
	    friend class BinaryHeap;
	private:
	    unsigned int position;
	public:
	    _T           data;
	};
	
	typedef void (*EventAfterInsert) (Element*, void*);
	typedef void (*EventBeforeRemove)(Element*, void*);
	
	BinaryHeap(void)
	{
	    m_eventAfterInsert  = NULL;
	    m_eventBeforeRemove = NULL;
	}
	
	~BinaryHeap(void)
	{
	    clear();
	}
	
	void onAfterInsert(EventAfterInsert event, void *arg)
	{
	    m_eventAfterInsert = event;
	    m_eventAfterInsertData = arg;
	}

	void onBeforeRemove(EventBeforeRemove event, void *arg)
	{
	    m_eventBeforeRemove = event;
	    m_eventBeforeRemoveData = arg;
	}	

	void clear(void)
	{
	    for (typename std::vector<Element*>::iterator i = m_vector.begin() ;
		 i != m_vector.end() ; i++)
		delete *i;
	    m_vector.clear();
	}
	
	Element* top(void) const
	{
	    return m_vector.empty() ? NULL : m_vector.at(0);
	}

	void pop(void)
	{
	    removePos(0);
	}
	
	void remove(Element* element)
	{
	    if (m_eventBeforeRemove)
		m_eventBeforeRemove(element, m_eventBeforeRemoveData);
	    removePos(element->position);
	}
	
	Element* insert(const _T& data)
	{
	    Element* element = new Element();
	    element->data = data;
	    const unsigned int pos = m_vector.size();
	    element->position = pos;	    
	    m_vector.push_back(element);
	    percolateUp(pos);
	    if (m_eventAfterInsert)
		m_eventAfterInsert(element, m_eventAfterInsertData);
	    return element;
	}
	
	void insert(const std::vector<_T>& list)
	{
	    const unsigned int n = m_vector.size();
	    const unsigned int m = list.size();
	    for (unsigned int i = 0 ; i < m ; i++)
	    {
		const unsigned int pos = i + n;
		Element* element = newElement(list[i], pos);
		m_vector.push_back(element);
		percolateUp(pos);
		if (m_eventAfterInsert)
		    m_eventAfterInsert(element, m_eventAfterInsertData);
	    }
	}
	
	void buildFrom(const std::vector<_T>& list)
	{
	    const unsigned int m = list.size();
	    for (unsigned int i = 0 ; i < m ; i++)
		m_vector.push_back(newElement(list[i], i));
	    build();
	}

	void rebuild(void)
	{	    
	    build();
	}

	void update(Element* element)
	{
	    const unsigned int pos = element->position;
	    assert(m_vector[pos] == element);
	    percolateUp(pos);
	    percolateDown(pos);
	}
	
	bool empty(void) const
	{
	    return m_vector.empty();
	}
	
	unsigned int size(void) const
	{
	    return m_vector.size();
	}
	
	void getContent(std::vector<_T> &content) const
	{
	    for (typename std::vector<Element*>::const_iterator i = m_vector.begin();
		 i != m_vector.end() ; i++)
		content.push_back((*i)->data);
	}
	
	void sort(std::vector<_T>& list)
	{	    
	    const unsigned int n         = list.size();
	    std::vector<Element*> backup = m_vector;
	    m_vector.clear();
	    for (unsigned int i = 0 ; i < n ; i++)
		m_vector.push_back(newElement(list[i], i));
	    build();
	    list.clear();
	    list.reserve(n);
	    
	    for (unsigned int i = 0 ; i < n ; i++)
	    {
		list.push_back(m_vector[0]->data);
		removePos(0);
	    }
	    m_vector = backup;
	}
	
    protected:
	
	std::vector<Element*>    m_vector;

	EventAfterInsert         m_eventAfterInsert;
	void                    *m_eventAfterInsertData;
	EventBeforeRemove        m_eventBeforeRemove;
	void                    *m_eventBeforeRemoveData;
	
	void removePos(unsigned int pos)
	{
	    const int n = m_vector.size() - 1;
	    delete m_vector[pos];
	    if ((int)pos < n)
	    {
		m_vector[pos] = m_vector.back();
		m_vector[pos]->position = pos;
		m_vector.pop_back();
		percolateDown(pos);
	    }
	    else
		m_vector.pop_back();
	}
	
	Element* newElement(_T& data, unsigned int pos) const
	{
	    Element* element = new Element();
	    element->data = data;
	    element->position = pos;
	    return element;	    
	}

	void build(void)
	{
	    for(int i = m_vector.size() / 2 - 1; i >= 0; --i)
		percolateDown(i);
	}

	void percolateDown(const unsigned int pos)
	{
	    const unsigned int n      = m_vector.size();
	    Element*           tmp    = m_vector[pos];
	    unsigned int       parent = pos;
	    unsigned int       child  = (pos + 1) << 1;
	    
	    while (child < n)
	    {
		if (LessThan()(m_vector[child - 1]->data, m_vector[child]->data)) child--;
		if (LessThan()(m_vector[child]->data,  tmp->data))
		{
		    m_vector[parent] = m_vector[child];
		    m_vector[parent]->position = parent;
		}		
		else
		    break;
		parent = child;
		child  = (child + 1) << 1;
	    }
	    if (child == n)
	    {
		child--;
		if (LessThan()(m_vector[child]->data, tmp->data))
		{
		    m_vector[parent] = m_vector[child];
		    m_vector[parent]->position = parent;
		    parent = child;
		}
	    }
	    if (parent != pos)
	    {
		m_vector[parent] = tmp;
		m_vector[parent]->position = parent;
	    }
	}

	void percolateUp(const unsigned int pos)
	{
	    Element*           tmp    = m_vector[pos];
	    unsigned int       child  = pos;
	    unsigned int       parent = (pos - 1) >> 1;
	    
	    while (child > 0 && LessThan()(tmp->data, m_vector[parent]->data))
	    {
		m_vector[child] = m_vector[parent];
		m_vector[child]->position = child;
		child  = parent;
		parent = (parent - 1) >> 1;
	    }
	    if (child != pos)
	    {
		m_vector[child] = tmp;
		m_vector[child]->position = child;
	    }
	}
    };

}

#endif
