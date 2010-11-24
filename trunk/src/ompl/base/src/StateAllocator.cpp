/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

#include "ompl/base/StateAllocator.h"
#include "ompl/util/Exception.h"
#include <boost/thread.hpp>
#include <stack>
#include <list>
#include <map>

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        
        // This class corresponds to an instance of a StateAllocator. It gets deleted only after the
        // StateAllocator is deleted and all threads that use this instance have terminated.
        // This ensures that all stacks of states will be cleared.
        class AllocatorStacks
        {
        public:
            AllocatorStacks(const StateManifoldPtr &manifold) : manifold_(manifold)
            {
            }
            
            ~AllocatorStacks(void)
            {
                freeMemory();
            }
            
            std::stack<State*>* getStack(void)
            {
                boost::mutex::scoped_lock lock(lock_);
                if (avail_.empty())
                {
                    std::stack<State*> *s = new std::stack<State*>();
                    inuse_.push_back(s);
                    return s;
                }
                
                std::list< std::stack<State*>* >::iterator pos = avail_.begin();
                for (std::list< std::stack<State*>* >::iterator it = ++avail_.begin() ; it != avail_.end() ; ++it)
                    if ((*pos)->size() < (*it)->size())
                        pos = it;
                std::stack<State*> *s = *pos;
                avail_.erase(pos);
                inuse_.push_back(s);
                return s;
            }
            
            void returnStack(std::stack<State*> *s)
            {
                boost::mutex::scoped_lock lock(lock_);
                std::list< std::stack<State*>* >::iterator pos = std::find(inuse_.begin(), inuse_.end(), s);
                if (pos == inuse_.end())
                    throw Exception("Returning stack that was not in use. This should not happen");
                inuse_.erase(pos);
                avail_.push_back(s);
            }
            
            void clearAvailStacks(void)
            {
                boost::mutex::scoped_lock lock(lock_);
                for (std::list< std::stack<State*>* >::iterator it = avail_.begin() ; it != avail_.end() ; ++it)
                {
                    clearStack(**it);
                    delete *it;
                }
                avail_.clear();
            }
            
            std::pair<std::size_t,std::size_t> size(void) 
            {
                std::size_t sz1 = 0;
                std::size_t sz2 = 0;
                boost::mutex::scoped_lock lock(lock_);
                for (std::list< std::stack<State*>* >::iterator it = avail_.begin() ; it != avail_.end() ; ++it)
                    sz1 += (*it)->size();
                for (std::list< std::stack<State*>* >::iterator it = inuse_.begin() ; it != inuse_.end() ; ++it)
                    sz2 += (*it)->size();
                return std::make_pair(sz1, sz2);
            }
            
        private:
            
            void clearStack(std::stack<State*> &s) const
            {
                while (!s.empty())
                {
                    manifold_->freeState(s.top());
                    s.pop();
                }
            }
            
            void freeMemory(void)
            {
                clearAvailStacks();
                if (!inuse_.empty())
                    throw Exception("Stack of states to be freed is in use by a thread. This should not happen.");
            }
            
            StateManifoldPtr                 manifold_;
            std::list< std::stack<State*>* > inuse_;
            std::list< std::stack<State*>* > avail_;
            boost::mutex                     lock_;
        };
        
        // this class gets allocated by each thread. it holds a shared pointer to the instance of AllocatorStacks 
        // associated to the state allocator. Even if the StateAllocator has been deleted, the AllocatorStacks will exist
        // and this instance will be able to return its stack whenever the thread terminates
        class ThreadStorage
        {
        public:
            
            ThreadStorage(const boost::shared_ptr<AllocatorStacks> &availableStacks) : astacks(availableStacks)
            {
                stack = astacks->getStack();
            }
            
            ~ThreadStorage(void)
            {
                astacks->returnStack(stack);
            }
            
            boost::shared_ptr<AllocatorStacks>  astacks;
            std::stack<base::State*>           *stack;
        };
        
        // this object is created for every instance of StateAllocator and casted as void*
        // to avoid having to include too many header files in the .h file
        // this class is allocated by the constructor of StateAllocator and freed by the destructor.
        class StateAllocatorData
        {
        public:
            StateAllocatorData(const StateManifoldPtr &manifold) : astacks(new AllocatorStacks(manifold))
            {
            }
            
            boost::shared_ptr<AllocatorStacks>        astacks;
            boost::thread_specific_ptr<ThreadStorage> ts;
        };
    }
}

#define CDATA reinterpret_cast<StateAllocatorData*>(data_)

/// @endcond

ompl::base::StateAllocator::StateAllocator(const StateManifoldPtr &manifold) : manifold_(manifold)
{
    data_ = new StateAllocatorData(manifold);
}

ompl::base::StateAllocator::~StateAllocator(void)
{
    clear();
    delete CDATA;
}

ompl::base::State* ompl::base::StateAllocator::allocState(void) const
{
    if (!CDATA->ts.get())
        CDATA->ts.reset(new ThreadStorage(CDATA->astacks));
    
    if (!CDATA->ts->stack->empty())
    {
        State *result = CDATA->ts->stack->top();
        CDATA->ts->stack->pop();
        return result;
    }

    return manifold_->allocState();
}

void ompl::base::StateAllocator::freeState(State *state) const
{
    if (!CDATA->ts.get())
        CDATA->ts.reset(new ThreadStorage(CDATA->astacks));
    CDATA->ts->stack->push(state);
}

std::size_t ompl::base::StateAllocator::sizeTotal(void) const
{
    std::pair<std::size_t, std::size_t> s = CDATA->astacks->size();
    return s.first + s.second;
}

std::size_t ompl::base::StateAllocator::sizeAvailable(void) const
{
    std::pair<std::size_t, std::size_t> s = CDATA->astacks->size();
    return s.first;
}

std::size_t ompl::base::StateAllocator::sizeInUse(void) const
{
    std::pair<std::size_t, std::size_t> s = CDATA->astacks->size();
    return s.second;
}

void ompl::base::StateAllocator::clear(void)
{
    CDATA->ts.reset();
    CDATA->astacks->clearAvailStacks();
}
