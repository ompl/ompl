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

ompl::base::State* ompl::base::StateAllocator::allocState(void) const
{
    State *result = NULL;
    
    StorageType::iterator it = storage_.find(boost::this_thread::get_id());
    
    if (it != storage_.end())
    {
	if (it->second.empty())
	    result = manifold_->allocState();
	else
	{
	    result = it->second.top();
	    it->second.pop();
	}
    }
    else
	result = manifold_->allocState();
    
    return result;
}

void ompl::base::StateAllocator::freeState(State *state) const
{
    boost::thread::id id = boost::this_thread::get_id();
    StorageType::iterator it = storage_.find(id);
    
    if (it != storage_.end())
	it->second.push(state);
    else
    {
	lock_.lock();
	storage_[id].push(state);
	lock_.unlock();
    }
}

unsigned int ompl::base::StateAllocator::size(void) const
{
    unsigned int s = 0;
    for (StorageType::const_iterator it = storage_.begin() ; it != storage_.end() ; ++it)
	s += it->second.size();
    return s;
}

void ompl::base::StateAllocator::clear(void)
{
    for (StorageType::iterator it = storage_.begin() ; it != storage_.end() ; ++it)
	while (!it->second.empty())
	{
	    manifold_->freeState(it->second.top());
	    it->second.pop();
	}
    storage_.clear();
}
