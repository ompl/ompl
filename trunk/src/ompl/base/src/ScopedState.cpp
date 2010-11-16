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

#include "ompl/base/ScopedState.h"

namespace ompl
{
    namespace base
    {
	
	/// @cond IGNORE
	
	static const int NO_DATA_COPIED   = 0;
	static const int SOME_DATA_COPIED = 1;
	static const int ALL_DATA_COPIED  = 2;
	
	// return 2 = all data in from copied, 1 = part of from copied, 0 = nothing copied
	int insertStateData(ScopedState<> &to, const ScopedState<> &from)
	{    
	    // if states correspond to the same manifold, simply do copy
	    if (to.getManifold()->getName() == from.getManifold()->getName())
	    {
		to = from;
		return ALL_DATA_COPIED;
	    }
	    
	    int result = NO_DATA_COPIED;
	    
	    // if "to" state is compound
	    CompoundStateManifold *toM = dynamic_cast<CompoundStateManifold*>(to.getManifold().get());
	    if (toM)
	    {
		CompoundState *dest = dynamic_cast<CompoundState*>(to.get());
		if (!dest)
		    throw Exception("State allocated by compound manifold is not compound");
		
		// if there is a submanifold in "to" that corresponds to "from", set the data and return
		for (unsigned int i = 0 ; i < toM->getSubManifoldCount() ; ++i)
		    if (toM->getSubManifold(i)->getName() == from.getManifold()->getName())
		    {
			from.getManifold()->copyState(dest->components[i], from.get());
			return ALL_DATA_COPIED;
		    }
		
		// it could be there are further levels of compound manifolds where the data can be set
		// so we call this function recursively
		for (unsigned int i = 0 ; i < toM->getSubManifoldCount() ; ++i)
		{
		    // get a component of "to" and store it in temp
		    ScopedState<> temp(toM->getSubManifold(i));
		    temp = dest->components[i];
		    int res = insertStateData(temp, from);
		    
		    // if some data was copied to temp, put it back in "to"
		    if (res != NO_DATA_COPIED)
		    {
			temp.getManifold()->copyState(dest->components[i], temp.get());
			result = SOME_DATA_COPIED;
		    }
		    
		    // if all data was copied, we stop
		    if (res == ALL_DATA_COPIED)
			return ALL_DATA_COPIED;
		}
	    }
	    
	    // if we got to this point, it means that the data in "from" could not be copied as a chunk to "to"
	    // it could be the case "from" is from a compound manifold as well, so we can copy parts of "from", as needed
	    CompoundStateManifold *fromM = dynamic_cast<CompoundStateManifold*>(from.getManifold().get());
	    if (fromM)
	    {
		CompoundState *source = dynamic_cast<CompoundState*>(from.get());
		if (!source)
		    throw Exception("State allocated by compound manifold is not compound");

		unsigned int copiedComponents = 0;
		
		// if there is a submanifold in "to" that corresponds to "from", set the data and return
		for (unsigned int i = 0 ; i < fromM->getSubManifoldCount() ; ++i)
		{
		    ScopedState<> temp(fromM->getSubManifold(i));
		    temp = source->components[i];
		    int res = insertStateData(to, temp);
		    if (res == ALL_DATA_COPIED)
			copiedComponents++;
		    if (res)
			result = SOME_DATA_COPIED;
		}
		
		// if each individual component got copied, then the entire data in "from" got copied
		if (copiedComponents == fromM->getSubManifoldCount())
		    result = ALL_DATA_COPIED;
	    }
	    
	    return result;
	}
	
	/// @endcond
    }
}
