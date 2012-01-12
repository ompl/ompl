/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage
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

/* Author: Ioan Sucan */

#include "ompl/base/StateSpace.h"
#include <iostream>

namespace ompl
{
    namespace base
    {

        /** \brief Manage loading and storing for a set of states of a specified state space */
        class StateStorage
        {
        public:

            /** \brief The state space to store states for */
            StateStorage(const StateSpacePtr &space);
            ~StateStorage(void);

            /** \brief Load a set of states from a specified file */
            void load(const char *filename);

            /** \brief Load a set of states from a stream */
            void load(std::istream &in);

            /** \brief Save a set of states to a file */
            void store(const char *filename);

            /** \brief Save a set of states to a stream */
            void store(std::ostream &out);

            /** \brief Add a state to the set of states maintained by
                this storage structure. Ownership of the state is
                assumed (i.e., the state will be freed when the
                StateStorage instance goes out of scope) */
            void addState(const State *state);

            /** \brief Generate \e count states uniformly at random and store them in this structure */
            void generateSamples(unsigned int count);

            /** \brief Clear the stored states. This frees all the memory */
            void clear(void);

            /** \brief Return the number of stored states */
            std::size_t size(void) const
            {
                return states_.size();
            }

            /** \brief Get the stored states */
            const std::vector<const State*>& getStates(void) const
            {
                return states_;
            }

            /** \brief Get a sampler allocator to a sampler that can
                be specified for a StateSpace, such that all sampled
                states are actually from this storage structure. */
            StateSamplerAllocator getStateSamplerAllocator(void) const;

            /** \brief Output the set of states to a specified stream, in a human readable fashion */
            void print(std::ostream &out = std::cout) const;

        private:

            StateSpacePtr             space_;
            std::vector<const State*> states_;
            msg::Interface            msg_;
        };

    }
}
