/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_LTL_WORLD_
#define OMPL_CONTROL_PLANNERS_LTL_WORLD_

#include <unordered_map>
#include <string>

namespace ompl
{
    namespace control
    {
        class World;
    }
}

/// @cond IGNORE
/** \brief Hash function for World to be used in std::unordered_map */
namespace std
{
    template <>
    struct hash<ompl::control::World>
    {
        size_t operator()(const ompl::control::World &w) const;
    };
}
/// @endcond

namespace ompl
{
    namespace control
    {
        /** \brief A class to represent an assignment of boolean values to propositions.
            A World can be partially restrictive, i.e., some propositions do not have to
            be assigned a value, in which case it can take on any value.
            Our notion of a World is similar to a set of truth assignments in propositional logic. */
        class World
        {
        public:
            /** \brief Initializes a world with a given number of propositions. */
            World(unsigned int numProps);

            /** \brief Returns the boolean value of a given proposition in this World.
                Reports an error if the proposition has not set in this World. */
            bool operator[](unsigned int i) const;

            /** \brief Returns the boolean value of a given proposition in this World.
                Creates a boolean value for the proposition if one does not already exist. */
            bool &operator[](unsigned int i);

            /** \brief Returns the number of propositions declared for this World.
                Not all of the propositions have necessarily been set. */
            unsigned int numProps() const;

            /** \brief Returns whether this World propositionally satisfies a given World w.
                Specifically, returns true iff for every proposition p assigned in w,
                p is assigned in this World and this[p] == w[p]. */
            bool satisfies(const World &w) const;

            /** \brief Returns a formatted string representation of this World,
                as a conjunction of literals. */
            std::string formula() const;

            /** \brief Returns this World's underlying proposition-to-boolean
                assignment map. */
            const std::unordered_map<unsigned int, bool> &props() const;

            /** \brief Returns whether this World is equivalent to a given World,
                by comparing their truth assignment maps. */
            bool operator==(const World &w) const;

            /** \brief Clears this world's truth assignment. */
            void clear();

            friend struct std::hash<World>;

        protected:
            unsigned int numProps_;
            std::unordered_map<unsigned int, bool> props_;
        };
    }
}
#endif
