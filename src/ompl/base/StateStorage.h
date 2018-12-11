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

#ifndef OMPL_BASE_STATE_STORAGE_
#define OMPL_BASE_STATE_STORAGE_

#include "ompl/base/StateSpace.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <functional>
#include <iostream>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::StateStorage */
        OMPL_CLASS_FORWARD(StateStorage);
        /// @endcond

        /** \brief Manage loading and storing for a set of states of a specified state space

            \deprecated This class is deprecated and will be removed in the future. Please use the improved
           PlannerDataStorage.
         */
        class StateStorage
        {
        public:
            /** \brief The state space to store states for is specified as argument */
            StateStorage(StateSpacePtr space);
            virtual ~StateStorage();

            /** \brief Get the state space this class maintains states for */
            const StateSpacePtr &getStateSpace() const
            {
                return space_;
            }

            /** \brief Load a set of states from a specified file */
            void load(const char *filename);

            /** \brief Load a set of states from a stream */
            virtual void load(std::istream &in);

            /** \brief Save a set of states to a file */
            void store(const char *filename);

            /** \brief Save a set of states to a stream */
            virtual void store(std::ostream &out);

            /** \brief Add a state to the set of states maintained by
                this storage structure. The state is copied to internal storage */
            virtual void addState(const State *state);

            /** \brief Generate \e count states uniformly at random and store them in this structure */
            virtual void generateSamples(unsigned int count);

            /** \brief Clear the stored states. This frees all the memory */
            virtual void clear();

            /** \brief Return the number of stored states */
            std::size_t size() const
            {
                return states_.size();
            }

            /** \brief Get the stored states */
            const std::vector<const State *> &getStates() const
            {
                return states_;
            }

            /** \brief Get a particular state for non-const access */
            State *getState(unsigned int index)
            {
                assert(states_.size() > index);
                return const_cast<State *>(states_[index]);
            }

            /** \brief Get a particular state */
            const State *getState(unsigned int index) const
            {
                assert(states_.size() > index);
                return states_[index];
            }

            /** \brief Return a flag that indicates whether there is metadata associated to the states in this storage
             */
            bool hasMetadata() const
            {
                return hasMetadata_;
            }

            /** \brief Sort the states according to the less-equal operator \e op. Metadata is NOT sorted;
                if metadata was added, the index values of the metadata will not match after the sort. */
            void sort(const std::function<bool(const State *, const State *)> &op);

            /** \brief Get a sampler allocator to a sampler that can be specified for a StateSpace, such that all
               sampled
                states are actually from this storage structure. */
            StateSamplerAllocator getStateSamplerAllocator() const;

            /** \brief Get a sampler allocator to a sampler that can be specified for a StateSpace, such that all
               sampled
                states are actually from this storage structure at an index less than or equal to \e until */
            StateSamplerAllocator getStateSamplerAllocatorRangeUntil(std::size_t until) const;

            /** \brief Get a sampler allocator to a sampler that can be specified for a StateSpace, such that all
               sampled
                states are actually from this storage structure at an index above or equal to \e after */
            StateSamplerAllocator getStateSamplerAllocatorRangeAfter(std::size_t after) const;

            /** \brief Get a sampler allocator to a sampler that can be specified for a StateSpace, such that all
               sampled
                states are actually from this storage structure at an index in the range [\e from, \e to] (inclusive) */
            virtual StateSamplerAllocator getStateSamplerAllocatorRange(std::size_t from, std::size_t to) const;

            /** \brief Output the set of states to a specified stream, in a human readable fashion */
            virtual void print(std::ostream &out = std::cout) const;

        protected:
            /** \brief Information stored at the beginning of the archive */
            struct Header
            {
                /** \brief OMPL specific marker (fixed value) */
                std::uint_fast32_t marker;

                /** \brief Number of states stored in the archive */
                std::size_t state_count;

                /** \brief Signature of state space that allocated the saved states (see
                 * ompl::base::StateSpace::computeSignature()) */
                std::vector<int> signature;

                /** \brief boost::serialization routine */
                template <typename Archive>
                void serialize(Archive &ar, const unsigned int /*version*/)
                {
                    ar &marker;
                    ar &state_count;
                    ar &signature;
                }
            };

            /** \brief Load the states from a binary archive \e ia, given the loaded header is \e h */
            virtual void loadStates(const Header &h, boost::archive::binary_iarchive &ia);

            /** \brief Load the state metadata from a binary archive
                \e ia, given the loaded header is \e h. No metadata is
                actually loaded unless the StateStorageWithMetadata
                class is used.*/
            virtual void loadMetadata(const Header &h, boost::archive::binary_iarchive &ia);

            /** \brief Store the states to a binary archive \e oa, given the stored header is \e h */
            virtual void storeStates(const Header &h, boost::archive::binary_oarchive &oa);

            /** \brief Save the state metadata to a binary archive
                \e oa, given the stored header is \e h. No metadata is
                actually saved unless the StateStorageWithMetadata
                class is used.*/
            virtual void storeMetadata(const Header &h, boost::archive::binary_oarchive &oa);

            /** \brief Free the memory allocated for states */
            void freeMemory();

            /** \brief State space that corresponds to maintained states */
            StateSpacePtr space_;

            /** \brief The list of maintained states */
            std::vector<const State *> states_;

            /** \brief Flag indicating whether there is metadata associated to the states in this storage */
            bool hasMetadata_;
        };

        /** \brief State storage that allows storing state metadata as well
            \tparam M the datatype for the stored metadata. boost::serialization operation needs to be defined */
        template <typename M>
        class StateStorageWithMetadata : public StateStorage
        {
        public:
            /** \brief the datatype of the metadata */
            using MetadataType = M;

            /** \brief The state space to store states for is specified as argument */
            StateStorageWithMetadata(const StateSpacePtr &space) : StateStorage(space)
            {
                hasMetadata_ = true;
            }

            /** \brief Add a state to the set of states maintained by
                this storage structure. The state is copied to
                internal storage and metadata with default values is stored as well. */
            void addState(const State *state) override
            {
                addState(state, M());
            }

            /** \brief Add a state to the set of states maintained by
                this storage structure. The state is copied to internal storage. Corresponding metadata is stored too.
               */
            virtual void addState(const State *state, const M &metadata)
            {
                StateStorage::addState(state);
                metadata_.push_back(metadata);
            }

            void clear() override
            {
                StateStorage::clear();
                metadata_.clear();
            }

            /** \brief Get const access to the metadata of a state at a particular index */
            const M &getMetadata(unsigned int index) const
            {
                assert(metadata_.size() > index);
                return metadata_[index];
            }

            /** \brief Get write access to the metadata of a state at a particular index */
            M &getMetadata(unsigned int index)
            {
                assert(metadata_.size() > index);
                return metadata_[index];
            }

        protected:
            void loadMetadata(const Header & /*h*/, boost::archive::binary_iarchive &ia) override
            {
                // clear default metadata that was added by StateStorage::loadStates()
                metadata_.clear();
                ia >> metadata_;
            }

            void storeMetadata(const Header & /*h*/, boost::archive::binary_oarchive &oa) override
            {
                oa << metadata_;
            }

            /** \brief The metadata for each state */
            std::vector<M> metadata_;
        };

        /** \brief Storage of states where the metadata is a vector of indices. This is is typically used to store a
         * graph */
        using GraphStateStorage = StateStorageWithMetadata<std::vector<std::size_t>>;
        using GraphStateStoragePtr = std::shared_ptr<GraphStateStorage>;
    }
}
#endif
