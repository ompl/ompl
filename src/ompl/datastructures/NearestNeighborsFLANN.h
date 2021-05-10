/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Mark Moll */

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_FLANN_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_FLANN_

#include "ompl/config.h"
#if OMPL_HAVE_FLANN == 0
#error FLANN is not available. Please use a different NearestNeighbors data structure.
#else

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/util/Exception.h"

#include <flann/flann.hpp>
#include <utility>

namespace ompl
{
    /** \brief Wrapper class to allow FLANN access to the
        NearestNeighbors::distFun_ callback function
    */
    template <typename _T>
    class FLANNDistance
    {
    public:
        using ElementType = _T;
        using ResultType = double;

        FLANNDistance(const typename NearestNeighbors<_T>::DistanceFunction &distFun) : distFun_(distFun)
        {
        }

        template <typename Iterator1, typename Iterator2>
        ResultType operator()(Iterator1 a, Iterator2 b, size_t /*size*/, ResultType /*worst_dist*/ = -1) const
        {
            return distFun_(*a, *b);
        }

    protected:
        const typename NearestNeighbors<_T>::DistanceFunction &distFun_;
    };

    /** \brief Wrapper class for nearest neighbor data structures in the
        FLANN library.

        See:
        M. Muja and D.G. Lowe, "Fast Approximate Nearest Neighbors with
        Automatic Algorithm Configuration", in International Conference
        on Computer Vision Theory and Applications (VISAPP'09), 2009.
        https://github.com/mariusmuja/flann
    */
    template <typename _T, typename _Dist = FLANNDistance<_T>>
    class NearestNeighborsFLANN : public NearestNeighbors<_T>
    {
    public:
        NearestNeighborsFLANN(std::shared_ptr<flann::IndexParams> params)
          : index_(nullptr), params_(std::move(params)), searchParams_(32, 0., true), dimension_(1)
        {
        }

        ~NearestNeighborsFLANN() override
        {
            if (index_)
                delete index_;
        }

        void clear() override
        {
            if (index_)
            {
                delete index_;
                index_ = nullptr;
            }
            data_.clear();
        }

        bool reportsSortedResults() const override
        {
            return searchParams_.sorted;
        }

        void setDistanceFunction(const typename NearestNeighbors<_T>::DistanceFunction &distFun) override
        {
            NearestNeighbors<_T>::setDistanceFunction(distFun);
            rebuildIndex();
        }

        void add(const _T &data) override
        {
            bool rebuild = index_ && (data_.size() + 1 > data_.capacity());

            if (rebuild)
                rebuildIndex(2 * data_.capacity());

            data_.push_back(data);
            const flann::Matrix<_T> mat(&data_.back(), 1, dimension_);

            if (index_)
                index_->addPoints(mat, std::numeric_limits<float>::max() / size());
            else
                createIndex(mat);
        }
        void add(const std::vector<_T> &data) override
        {
            if (data.empty()) return;
            unsigned int oldSize = data_.size();
            unsigned int newSize = oldSize + data.size();
            bool rebuild = index_ && (newSize > data_.capacity());

            if (rebuild)
                rebuildIndex(std::max(2 * oldSize, newSize));

            if (index_)
            {
                std::copy(data.begin(), data.end(), data_.begin() + oldSize);
                const flann::Matrix<_T> mat(&data_[oldSize], data.size(), dimension_);
                index_->addPoints(mat, std::numeric_limits<float>::max() / size());
            }
            else
            {
                data_ = data;
                const flann::Matrix<_T> mat(&data_[0], data_.size(), dimension_);
                createIndex(mat);
            }
        }
        bool remove(const _T &data) override
        {
            if (!index_)
                return false;
            auto &elt = const_cast<_T &>(data);
            const flann::Matrix<_T> query(&elt, 1, dimension_);
            std::vector<std::vector<size_t>> indices(1);
            std::vector<std::vector<double>> dists(1);
            index_->knnSearch(query, indices, dists, 1, searchParams_);
            if (*index_->getPoint(indices[0][0]) == data)
            {
                index_->removePoint(indices[0][0]);
                rebuildIndex();
                return true;
            }
            return false;
        }
        _T nearest(const _T &data) const override
        {
            if (size())
            {
                auto &elt = const_cast<_T &>(data);
                const flann::Matrix<_T> query(&elt, 1, dimension_);
                std::vector<std::vector<size_t>> indices(1);
                std::vector<std::vector<double>> dists(1);
                index_->knnSearch(query, indices, dists, 1, searchParams_);
                return *index_->getPoint(indices[0][0]);
            }
            throw Exception("No elements found in nearest neighbors data structure");
        }
        /// \brief Return the k nearest neighbors in sorted order if
        /// searchParams_.sorted==true (the default)
        void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const override
        {
            auto &elt = const_cast<_T &>(data);
            const flann::Matrix<_T> query(&elt, 1, dimension_);
            std::vector<std::vector<size_t>> indices;
            std::vector<std::vector<double>> dists;
            k = index_ ? index_->knnSearch(query, indices, dists, k, searchParams_) : 0;
            nbh.resize(k);
            for (std::size_t i = 0; i < k; ++i)
                nbh[i] = *index_->getPoint(indices[0][i]);
        }
        /// \brief Return the nearest neighbors within distance \c radius in sorted
        /// order if searchParams_.sorted==true (the default)
        void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const override
        {
            auto &elt = const_cast<_T &>(data);
            flann::Matrix<_T> query(&elt, 1, dimension_);
            std::vector<std::vector<size_t>> indices;
            std::vector<std::vector<double>> dists;
            int k = index_ ? index_->radiusSearch(query, indices, dists, radius, searchParams_) : 0;
            nbh.resize(k);
            for (int i = 0; i < k; ++i)
                nbh[i] = *index_->getPoint(indices[0][i]);
        }

        std::size_t size() const override
        {
            return index_ ? index_->size() : 0;
        }

        void list(std::vector<_T> &data) const override
        {
            std::size_t sz = size();
            if (sz == 0)
            {
                data.resize(0);
                return;
            }
            const _T &dummy = *index_->getPoint(0);
            int checks = searchParams_.checks;
            searchParams_.checks = size();
            nearestK(dummy, sz, data);
            searchParams_.checks = checks;
        }

        /// \brief Set the FLANN index parameters.
        ///
        /// The parameters determine the type of nearest neighbor
        /// data structure to be constructed.
        virtual void setIndexParams(const std::shared_ptr<flann::IndexParams> &params)
        {
            params_ = params;
            rebuildIndex();
        }

        /// \brief Get the FLANN parameters used to build the current index.
        virtual const std::shared_ptr<flann::IndexParams> &getIndexParams() const
        {
            return params_;
        }

        /// \brief Set the FLANN parameters to be used during nearest neighbor
        /// searches
        virtual void setSearchParams(const flann::SearchParams &searchParams)
        {
            searchParams_ = searchParams;
        }

        /// \brief Get the FLANN parameters used during nearest neighbor
        /// searches
        flann::SearchParams &getSearchParams()
        {
            return searchParams_;
        }

        /// \brief Get the FLANN parameters used during nearest neighbor
        /// searches
        const flann::SearchParams &getSearchParams() const
        {
            return searchParams_;
        }

        unsigned int getContainerSize() const
        {
            return dimension_;
        }

    protected:
        /// \brief Internal function to construct nearest neighbor
        /// data structure with initial elements stored in mat.
        void createIndex(const flann::Matrix<_T> &mat)
        {
            index_ = new flann::Index<_Dist>(mat, *params_, _Dist(NearestNeighbors<_T>::distFun_));
            index_->buildIndex();
        }

        /// \brief Rebuild the nearest neighbor data structure (necessary when
        /// changing the distance function or index parameters).
        void rebuildIndex(unsigned int capacity = 0)
        {
            if (index_)
            {
                std::vector<_T> data;
                list(data);
                clear();
                if (capacity != 0u)
                    data_.reserve(capacity);
                add(data);
            }
        }

        /// \brief vector of data stored in FLANN's index. FLANN only indexes
        /// references, so we need store the original data.
        std::vector<_T> data_;

        /// \brief The FLANN index (the actual index type depends on params_).
        flann::Index<_Dist> *index_;

        /// \brief The FLANN index parameters. This contains both the type of
        /// index and the parameters for that type.
        std::shared_ptr<flann::IndexParams> params_;

        /// \brief The parameters used to seach for nearest neighbors.
        mutable flann::SearchParams searchParams_;

        /// \brief If each element has an array-like structure that is exposed
        /// to FLANN, then the dimension_ needs to be set to the length of
        /// this array.
        unsigned int dimension_;
    };

    template <>
    inline void NearestNeighborsFLANN<double, flann::L2<double>>::createIndex(
        const flann::Matrix<double> &mat)
    {
        index_ = new flann::Index<flann::L2<double>>(mat, *params_);
        index_->buildIndex();
    }

    template <typename _T, typename _Dist = FLANNDistance<_T>>
    class NearestNeighborsFLANNLinear : public NearestNeighborsFLANN<_T, _Dist>
    {
    public:
        NearestNeighborsFLANNLinear()
          : NearestNeighborsFLANN<_T, _Dist>(std::shared_ptr<flann::LinearIndexParams>(new flann::LinearIndexParams()))
        {
        }
    };

    template <typename _T, typename _Dist = FLANNDistance<_T>>
    class NearestNeighborsFLANNHierarchicalClustering : public NearestNeighborsFLANN<_T, _Dist>
    {
    public:
        NearestNeighborsFLANNHierarchicalClustering()
          : NearestNeighborsFLANN<_T, _Dist>(std::shared_ptr<flann::HierarchicalClusteringIndexParams>(
                new flann::HierarchicalClusteringIndexParams()))
        {
        }
    };
}
#endif

#endif
