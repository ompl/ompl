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

/* Author: Mark Moll, Bryant Gipson */

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_GNAT_NO_THREAD_SAFETY_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_GNAT_NO_THREAD_SAFETY_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/GreedyKCenters.h"
#include "ompl/datastructures/Permutation.h"
#ifdef GNAT_SAMPLER
#include "ompl/datastructures/PDF.h"
#endif
#include "ompl/util/Exception.h"
#include <unordered_set>
#include <queue>
#include <algorithm>
#include <utility>

namespace ompl
{
    /** \brief Geometric Near-neighbor Access Tree (GNAT), a data
        structure for nearest neighbor search.

        If GNAT_SAMPLER is defined, then extra code will be enabled to sample
        elements from the GNAT with probability inversely proportial to their
        local density.

        @par External documentation
        S. Brin, Near neighbor search in large metric spaces, in <em>Proc. 21st
        Conf. on Very Large Databases (VLDB)</em>, pp. 574–584, 1995.

        B. Gipson, M. Moll, and L.E. Kavraki, Resolution independent density
         estimation for motion planning in high-dimensional spaces, in
        <em>IEEE Intl. Conf. on Robotics and Automation</em>, 2013.
        [[PDF]](http://kavrakilab.org/sites/default/files/2013%20resolution%20independent%20density%20estimation%20for%20motion.pdf)
    */
    template <typename _T>
    class NearestNeighborsGNATNoThreadSafety : public NearestNeighbors<_T>
    {
    protected:
        /// \cond IGNORE
        // internally, we use a priority queue for nearest neighbors, paired
        // with their distance to the query point
        using NearQueue = std::priority_queue<std::pair<double, const _T *>>;
        /// \endcond

    public:
        NearestNeighborsGNATNoThreadSafety(unsigned int degree = 8, unsigned int minDegree = 4,
                                           unsigned int maxDegree = 12, unsigned int maxNumPtsPerLeaf = 50,
                                           unsigned int removedCacheSize = 500, bool rebalancing = false
#ifdef GNAT_SAMPLER
                                           ,
                                           double estimatedDimension = 6.0
#endif
                                           )
          : NearestNeighbors<_T>()
          , degree_(degree)
          , minDegree_(std::min(degree, minDegree))
          , maxDegree_(std::max(maxDegree, degree))
          , maxNumPtsPerLeaf_(maxNumPtsPerLeaf)
          , rebuildSize_(rebalancing ? maxNumPtsPerLeaf * degree : std::numeric_limits<std::size_t>::max())
          , removedCacheSize_(removedCacheSize)
          , permutation_(maxDegree_)
          , pivots_(maxDegree_)
#ifdef GNAT_SAMPLER
          , estimatedDimension_(estimatedDimension)
#endif
        {
        }

        ~NearestNeighborsGNATNoThreadSafety() override
        {
            delete tree_;
        }
        /// \brief Set the distance function to use
        void setDistanceFunction(const typename NearestNeighbors<_T>::DistanceFunction &distFun) override
        {
            NearestNeighbors<_T>::setDistanceFunction(distFun);
            pivotSelector_.setDistanceFunction(distFun);
            if (tree_)
                rebuildDataStructure();
        }

        void clear() override
        {
            if (tree_)
            {
                delete tree_;
                tree_ = nullptr;
            }
            size_ = 0;
            removed_.clear();
            if (rebuildSize_ != std::numeric_limits<std::size_t>::max())
                rebuildSize_ = maxNumPtsPerLeaf_ * degree_;
        }

        bool reportsSortedResults() const override
        {
            return true;
        }

        void add(const _T &data) override
        {
            if (tree_)
            {
                if (isRemoved(data))
                    rebuildDataStructure();
                tree_->add(*this, data);
            }
            else
            {
                tree_ = new Node(degree_, maxNumPtsPerLeaf_, data);
                size_ = 1;
            }
        }
        void add(const std::vector<_T> &data) override
        {
            if (tree_)
                NearestNeighbors<_T>::add(data);
            else if (!data.empty())
            {
                tree_ = new Node(degree_, maxNumPtsPerLeaf_, data[0]);
#ifdef GNAT_SAMPLER
                tree_->subtreeSize_ = data.size();
#endif
                tree_->data_.insert(tree_->data_.end(), data.begin() + 1, data.end());
                size_ += data.size();
                if (tree_->needToSplit(*this))
                    tree_->split(*this);
            }
        }
        /// \brief Rebuild the internal data structure.
        void rebuildDataStructure()
        {
            std::vector<_T> lst;
            list(lst);
            clear();
            add(lst);
        }
        /// \brief Remove data from the tree.
        /// The element won't actually be removed immediately, but just marked
        /// for removal in the removed_ cache. When the cache is full, the tree
        /// will be rebuilt and the elements marked for removal will actually
        /// be removed.
        bool remove(const _T &data) override
        {
            if (size_ == 0u)
                return false;
            // find data in tree
            bool isPivot = nearestKInternal(data, 1);
            const _T *d = nearQueue_.top().second;
            nearQueue_.pop();
            if (*d != data)
                return false;
            removed_.insert(d);
            size_--;
            // if we removed a pivot or if the capacity of removed elements
            // has been reached, we rebuild the entire GNAT
            if (isPivot || removed_.size() >= removedCacheSize_)
                rebuildDataStructure();
            return true;
        }

        _T nearest(const _T &data) const override
        {
            if (size_)
            {
                nearestKInternal(data, 1);
                if (!nearQueue_.empty())
                {
                    _T result = *nearQueue_.top().second;
                    nearQueue_.pop();
                    return result;
                }
            }
            throw Exception("No elements found in nearest neighbors data structure");
        }

        /// Return the k nearest neighbors in sorted order
        void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const override
        {
            nbh.clear();
            if (k == 0)
                return;
            if (size_)
            {
                nearestKInternal(data, k);
                postprocessNearest(nbh);
            }
        }

        /// Return the nearest neighbors within distance \c radius in sorted order
        void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const override
        {
            nbh.clear();
            if (size_)
            {
                nearestRInternal(data, radius);
                postprocessNearest(nbh);
            }
            assert(nearQueue_.empty());
            assert(nodeQueue_.empty());
        }

        std::size_t size() const override
        {
            return size_;
        }

#ifdef GNAT_SAMPLER
        /// Sample an element from the GNAT.
        const _T &sample(RNG &rng) const
        {
            if (!size())
                throw Exception("Cannot sample from an empty tree");
            else
                return tree_->sample(*this, rng);
        }
#endif

        void list(std::vector<_T> &data) const override
        {
            data.clear();
            data.reserve(size());
            if (tree_)
                tree_->list(*this, data);
        }

        /// \brief Print a GNAT structure (mostly useful for debugging purposes).
        friend std::ostream &operator<<(std::ostream &out, const NearestNeighborsGNATNoThreadSafety<_T> &gnat)
        {
            if (gnat.tree_)
            {
                out << *gnat.tree_;
                if (!gnat.removed_.empty())
                {
                    out << "Elements marked for removal:\n";
                    for (const auto &elt : gnat.removed_)
                        out << *elt << '\t';
                    out << std::endl;
                }
            }
            return out;
        }

        // for debugging purposes
        void integrityCheck()
        {
            std::vector<_T> lst;
            std::unordered_set<const _T *> tmp;
            // get all elements, including those marked for removal
            removed_.swap(tmp);
            list(lst);
            // check if every element marked for removal is also in the tree
            for (const auto &elt : tmp)
            {
                unsigned int i;
                for (i = 0; i < lst.size(); ++i)
                    if (lst[i] == *elt)
                        break;
                if (i == lst.size())
                {
                    // an element marked for removal is not actually in the tree
                    std::cout << "***** FAIL!! ******\n" << *this << '\n';
                    for (const auto &l : lst)
                        std::cout << l << '\t';
                    std::cout << std::endl;
                }
                assert(i != lst.size());
            }
            // restore
            removed_.swap(tmp);
            // get elements in the tree with elements marked for removal purged from the list
            list(lst);
            if (lst.size() != size_)
                std::cout << "#########################################\n" << *this << std::endl;
            assert(lst.size() == size_);
        }

    protected:
        using GNAT = NearestNeighborsGNATNoThreadSafety<_T>;

        /// Return true iff data has been marked for removal.
        bool isRemoved(const _T &data) const
        {
            return !removed_.empty() && removed_.find(&data) != removed_.end();
        }
        /// \brief Return in nearQueue_ the k nearest neighbors of data.
        /// For k=1, return true if the nearest neighbor is a pivot.
        /// (which is important during removal; removing pivots is a
        /// special case).
        bool nearestKInternal(const _T &data, std::size_t k) const
        {
            bool isPivot;
            double dist;
            Node *node;

            tree_->distToPivot_ = NearestNeighbors<_T>::distFun_(data, tree_->pivot_);
            isPivot = tree_->insertNeighborK(nearQueue_, k, tree_->pivot_, data, tree_->distToPivot_);
            tree_->nearestK(*this, data, k, isPivot);
            while (!nodeQueue_.empty())
            {
                dist = nearQueue_.top().first;  // note the difference with nearestRInternal
                node = nodeQueue_.top();
                nodeQueue_.pop();
                if (nearQueue_.size() == k &&
                    (node->distToPivot_ > node->maxRadius_ + dist || node->distToPivot_ < node->minRadius_ - dist))
                    continue;
                node->nearestK(*this, data, k, isPivot);
            }
            return isPivot;
        }
        /// \brief Return in nearQueue_ the elements that are within distance radius of data.
        void nearestRInternal(const _T &data, double radius) const
        {
            double dist = radius;  // note the difference with nearestKInternal
            Node *node;

            tree_->insertNeighborR(nearQueue_, radius, tree_->pivot_,
                                   NearestNeighbors<_T>::distFun_(data, tree_->pivot_));
            tree_->nearestR(*this, data, radius);
            while (!nodeQueue_.empty())
            {
                node = nodeQueue_.top();
                nodeQueue_.pop();
                if (node->distToPivot_ > node->maxRadius_ + dist || node->distToPivot_ < node->minRadius_ - dist)
                    continue;
                node->nearestR(*this, data, radius);
            }
        }
        /// \brief Convert the internal data structure used for storing neighbors
        /// to the vector that NearestNeighbor API requires.
        void postprocessNearest(std::vector<_T> &nbh) const
        {
            nbh.resize(nearQueue_.size());
            for (auto it = nbh.rbegin(); it != nbh.rend(); it++, nearQueue_.pop())
                *it = *nearQueue_.top().second;
        }

        /// The class used internally to define the GNAT.
        class Node
        {
        public:
            /// \brief Construct a node of given degree with at most
            /// \e capacity data elements and with given pivot.
            Node(int degree, int capacity, _T pivot)
              : degree_(degree)
              , pivot_(std::move(pivot))
              , minRadius_(std::numeric_limits<double>::infinity())
              , maxRadius_(-minRadius_)
              , minRange_(degree, minRadius_)
              , maxRange_(degree, maxRadius_)
#ifdef GNAT_SAMPLER
              , subtreeSize_(1)
              , activity_(0)
#endif
            {
                // The "+1" is needed because we add an element before we check whether to split
                data_.reserve(capacity + 1);
            }

            ~Node()
            {
                for (auto &child : children_)
                    delete child;
            }

            /// \brief Update minRadius_ and maxRadius_, given that an element
            /// was added with distance dist to the pivot.
            void updateRadius(double dist)
            {
                if (minRadius_ > dist)
                    minRadius_ = dist;
#ifndef GNAT_SAMPLER
                if (maxRadius_ < dist)
                    maxRadius_ = dist;
#else
                if (maxRadius_ < dist)
                {
                    maxRadius_ = dist;
                    activity_ = 0;
                }
                else
                    activity_ = std::max(-32, activity_ - 1);
#endif
            }
            /// \brief Update minRange_[i] and maxRange_[i], given that an
            /// element was added to the i-th child of the parent that has
            /// distance dist to this Node's pivot.
            void updateRange(unsigned int i, double dist)
            {
                if (minRange_[i] > dist)
                    minRange_[i] = dist;
                if (maxRange_[i] < dist)
                    maxRange_[i] = dist;
            }
            /// Add an element to the tree rooted at this node.
            void add(GNAT &gnat, const _T &data)
            {
#ifdef GNAT_SAMPLER
                subtreeSize_++;
#endif
                if (children_.empty())
                {
                    data_.push_back(data);
                    gnat.size_++;
                    if (needToSplit(gnat))
                    {
                        if (!gnat.removed_.empty())
                            gnat.rebuildDataStructure();
                        else if (gnat.size_ >= gnat.rebuildSize_)
                        {
                            gnat.rebuildSize_ <<= 1;
                            gnat.rebuildDataStructure();
                        }
                        else
                            split(gnat);
                    }
                }
                else
                {
                    double minDist = children_[0]->distToPivot_ = gnat.distFun_(data, children_[0]->pivot_);
                    int minInd = 0;

                    for (unsigned int i = 1; i < children_.size(); ++i)
                        if ((children_[i]->distToPivot_ = gnat.distFun_(data, children_[i]->pivot_)) < minDist)
                        {
                            minDist = children_[i]->distToPivot_;
                            minInd = i;
                        }
                    for (unsigned int i = 0; i < children_.size(); ++i)
                        children_[i]->updateRange(minInd, children_[i]->distToPivot_);
                    children_[minInd]->updateRadius(minDist);
                    children_[minInd]->add(gnat, data);
                }
            }
            /// Return true iff the node needs to be split into child nodes.
            bool needToSplit(const GNAT &gnat) const
            {
                unsigned int sz = data_.size();
                return sz > gnat.maxNumPtsPerLeaf_ && sz > degree_;
            }
            /// \brief The split operation finds pivot elements for the child
            /// nodes and moves each data element of this node to the appropriate
            /// child node.
            void split(GNAT &gnat)
            {
                typename GreedyKCenters<_T>::Matrix &dists = gnat.distances_;
                std::vector<unsigned int> &pivots = gnat.pivots_;

                children_.reserve(degree_);
                gnat.pivotSelector_.kcenters(data_, degree_, pivots, dists);
                for (unsigned int &pivot : pivots)
                    children_.push_back(new Node(degree_, gnat.maxNumPtsPerLeaf_, data_[pivot]));
                degree_ = pivots.size();  // in case fewer than degree_ pivots were found
                for (unsigned int j = 0; j < data_.size(); ++j)
                {
                    unsigned int k = 0;
                    for (unsigned int i = 1; i < degree_; ++i)
                        if (dists(j, i) < dists(j, k))
                            k = i;
                    Node *child = children_[k];
                    if (j != pivots[k])
                    {
                        child->data_.push_back(data_[j]);
                        child->updateRadius(dists(j, k));
                    }
                    for (unsigned int i = 0; i < degree_; ++i)
                        children_[i]->updateRange(k, dists(j, i));
                }

                for (auto &child : children_)
                {
                    // make sure degree lies between minDegree_ and maxDegree_
                    child->degree_ =
                        std::min(std::max((unsigned int)((degree_ * child->data_.size()) / data_.size()),
                                          gnat.minDegree_),
                                 gnat.maxDegree_);
                    // singleton
                    if (child->minRadius_ >= std::numeric_limits<double>::infinity())
                        child->minRadius_ = child->maxRadius_ = 0.;
#ifdef GNAT_SAMPLER
                    // set subtree size
                    child->subtreeSize_ = child->data_.size() + 1;
#endif
                }
                // this does more than clear(); it also sets capacity to 0 and frees the memory
                std::vector<_T> tmp;
                data_.swap(tmp);
                // check if new leaves need to be split
                for (auto &child : children_)
                    if (child->needToSplit(gnat))
                        child->split(gnat);
            }

            /// Insert data in nbh if it is a near neighbor. Return true iff data was added to nbh.
            bool insertNeighborK(NearQueue &nbh, std::size_t k, const _T &data, const _T &key, double dist) const
            {
                if (nbh.size() < k)
                {
                    nbh.emplace(dist, &data);
                    return true;
                }
                if (dist < nbh.top().first || (dist < std::numeric_limits<double>::epsilon() && data == key))
                {
                    nbh.pop();
                    nbh.emplace(dist, &data);
                    return true;
                }
                return false;
            }

            /// \brief Compute the k nearest neighbors of data in the tree.
            /// For k=1, isPivot is true if the nearest neighbor is a pivot
            /// (which is important during removal; removing pivots is a
            /// special case).
            void nearestK(const GNAT &gnat, const _T &data, std::size_t k, bool &isPivot) const
            {
                NearQueue &nbh = gnat.nearQueue_;
                for (const auto &d : data_)
                    if (!gnat.isRemoved(d))
                    {
                        if (insertNeighborK(nbh, k, d, data, gnat.distFun_(data, d)))
                            isPivot = false;
                    }
                if (!children_.empty())
                {
                    double dist;
                    Node *child;
                    Permutation &permutation = gnat.permutation_;
                    permutation.permute(children_.size());

                    for (unsigned int i = 0; i < children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            child->distToPivot_ = gnat.distFun_(data, child->pivot_);
                            if (insertNeighborK(nbh, k, child->pivot_, data, child->distToPivot_))
                                isPivot = true;
                            if (nbh.size() == k)
                            {
                                dist = nbh.top().first;  // note difference with nearestR
                                for (unsigned int j = 0; j < children_.size(); ++j)
                                    if (permutation[j] >= 0 && i != j &&
                                        (child->distToPivot_ - dist > child->maxRange_[permutation[j]] ||
                                         child->distToPivot_ + dist < child->minRange_[permutation[j]]))
                                        permutation[j] = -1;
                            }
                        }

                    dist = nbh.top().first;
                    for (unsigned int i = 0; i < children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            if (nbh.size() < k || (child->distToPivot_ - dist <= child->maxRadius_ &&
                                                   child->distToPivot_ + dist >= child->minRadius_))
                                gnat.nodeQueue_.push(child);
                        }
                }
            }
            /// Insert data in nbh if it is a near neighbor.
            void insertNeighborR(NearQueue &nbh, double r, const _T &data, double dist) const
            {
                if (dist <= r)
                    nbh.emplace(dist, &data);
            }
            /// \brief Return all elements that are within distance r in nbh.
            void nearestR(const GNAT &gnat, const _T &data, double r) const
            {
                NearQueue &nbh = gnat.nearQueue_;
                double dist = r;  // note difference with nearestK

                for (const auto &d : data_)
                    if (!gnat.isRemoved(d))
                        insertNeighborR(nbh, r, d, gnat.distFun_(data, d));
                if (!children_.empty())
                {
                    Node *child;
                    Permutation &permutation = gnat.permutation_;
                    permutation.permute(children_.size());

                    for (unsigned int i = 0; i < children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            child->distToPivot_ = gnat.distFun_(data, child->pivot_);
                            insertNeighborR(nbh, r, child->pivot_, child->distToPivot_);
                            for (unsigned int j = 0; j < children_.size(); ++j)
                                if (permutation[j] >= 0 && i != j &&
                                    (child->distToPivot_ - dist > child->maxRange_[permutation[j]] ||
                                     child->distToPivot_ + dist < child->minRange_[permutation[j]]))
                                    permutation[j] = -1;
                        }

                    for (unsigned int i = 0; i < children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            if (child->distToPivot_ - dist <= child->maxRadius_ &&
                                child->distToPivot_ + dist >= child->minRadius_)
                                gnat.nodeQueue_.push(child);
                        }
                }
            }

#ifdef GNAT_SAMPLER
            double getSamplingWeight(const GNAT &gnat) const
            {
                double minR = std::numeric_limits<double>::max();
                for (auto minRange : minRange_)
                    if (minRange < minR && minRange > 0.0)
                        minR = minRange;
                minR = std::max(minR, maxRadius_);
                return std::pow(minR, gnat.estimatedDimension_) / (double)subtreeSize_;
            }
            const _T &sample(const GNAT &gnat, RNG &rng) const
            {
                if (children_.size() != 0)
                {
                    if (rng.uniform01() < 1. / (double)subtreeSize_)
                        return pivot_;
                    PDF<const Node *> distribution;
                    for (const auto &child : children_)
                        distribution.add(child, child->getSamplingWeight(gnat));
                    return distribution.sample(rng.uniform01())->sample(gnat, rng);
                }
                else
                {
                    unsigned int i = rng.uniformInt(0, data_.size());
                    return (i == data_.size()) ? pivot_ : data_[i];
                }
            }
#endif

            void list(const GNAT &gnat, std::vector<_T> &data) const
            {
                if (!gnat.isRemoved(pivot_))
                    data.push_back(pivot_);
                for (const auto &d : data_)
                    if (!gnat.isRemoved(d))
                        data.push_back(d);
                for (const auto &child : children_)
                    child->list(gnat, data);
            }

            friend std::ostream &operator<<(std::ostream &out, const Node &node)
            {
                out << "\ndegree:\t" << node.degree_;
                out << "\nminRadius:\t" << node.minRadius_;
                out << "\nmaxRadius:\t" << node.maxRadius_;
                out << "\nminRange:\t";
                for (auto minR : node.minRange_)
                    out << minR << '\t';
                out << "\nmaxRange: ";
                for (auto maxR : node.maxRange_)
                    out << maxR << '\t';
                out << "\npivot:\t" << node.pivot_;
                out << "\ndata: ";
                for (auto &data : node.data_)
                    out << data << '\t';
                out << "\nthis:\t" << &node;
#ifdef GNAT_SAMPLER
                out << "\nsubtree size:\t" << node.subtreeSize_;
                out << "\nactivity:\t" << node.activity_;
#endif
                out << "\nchildren:\n";
                for (auto &child : node.children_)
                    out << child << '\t';
                out << '\n';
                for (auto &child : node.children_)
                    out << *child << '\n';
                return out;
            }

            /// Number of child nodes
            unsigned int degree_;
            /// Data element stored in this Node
            const _T pivot_;
            /// Minimum distance between the pivot element and the elements stored in data_
            double minRadius_;
            /// Maximum distance between the pivot element and the elements stored in data_
            double maxRadius_;
            /// \brief The i-th element in minRange_ is the minimum distance between the
            /// pivot and any data_ element in the i-th child node of this node's parent.
            std::vector<double> minRange_;
            /// \brief The i-th element in maxRange_ is the maximum distance between the
            /// pivot and any data_ element in the i-th child node of this node's parent.
            std::vector<double> maxRange_;
            /// \brief The data elements stored in this node (in addition to the pivot
            /// element). An internal node has no elements stored in data_.
            std::vector<_T> data_;
            /// \brief The child nodes of this node. By definition, only internal nodes
            /// have child nodes.
            std::vector<Node *> children_;

            /// \brief Scratch space to store distance to pivot during nearest neighbor queries
            mutable double distToPivot_;

#ifdef GNAT_SAMPLER
            /// Number of elements stored in the subtree rooted at this Node
            unsigned int subtreeSize_;
            /// \brief The extent to which a Node's maxRadius_ is increasing. A value of 0
            /// means the Node's maxRadius_ was increased the last time an element was added,
            /// while a negative value i means the Node hasn't expanded the last -i times
            /// an element was added.
            int activity_;
#endif
        };

        /// \cond IGNORE
        // another internal data structure is a priority queue of nodes to
        // check next for possible nearest neighbors
        struct NodeCompare
        {
            bool operator()(const Node *n0, const Node *n1) const
            {
                return (n0->distToPivot_ - n0->maxRadius_) > (n1->distToPivot_ - n1->maxRadius_);
            }
        };
        using NodeQueue = std::priority_queue<Node *, std::vector<Node *>, NodeCompare>;
        /// \endcond

        /// \brief The data structure containing the elements stored in this structure.
        Node *tree_{nullptr};
        /// The desired degree of each node.
        unsigned int degree_;
        /// \brief After splitting a Node, each child Node has degree equal to
        /// the default degree times the fraction of data elements from the
        /// original node that got assigned to that child Node. However, its
        /// degree can be no less than minDegree_.
        unsigned int minDegree_;
        /// \brief After splitting a Node, each child Node has degree equal to
        /// the default degree times the fraction of data elements from the
        /// original node that got assigned to that child Node. However, its
        /// degree can be no larger than maxDegree_.
        unsigned int maxDegree_;
        /// \brief Maximum number of elements allowed to be stored in a Node before
        /// it needs to be split into several nodes.
        unsigned int maxNumPtsPerLeaf_;
        /// \brief Number of elements stored in the tree.
        std::size_t size_{0};
        /// \brief If size_ exceeds rebuildSize_, the tree will be rebuilt (and
        /// automatically rebalanced), and rebuildSize_ will be doubled.
        std::size_t rebuildSize_;
        /// \brief Maximum number of removed elements that can be stored in the
        /// removed_ cache. If the cache is full, the tree will be rebuilt with
        /// the elements in removed_ actually removed from the tree.
        std::size_t removedCacheSize_;
        /// \brief The data structure used to split data into subtrees.
        GreedyKCenters<_T> pivotSelector_;
        /// \brief Cache of removed elements.
        std::unordered_set<const _T *> removed_;

        /// \name Internal scratch space
        /// \{
        /// \brief Internal data structure to store nearest neighbors
        mutable NearQueue nearQueue_;
        /// \brief Nodes yet to be processed for possible nearest neighbors
        mutable NodeQueue nodeQueue_;
        /// \brief Permutation of indices to process children of a node in random order
        mutable Permutation permutation_;
        /// \brief Pivot indices within a vector of elements as selected by GreedyKCenters
        mutable std::vector<unsigned int> pivots_;
        /// \brief Matrix of distances to pivots
        mutable typename GreedyKCenters<_T>::Matrix distances_;
        /// \}

#ifdef GNAT_SAMPLER
        /// \brief Estimated dimension of the local free space.
        double estimatedDimension_;
#endif
    };
}

#endif
