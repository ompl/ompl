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

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_GNAT_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_GNAT_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/GreedyKCenters.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <algorithm>
#include <boost/range/algorithm_ext/iota.hpp>

namespace ompl
{

    /** \brief Geometric Near-neighbor Access Tree (GNAT), a data
        structure for nearest neighbor search.

        See:
        S. Brin, “Near neighbor search in large metric spaces,” in Proc. 21st
        Conf. on Very Large Databases (VLDB), pp. 574–584, 1995.

    */
    template<typename _T>
    class NearestNeighborsGNAT : public NearestNeighbors<_T>
    {
    protected:
        // internally, we use a priority queue for nearest neighbors, paired
        // with their distance to the query point
        typedef std::pair<_T,double> DataDist;
        struct DataDistFun
        {
            bool operator()(const DataDist& d0, const DataDist& d1)
            {
                return d0.second < d1.second;
            }
        };
        typedef std::priority_queue<DataDist, std::vector<DataDist>, DataDistFun> NearQueue;

    public:
        NearestNeighborsGNAT(unsigned int degree = 4, unsigned int minDegree = 2,
                unsigned int maxDegree = 6, unsigned int maxNumPtsPerLeaf = 50,
                unsigned int removedCacheSize = 50)
                : NearestNeighbors<_T>(), tree_(NULL), degree_(degree),
                minDegree_(std::min(degree,minDegree)), maxDegree_(std::max(maxDegree,degree)),
                maxNumPtsPerLeaf_(maxNumPtsPerLeaf), size_(0)
        {
            removed.reserve(removedCacheSize);
        }

        virtual ~NearestNeighborsGNAT(void)
        {
            delete tree_;
        }

        /** \brief Set the distance function to use */
        virtual void setDistanceFunction(const typename NearestNeighbors<_T>::DistanceFunction &distFun)
        {
            NearestNeighbors<_T>::setDistanceFunction(distFun);
            pivotSelector_.setDistanceFunction(distFun);
        }

        virtual void clear(void)
        {
            if (tree_) delete tree_;
            tree_ = NULL;
        }

        virtual void add(const _T &data)
        {
            if (tree_)
                tree_->add(*this, data);
            else
                tree_ = new Node(NULL, degree_, data);
            size_++;
        }

        /** \brief Add a vector of points */
        virtual void add(const std::vector<_T> &data)
        {
            if (tree_)
                NearestNeighbors<_T>::add(data);
            else if (data.size()>0)
            {
                tree_ = new Node(NULL, degree_, data[0]);
                for (unsigned int i=1; i<data.size(); ++i)
                    tree_->data_.push_back(data[i]);
                if (tree_->needToSplit(*this))
                    tree_->split(*this);
            }
            size_ += data.size();
        }
        virtual bool remove(const _T &data)
        {
            if (!tree_) return false;
            if (tree_->find(data))
            {
                // if capacity of removed elements has been reached, we rebuild
                // the entire GNAT
                if (removed.size()==removed.capacity())
                {
                    std::vector<_T> lst;
                    typename std::vector<_T>::iterator it;

                    list(lst);
                    for (it = lst.begin(); it != lst.end(); it++)
                        if (*it == data)
                        {
                            lst.erase(it);
                            break;
                        }
                    delete tree_;
                    size_ = 0;
                    removed.clear();
                    add(lst);
                }
                else
                    size_--;
            }
            return false;
        }

        virtual _T nearest(const _T &data) const
        {
            if (tree_)
            {
                std::vector<_T> nbh;
                nearestK(data, 1, nbh);
                return nbh[0];
            }
            throw Exception("No elements found");
        }

        virtual void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const
        {
            nbh.clear();

            if (tree_)
            {
                double dist;
                Node* child;
                NearQueue nbh_queue;

                tree_->distToPivot_ = distFun_(data, tree_->pivot_);
                tree_->insertNeighborK(nbh_queue, k, tree_->pivot_, tree_->distToPivot_);
                tree_->nearestK(*this, data, k + removed.size(), nbh_queue);
                while (queue_.size() > 0)
                {
                    dist = nbh_queue.top().second;
                    child = queue_.top();
                    queue_.pop();
                    if (nbh_queue.size() == k &&
                        (child->distToPivot_ > child->maxRadius_ + dist ||
                        child->distToPivot_ < child->minRadius_ - dist))
                        break;
                    child->nearestK(*this, data, k, nbh_queue);
                }
                postprocessNearest(nbh_queue, nbh);
            }
        }

        virtual void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const
        {
            assert(queue_.size()==0);
            nbh.clear();
            if (tree_)
            {
                double dist = radius;
                Node* child;
                NearQueue nbh_queue;

                tree_->distToPivot_ = distFun_(data, tree_->pivot_);
                tree_->insertNeighborR(nbh_queue, radius, tree_->pivot_, tree_->distToPivot_);
                tree_->nearestR(*this, data, radius, nbh_queue);
                while (queue_.size() > 0)
                {
                    child = queue_.top();
                    queue_.pop();
                    if ((child->distToPivot_ > child->maxRadius_ + dist ||
                        child->distToPivot_ < child->minRadius_ - dist))
                        break;
                    child->nearestR(*this, data, radius, nbh_queue);
                }
                postprocessNearest(nbh_queue, nbh);
            }
        }

        virtual std::size_t size(void) const
        {
            return size_;
        }

        virtual void list(std::vector<_T> &data) const
        {
            data.clear();
            data.reserve(size());
            if (tree_)
                tree_->list(data);
        }

        friend std::ostream& operator<<(std::ostream& out, const NearestNeighborsGNAT<_T>& gnat)
        {
            return gnat.tree_ ? (out << *gnat.tree_) : out;
        }

    protected:
        typedef NearestNeighborsGNAT<_T> GNAT;

        void postprocessNearest(NearQueue& nbh_queue, std::vector<_T> &nbh) const
        {
            while(queue_.size() > 0)
                queue_.pop();

            if (removed.size()>0)
            {
                unsigned int i;
                while (nbh_queue.size()>0)
                {
                    const _T& elt = nbh_queue.top().first;
                    for (i=0; i<removed.size(); ++i)
                        if (removed[i] == elt)
                            break;
                    if (i==removed.size())
                        nbh.push_back(elt);
                    nbh_queue.pop();
                }
            }
            else
            {
                typename std::vector<_T>::reverse_iterator it;
                nbh.resize(nbh_queue.size());
                for (it=nbh.rbegin(); it!=nbh.rend(); it++, nbh_queue.pop())
                    *it = nbh_queue.top().first;
            }
        }

        class Node
        {
        public:
            Node(const Node* parent, int degree, const _T& pivot)
                : parent_(parent), degree_(degree), pivot_(pivot),
                minRadius_(std::numeric_limits<double>::infinity()),
                maxRadius_(-minRadius_), minRange_(degree, minRadius_),
                maxRange_(degree, maxRadius_), distToPivot_(0.)
            {
            }

            ~Node()
            {
                for (unsigned int i=0; i<children_.size(); ++i)
                    delete children_[i];
            }

            void add(GNAT& gnat, const _T& data)
            {
                if (children_.size()==0)
                {
                    data_.push_back(data);
                    if (needToSplit(gnat))
                        split(gnat);
                }
                else
                {
                    double minDist = std::numeric_limits<double>::infinity();
                    int minInd = -1;

                    for (unsigned int i=0; i<children_.size(); ++i)
                    {
                        if ((children_[i]->distToPivot_ = gnat.distFun_(data, children_[i]->pivot_)) < minDist)
                        {
                            minDist = children_[i]->distToPivot_;
                            minInd = i;
                        }
                    }
                    if (minDist < children_[minInd]->minRadius_)
                        children_[minInd]->minRadius_ = minDist;
                    if (minDist > children_[minInd]->maxRadius_)
                        children_[minInd]->maxRadius_ = minDist;

                    for (unsigned int i=0; i<children_.size(); ++i)
                    {
                        if (children_[i]->minRange_[minInd] > children_[i]->distToPivot_)
                            children_[i]->minRange_[minInd] = children_[i]->distToPivot_;
                        if (children_[i]->maxRange_[minInd] < children_[i]->distToPivot_)
                            children_[i]->maxRange_[minInd] = children_[i]->distToPivot_;
                    }
                    children_[minInd]->add(gnat, data);
                }
            }

            bool needToSplit(const GNAT& gnat) const
            {
                unsigned int sz = data_.size();
                return sz > gnat.maxNumPtsPerLeaf_ && sz > degree_;
            }
            void split(GNAT& gnat)
            {
                std::vector<std::vector<double> > dists;
                std::vector<unsigned int> pivots;
                unsigned int i, j, k, cdegree;
                Node* child;

                children_.reserve(degree_);
                gnat.pivotSelector_.kcenters(data_, degree_, pivots, dists);
                for(i=0; i<pivots.size(); i++)
                    children_.push_back(new Node(this, degree_, data_[pivots[i]]));
                degree_ = pivots.size(); // in case fewer than degree_ pivots were found
                for (j=0; j<data_.size(); ++j)
                {
                    for (k=0, i=1; i<degree_; ++i)
                        if (dists[j][i] < dists[j][k])
                            k = i;
                    child = children_[k];
                    if (j != pivots[k])
                    {
                        child->data_.push_back(data_[j]);
                        if (dists[j][k] > child->maxRadius_)
                            child->maxRadius_ = dists[j][k];
                        if (dists[j][k] < child->minRadius_)
                            child->minRadius_ = dists[j][k];
                    }
                    for (i=0; i<degree_; ++i)
                    {
                        if (children_[i]->minRange_[k] > dists[j][i])
                            children_[i]->minRange_[k] = dists[j][i];
                        if (children_[i]->maxRange_[k] < dists[j][i])
                            children_[i]->maxRange_[k] = dists[j][i];
                    }
                }

                for (i=0; i<degree_; ++i)
                {
                    // make sure degree lies between minDegree_ and maxDegree_
                    children_[i]->degree_ = std::min(std::max(
                        degree_ * (unsigned int)(children_[i]->data_.size() / data_.size()),
                        gnat.minDegree_), gnat.maxDegree_);
                    // singleton
                    if (children_[i]->minRadius_ == std::numeric_limits<double>::infinity())
                        children_[i]->minRadius_ = children_[i]->maxRadius_ = 0.;
                }
                data_.clear();
                // check if new leaves need to be split
                for (i=0; i<degree_; ++i)
                    if (children_[i]->needToSplit(gnat))
                        children_[i]->split(gnat);
            }

            bool find(const _T& data) const
            {
                if (pivot_ == data)
                    return true;
                for (unsigned int i=0; i<data_.size(); ++i)
                    if (data_[i] == data)
                        return true;
                for (unsigned int i=0; i<children_.size(); ++i)
                    if (children_[i]->find(data))
                        return true;
                return false;
            }

            void insertNeighborK(NearQueue& nbh, std::size_t k, const _T& data, double dist) const
            {
                if (nbh.size() < k)
                    nbh.push(std::make_pair(data, dist));
                else if (dist < nbh.top().second)
                {
                    nbh.pop();
                    nbh.push(std::make_pair(data, dist));
                }
            }

            void nearestK(const GNAT& gnat, const _T &data, std::size_t k, NearQueue& nbh) const
            {
                unsigned int i, j;
                double dist;

                for (i=0; i<data_.size(); ++i)
                    insertNeighborK(nbh, k, data_[i], gnat.distFun_(data, data_[i]));
                if (children_.size() > 0)
                {
                    Node* child;
                    std::vector<int> permutation(children_.size());
                    boost::range::iota(permutation, 0);
                    std::random_shuffle(permutation.begin(), permutation.end());

                    for (i=0; i<children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            child->distToPivot_ = gnat.distFun_(data, child->pivot_);
                            insertNeighborK(nbh, k, child->pivot_, child->distToPivot_);
                            if (nbh.size()==k)
                            {
                                dist = nbh.top().second;
                                for (j=0; j<children_.size(); ++j)
                                    if (permutation[j] >=0 && i != j &&
                                        (child->distToPivot_ - dist > child->maxRange_[permutation[j]] ||
                                        child->distToPivot_ + dist < child->minRange_[permutation[j]]))
                                        permutation[j] = -1;
                            }
                        }

                    dist = nbh.top().second;
                    for (i=0; i<children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            if (nbh.size()<k ||
                                (child->distToPivot_ <= (child->maxRadius_ + dist) &&
                                child->distToPivot_ >= (child->minRadius_ - dist)))
                                gnat.queue_.push(child);
                        }
                }
            }

            void insertNeighborR(NearQueue& nbh, double r, const _T& data, double dist) const
            {
                if (dist < r)
                    nbh.push(std::make_pair(data, dist));
            }

            void nearestR(const GNAT& gnat, const _T &data, double r, NearQueue& nbh) const
            {
                unsigned int i, j;
                double dist = r;

                for (i=0; i<data_.size(); ++i)
                    insertNeighborR(nbh, r, data_[i], gnat.distFun_(data, data_[i]));
                if (children_.size() > 0)
                {
                    Node* child;
                    std::vector<int> permutation(children_.size());
                    boost::range::iota(permutation, 0);
                    std::random_shuffle(permutation.begin(), permutation.end());

                    for (i=0,j=permutation.size();i<permutation.size();++i) j+=permutation[i];
                    assert(j=(permutation.size()+1)*permutation.size()/2);

                    for (i=0; i<children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            child->distToPivot_ = gnat.distFun_(data, child->pivot_);
                            insertNeighborR(nbh, r, child->pivot_, child->distToPivot_);
                            for (j=0; j<children_.size(); ++j)
                                if (permutation[j] >=0 && i != j &&
                                (child->distToPivot_ - dist > child->maxRange_[permutation[j]] ||
                                child->distToPivot_ + dist < child->minRange_[permutation[j]]))
                                permutation[j] = -1;
                        }

                    for (i=0; i<children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            if ((child->distToPivot_ <= (child->maxRadius_ + dist) &&
                                child->distToPivot_ >= (child->minRadius_ - dist)))
                                gnat.queue_.push(child);
                        }
                }
            }

            void list(std::vector<_T> &data) const
            {
                data.push_back(pivot_);
                for (unsigned int i=0; i<data_.size(); ++i)
                    data.push_back(data_[i]);
                for (unsigned int i=0; i<children_.size(); ++i)
                    children_[i]->list(data);
            }

            friend std::ostream& operator<<(std::ostream& out, const Node& node)
            {
                unsigned int i;
                out << "\ndegree:\t" << node.degree_;
                out << "\nminRadius:\t" << node.minRadius_;
                out << "\nmaxRadius:\t" << node.maxRadius_;
                out << "\nminRange:\t";
                for (i=0; i<node.minRange_.size(); ++i) out << node.minRange_[i] << '\t';
                out << "\nmaxRange: ";
                for (i=0; i<node.maxRange_.size(); ++i) out << node.maxRange_[i] << '\t';
                out << "\npivot:\t" << node.pivot_;
                out << "\ndata: ";
                for (i=0; i<node.data_.size(); ++i) out << node.data_[i] << '\t';
                out << "\nthis:\t" << &node << "\nparent:\t" << node.parent_;
                out << "\nchildren:\n";
                for (i=0; i<node.children_.size(); ++i) out << node.children_[i] << '\t';
                out << '\n';
                for (i=0; i<node.children_.size(); ++i) out << *node.children_[i] << '\n';
                return out;
            }

            const Node* parent_;
            unsigned int degree_;
            const _T pivot_;
            double minRadius_;
            double maxRadius_;
            std::vector<double> minRange_;
            std::vector<double> maxRange_;
            double distToPivot_;
            std::vector<_T> data_;
            std::vector<Node*> children_;
        };

        struct NodeCompare
        {
            bool operator()(const Node* n0, const Node* n1) const
            {
                return (n0->distToPivot_ - n0->maxRadius_) > (n1->distToPivot_ - n1->maxRadius_);
            }
        };

        /** \brief The data elements stored in this structure */
        Node* tree_;

        unsigned int degree_;
        unsigned int minDegree_;
        unsigned int maxDegree_;
        unsigned int maxNumPtsPerLeaf_;
        unsigned int size_;

        /** \brief The data structure used to split data into subtrees */
        GreedyKCenters<_T> pivotSelector_;

        /** \brief Queue of nodes to be checked for nearest neighbors */
        mutable std::priority_queue<Node*, std::vector<Node*>, NodeCompare> queue_;

        /** \brief Cache of removed elements */
        std::vector<_T> removed;

    };

}

#endif
