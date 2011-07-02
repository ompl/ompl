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
        struct DataDistCompare
        {
            bool operator()(const DataDist& d0, const DataDist& d1)
            {
                return d0.second < d1.second;
            }
        };
        typedef std::priority_queue<DataDist, std::vector<DataDist>, DataDistCompare> NearQueue;

        // another internal data structure is a priority queue of nodes to
        // check next for possible nearest neighbors
        class Node;
        typedef std::pair<Node*,double> NodeDist;
        struct NodeDistCompare
        {
            bool operator()(const NodeDist& n0, const NodeDist& n1) const
            {
                return (n0.second - n0.first->maxRadius_) > (n1.second - n1.first->maxRadius_);
            }
        };
        typedef std::priority_queue<NodeDist, std::vector<NodeDist>, NodeDistCompare> NodeQueue;


    public:
        NearestNeighborsGNAT(unsigned int degree = 4, unsigned int minDegree = 2,
            unsigned int maxDegree = 6, unsigned int maxNumPtsPerLeaf = 50,
            unsigned int removedCacheSize = 50)
            : NearestNeighbors<_T>(), tree_(NULL), degree_(degree),
            minDegree_(std::min(degree,minDegree)), maxDegree_(std::max(maxDegree,degree)),
            maxNumPtsPerLeaf_(maxNumPtsPerLeaf), size_(0)
        {
            removed_.reserve(removedCacheSize);
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
                removed_.push_back(data);
                // if capacity of removed elements has been reached, we rebuild
                // the entire GNAT
                if (removed_.size()==removed_.capacity())
                {
                    std::vector<_T> lst;
                    typename std::vector<_T>::iterator it;

                    list(lst);
                    while (removed_.size() > 0)
                    {
                        for (it = lst.begin(); it != lst.end(); it++)
                            if (*it == removed_.back())
                                break;
                        assert(it != lst.end());
                        lst.erase(it);
                        removed_.pop_back();
                    }
                    delete tree_;
                    tree_ = NULL;
                    size_ = 0;
                    add(lst);
                }
                else
                    size_--;
                return true;
            }
            return false;
        }

        virtual _T nearest(const _T &data) const
        {
            if (tree_)
            {
                std::vector<_T> nbh;
                nearestK(data, 1, nbh);
                if (!nbh.empty()) return nbh[0];
            }
            throw Exception("No elements found");
        }

        virtual void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const
        {
            nbh.clear();

            if (k == 0) return;

            if (tree_)
            {
                double dist;
                NearQueue nbhQueue;
                NodeQueue nodeQueue;
                NodeDist nodeDist;

                tree_->insertNeighborK(nbhQueue, k, tree_->pivot_,
                    NearestNeighbors<_T>::distFun_(data, tree_->pivot_));
                tree_->nearestK(*this, data, k + removed_.size(), nbhQueue, nodeQueue);
                while (nodeQueue.size() > 0)
                {
                    dist = nbhQueue.top().second; // note the difference with nearestR
                    nodeDist = nodeQueue.top();
                    nodeQueue.pop();
                    if (nbhQueue.size() == k &&
                        (nodeDist.second > nodeDist.first->maxRadius_ + dist ||
                         nodeDist.second < nodeDist.first->minRadius_ - dist))
                        break;
                    nodeDist.first->nearestK(*this, data, k + removed_.size(), nbhQueue, nodeQueue);
                }
                postprocessNearest(nbhQueue, nbh, k);
            }
        }

        virtual void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const
        {
            nbh.clear();

            if (tree_)
            {
                double dist = radius; // note the difference with nearestK
                NearQueue nbhQueue;
                NodeQueue nodeQueue;
                NodeDist nodeDist;

                tree_->insertNeighborR(nbhQueue, radius, tree_->pivot_,
                    NearestNeighbors<_T>::distFun_(data, tree_->pivot_));
                tree_->nearestR(*this, data, radius, nbhQueue, nodeQueue);
                while (nodeQueue.size() > 0)
                {
                    nodeDist = nodeQueue.top();
                    nodeQueue.pop();
                    if (nodeDist.second > nodeDist.first->maxRadius_ + dist ||
                        nodeDist.second < nodeDist.first->minRadius_ - dist)
                        break;
                    nodeDist.first->nearestR(*this, data, radius, nbhQueue, nodeQueue);
                }
                postprocessNearest(nbhQueue, nbh);
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

        void postprocessNearest(NearQueue& nbhQueue, std::vector<_T> &nbh,
            unsigned int k=std::numeric_limits<unsigned int>::infinity()) const
        {
            if (removed_.size()>0)
            {
                while (nbhQueue.size()>0 && nbh.size()<k)
                {
                    unsigned int i;
                    const _T& elt = nbhQueue.top().first;
                    for (i=0; i<removed_.size(); ++i)
                        if (removed_[i] == elt)
                            break;
                    if (i==removed_.size())
                        nbh.push_back(elt);
                    nbhQueue.pop();
                }
            }
            else
            {
                typename std::vector<_T>::reverse_iterator it;
                nbh.resize(nbhQueue.size());
                for (it=nbh.rbegin(); it!=nbh.rend(); it++, nbhQueue.pop())
                    *it = nbhQueue.top().first;
            }
        }

        class Node
        {
        public:
            Node(const Node* parent, int degree, const _T& pivot)
                : degree_(degree), pivot_(pivot),
                minRadius_(std::numeric_limits<double>::infinity()),
                maxRadius_(-minRadius_), minRange_(degree, minRadius_),
                maxRange_(degree, maxRadius_)
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
                        double dist;

                        if ((dist = gnat.distFun_(data, children_[i]->pivot_)) < minDist)
                        {
                            minDist = dist;
                            minInd = i;
                        }
                        if (children_[i]->minRange_[minInd] > dist)
                            children_[i]->minRange_[minInd] = dist;
                        if (children_[i]->maxRange_[minInd] < dist)
                            children_[i]->maxRange_[minInd] = dist;
                    }
                    if (minDist < children_[minInd]->minRadius_)
                        children_[minInd]->minRadius_ = minDist;
                    if (minDist > children_[minInd]->maxRadius_)
                        children_[minInd]->maxRadius_ = minDist;

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

                children_.reserve(degree_);
                gnat.pivotSelector_.kcenters(data_, degree_, pivots, dists);
                for(unsigned int i=0; i<pivots.size(); i++)
                    children_.push_back(new Node(this, degree_, data_[pivots[i]]));
                degree_ = pivots.size(); // in case fewer than degree_ pivots were found
                for (unsigned int j=0; j<data_.size(); ++j)
                {
                    unsigned int k = 0;
                    for (unsigned int i=1; i<degree_; ++i)
                        if (dists[j][i] < dists[j][k])
                            k = i;
                    Node* child = children_[k];
                    if (j != pivots[k])
                    {
                        child->data_.push_back(data_[j]);
                        if (dists[j][k] > child->maxRadius_)
                            child->maxRadius_ = dists[j][k];
                        if (dists[j][k] < child->minRadius_)
                            child->minRadius_ = dists[j][k];
                    }
                    for (unsigned int i=0; i<degree_; ++i)
                    {
                        if (children_[i]->minRange_[k] > dists[j][i])
                            children_[i]->minRange_[k] = dists[j][i];
                        if (children_[i]->maxRange_[k] < dists[j][i])
                            children_[i]->maxRange_[k] = dists[j][i];
                    }
                }

                for (unsigned int i=0; i<degree_; ++i)
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
                for (unsigned int i=0; i<degree_; ++i)
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

            void nearestK(const GNAT& gnat, const _T &data, std::size_t k, NearQueue& nbh, NodeQueue& nodeQueue) const
            {
                for (unsigned int i=0; i<data_.size(); ++i)
                    insertNeighborK(nbh, k, data_[i], gnat.distFun_(data, data_[i]));
                if (children_.size() > 0)
                {
                    double dist;
                    Node* child;
                    std::vector<double> distToPivot(children_.size());
                    std::vector<int> permutation(children_.size());
                    boost::range::iota(permutation, 0);
                    std::random_shuffle(permutation.begin(), permutation.end());

                    for (unsigned int i=0; i<children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            distToPivot[permutation[i]] = gnat.distFun_(data, child->pivot_);
                            insertNeighborK(nbh, k, child->pivot_, distToPivot[permutation[i]]);
                            if (nbh.size()==k)
                            {
                                dist = nbh.top().second; // note difference with nearestR
                                for (unsigned int j=0; j<children_.size(); ++j)
                                    if (permutation[j] >=0 && i != j &&
                                        (distToPivot[permutation[i]] - dist > child->maxRange_[permutation[j]] ||
                                         distToPivot[permutation[i]] + dist < child->minRange_[permutation[j]]))
                                        permutation[j] = -1;
                            }
                        }

                    dist = nbh.top().second;
                    for (unsigned int i=0; i<children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            if (nbh.size()<k ||
                                (distToPivot[permutation[i]] <= (child->maxRadius_ + dist) &&
                                 distToPivot[permutation[i]] >= (child->minRadius_ - dist)))
                                nodeQueue.push(std::make_pair(child, distToPivot[permutation[i]]));
                        }
                }
            }

            void insertNeighborR(NearQueue& nbh, double r, const _T& data, double dist) const
            {
                if (dist < r)
                    nbh.push(std::make_pair(data, dist));
            }

            void nearestR(const GNAT& gnat, const _T &data, double r, NearQueue& nbh, NodeQueue& nodeQueue) const
            {
                double dist = r; //note difference with nearestK

                for (unsigned int i=0; i<data_.size(); ++i)
                    insertNeighborR(nbh, r, data_[i], gnat.distFun_(data, data_[i]));
                if (children_.size() > 0)
                {
                    Node* child;
                    std::vector<double> distToPivot(children_.size());
                    std::vector<int> permutation(children_.size());
                    boost::range::iota(permutation, 0);
                    std::random_shuffle(permutation.begin(), permutation.end());

                    for (unsigned int i=0; i<children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            distToPivot[i] = gnat.distFun_(data, child->pivot_);
                            insertNeighborR(nbh, r, child->pivot_, distToPivot[i]);
                            for (unsigned int j=0; j<children_.size(); ++j)
                                if (permutation[j] >=0 && i != j &&
                                    (distToPivot[i] - dist > child->maxRange_[permutation[j]] ||
                                     distToPivot[i] + dist < child->minRange_[permutation[j]]))
                                    permutation[j] = -1;
                        }

                    for (unsigned int i=0; i<children_.size(); ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            if ((distToPivot[i] <= (child->maxRadius_ + dist) &&
                                 distToPivot[i] >= (child->minRadius_ - dist)))
                                nodeQueue.push(std::make_pair(child, distToPivot[i]));
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
                out << "\ndegree:\t" << node.degree_;
                out << "\nminRadius:\t" << node.minRadius_;
                out << "\nmaxRadius:\t" << node.maxRadius_;
                out << "\nminRange:\t";
                for (unsigned int i=0; i<node.minRange_.size(); ++i)
                    out << node.minRange_[i] << '\t';
                out << "\nmaxRange: ";
                for (unsigned int i=0; i<node.maxRange_.size(); ++i)
                    out << node.maxRange_[i] << '\t';
                out << "\npivot:\t" << node.pivot_;
                out << "\ndata: ";
                for (unsigned int i=0; i<node.data_.size(); ++i)
                    out << node.data_[i] << '\t';
                out << "\nthis:\t" << &node;
                out << "\nchildren:\n";
                for (unsigned int i=0; i<node.children_.size(); ++i)
                    out << node.children_[i] << '\t';
                out << '\n';
                for (unsigned int i=0; i<node.children_.size(); ++i)
                    out << *node.children_[i] << '\n';
                return out;
            }

            unsigned int degree_;
            const _T pivot_;
            double minRadius_;
            double maxRadius_;
            std::vector<double> minRange_;
            std::vector<double> maxRange_;
            std::vector<_T> data_;
            std::vector<Node*> children_;
        };


        /** \brief The data elements stored in this structure */
        Node* tree_;

        unsigned int degree_;
        unsigned int minDegree_;
        unsigned int maxDegree_;
        unsigned int maxNumPtsPerLeaf_;
        std::size_t size_;

        /** \brief The data structure used to split data into subtrees */
        GreedyKCenters<_T> pivotSelector_;

        /** \brief Cache of removed elements */
        std::vector<_T> removed_;

    };

}

#endif
