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
/* Sampler extension by Bryant Gipson */

#ifndef OMPL_DATASTRUCTURES_GNAT_SAMPLER_
#define OMPL_DATASTRUCTURES_GNAT_SAMPLER_

#include <ompl/util/Profiler.h>
#include <ompl/util/RandomNumbers.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/GreedyKCenters.h"
#include "ompl/util/Exception.h"
#include <boost/unordered_set.hpp>
#include <boost/bimap.hpp>
#include <queue>
#include <algorithm>

#ifndef doubleVect
typedef std::vector<double> doubleVect;
#endif
namespace ompl
{

template<typename _T>
class gnatSampler;

/** \brief Geometric Near-neighbor Access Tree (GNAT), a data
	structure for nearest neighbor search.

See:
S. Brin, “Near neighbor search in large metric spaces,” in Proc. 21st
Conf. on Very Large Databases (VLDB), pp. 574–584, 1995.

 */

template<typename _T>
class gnatSampler : public ompl::NearestNeighbors<_T>
{
	protected:
		// internally, we use a priority queue for nearest neighbors, paired
		// with their distance to the query point
		typedef std::pair<const _T*,double> DataDist;
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
		gnatSampler(unsigned int degree = 4, unsigned int minDegree = 2,
				unsigned int maxDegree = 6, unsigned int maxNumPtsPerLeaf = 50,
				unsigned int removedCacheSize = 50)
			: ompl::NearestNeighbors<_T>(), tree_(NULL), degree_(degree),
			minDegree_(std::min(degree,minDegree)), maxDegree_(std::max(maxDegree,degree)),
			maxNumPtsPerLeaf_(maxNumPtsPerLeaf), size_(0), removedCacheSize_(removedCacheSize)
	{
	}

		const _T& sample( double borderFraction = 0.0)
		{
      if(borderFraction > 0.5 && _dataLeaves.size() > 2)
      {
        ompl::Profiler::Begin("Check border distribution");
        std::vector<Node*> r2 = rng_.discreteDistributionMap<Node*>(_dataLeaves,1);
        ompl::Profiler::End("Check border distribution");
        return r2[0]->sample(*this);
      }
      else
      {
        return tree_->sample(*this);
      }
    }

    virtual ~gnatSampler(void)
    {
      if (tree_)
        delete tree_;
    }

    /** \brief Set the distance function to use */
    virtual void setDistanceFunction(const typename ompl::NearestNeighbors<_T>::DistanceFunction &distFun)
    {
      ompl::NearestNeighbors<_T>::setDistanceFunction(distFun);
      pivotSelector_.setDistanceFunction(distFun);
    }

    virtual void clear(void)
    {
      if (tree_)
      {
        delete tree_;
        tree_ = NULL;
      }
      size_ = 0;
      removed_.clear();
      _dataLeaves.clear();
    }

    virtual void add(const _T &data)
    {
      if (tree_)
      {
        tree_->add(*this, data);
        double N = (double)size() + 1;

        ompl::Profiler::Begin("GNAT - rebuildDataStructure");
        if(fabs(fmod(log2(N),2.0)) < 1e-8) 
        {
          rebuildDataStructure();
        }
        ompl::Profiler::End("GNAT - rebuildDataStructure");
      }
      else
      {
        tree_ = new Node(degree_, maxNumPtsPerLeaf_, data);
        size_ = 1;
      }
    }

    /** \brief Add a vector of points */
    virtual void add(const std::vector<_T> &data)
    {
      if (tree_)
        ompl::NearestNeighbors<_T>::add(data);
      else if (data.size()>0)
      {
        tree_ = new Node(degree_, maxNumPtsPerLeaf_, data[0]);
        for (unsigned int i=1; i<data.size(); ++i)
        {
          tree_->data_.push_back(data[i]);
          tree_->_totalChildNodes++;
          double r = distFun_(data[i],tree_->pivot_);
          if(r>tree_->_maxObservedRadius) tree_->_maxObservedRadius = r;
        }
        if (tree_->needToSplit(*this))
          tree_->split(*this);
      }
      size_ += data.size();
    }
    /** \brief Rebuild the internal data structure */
    void rebuildDataStructure()
    {
      std::vector<_T> lst;
      list(lst);
      clear();
      add(lst);
    }
    virtual bool remove(const _T &data)
    {
      if (!tree_) return false;
      NearQueue nbhQueue;
      // find data in tree
      bool isPivot = nearestKInternal(data, 1, nbhQueue);
      if (*nbhQueue.top().first != data)
        return false;
      removed_.insert(nbhQueue.top().first);
      size_--;
      // if we removed a pivot or if the capacity of removed elements
      // has been reached, we rebuild the entire GNAT
      if (isPivot || removed_.size()>=removedCacheSize_)
        rebuildDataStructure();
      return true;
    }
    virtual _T nearest(const _T &data) const
    {
      if (tree_)
      {
        std::vector<_T> nbh;
        nearestK(data, 1, nbh);
        if (!nbh.empty()) return nbh[0];
      }
      throw ompl::Exception("No elements found");
    }

    virtual void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const
    {
      nbh.clear();
      if (k == 0) return;
      if (tree_)
      {
        NearQueue nbhQueue;
        nearestKInternal(data, k, nbhQueue);
        postprocessNearest(nbhQueue, nbh);
      }
    }

    virtual void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const
    {
      nbh.clear();
      if (tree_)
      {
        NearQueue nbhQueue;
        nearestRInternal(data, radius, nbhQueue);
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
        tree_->list(*this, data);
    }

    friend std::ostream& operator<<(std::ostream& out, const gnatSampler<_T>& gnat)
    {
      if (gnat.tree_)
      {
        out << *gnat.tree_;
        if (!gnat.removed_.empty())
        {
          out << "Elements marked for removal:\n";
          for (typename boost::unordered_set<const _T*>::const_iterator it = gnat.removed_.begin();
              it != gnat.removed_.end(); it++)
            out << **it << '\t';
          out << std::endl;
        }
      }
      return out;
    }

    // for debugging purposes
    void integrityCheck()
    {
      std::vector<_T> lst;
      boost::unordered_set<const _T*> tmp;
      // get all elements, including those marked for removal
      removed_.swap(tmp);
      list(lst);
      // check if every element marked for removal is also in the tree
      for (typename boost::unordered_set<const _T*>::iterator it=tmp.begin(); it!=tmp.end(); it++)
      {
        unsigned int i;
        for (i=0; i<lst.size(); ++i)
          if (lst[i]==**it)
            break;
        if (i == lst.size())
        {
          // an element marked for removal is not actually in the tree
          std::cout << "***** FAIL!! ******\n" << *this << '\n';
          for (unsigned int j=0; j<lst.size(); ++j) std::cout<<lst[j]<<'\t';
          std::cout<<std::endl;
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
    typedef gnatSampler<_T> GNAT;

    bool isRemoved(const _T& data) const
    {
      return !removed_.empty() && removed_.find(&data) != removed_.end();
    }

    // for k=1, return true if the nearest neighbor is a pivot
    bool nearestKInternal(const _T &data, std::size_t k, NearQueue& nbhQueue) const
    {
      bool isPivot;
      double dist;
      NodeDist nodeDist;
      NodeQueue nodeQueue;

      isPivot = tree_->insertNeighborK(nbhQueue, k, tree_->pivot_, data,
          ompl::NearestNeighbors<_T>::distFun_(data, tree_->pivot_));
      tree_->nearestK(*this, data, k, nbhQueue, nodeQueue, isPivot);
      while (nodeQueue.size() > 0)
      {
        dist = nbhQueue.top().second; // note the difference with nearestRInternal
        nodeDist = nodeQueue.top();
        nodeQueue.pop();
        if (nbhQueue.size() == k &&
            (nodeDist.second > nodeDist.first->maxRadius_ + dist ||
             nodeDist.second < nodeDist.first->minRadius_ - dist))
          break;
        nodeDist.first->nearestK(*this, data, k, nbhQueue, nodeQueue, isPivot);
      }
      return isPivot;
    }
    void nearestRInternal(const _T &data, double radius, NearQueue& nbhQueue) const
    {
      double dist = radius; // note the difference with nearestKInternal
      NodeQueue nodeQueue;
      NodeDist nodeDist;

      tree_->insertNeighborR(nbhQueue, radius, tree_->pivot_,
          ompl::NearestNeighbors<_T>::distFun_(data, tree_->pivot_));
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
    }
    void postprocessNearest(NearQueue& nbhQueue, std::vector<_T> &nbh) const
    {
      typename std::vector<_T>::reverse_iterator it;
      nbh.resize(nbhQueue.size());
      for (it=nbh.rbegin(); it!=nbh.rend(); it++, nbhQueue.pop())
        *it = *nbhQueue.top().first;
    }

    class Node
    {
      public:
        Node(int degree, int capacity, const _T& pivot)
          : degree_(degree), pivot_(pivot),
          minRadius_(std::numeric_limits<double>::infinity()),
          maxRadius_(-minRadius_), minRange_(degree, minRadius_),
          maxRange_(degree, maxRadius_),
          _maxActivity(2.0),
          _previousObservedRadius(0.0),
          _deltaRadius(0.0),
          _activity(-100.0),
          _totalChildNodes(0),
          _maxObservedRadius(0.0)
      {
        // The "+1" is needed because we add an element before we check whether to split
        data_.reserve(capacity+1);

      }

        ~Node()
        {
          for (unsigned int i=0; i<children_.size(); ++i)
            delete children_[i];
        }

        double boundingRadius(GNAT &gnat, const _T&data, bool isPivot)
        {
          NearQueue nbh;
          NodeQueue nQ;
          nearestK(gnat, data, 2, nbh, nQ, isPivot);
          if(nbh.size())
            return nbh.top().second;
          else return 0.0;
        }

        double getSamplingProbability(GNAT &gnat)
        {
          double N = double(totalChildren() + 1);
          double d = (1.0 + 2.0*_deltaRadius);
          double a = powf(gnat._dataLeaves.size(),_activity);
          double c = 2.0;
          double V = M_PI*powf(nodeRadius(gnat),c);
          return(a*d*V/(double)N);
        }

        double nodeRadius(GNAT& gnat)
        {
          double radius = maxRadius_;
          if(radius<=0.0) radius = _maxObservedRadius;
          if(radius<=0.0) radius = boundingRadius(gnat,pivot_,true);
          return radius;
        }

        const _T& sample(GNAT& gnat, double borderFraction = 0.0, size_t total = 1)
        {
          bool sampleBorder = 0.5 < borderFraction;
          doubleVect distribution;
          double c = 2.0;
          double d = 1.0;
          size_t t = totalChildren() + 1;
          if(total==1) total = t;
          double radius = maxRadius_;
          if(radius<=0.0)
          {
            radius = _maxObservedRadius;
          }

          if(data_.size()==0)
          {
            distribution.clear();
            ompl::Profiler::Begin("GNAT - Child Distribution");
            if(!sampleBorder) distribution.push_back(M_PI*powf(radius/std::max(gnat.tree_->_maxObservedRadius,1.0),c)*powf(1.0 - (double)t/(double)total,d)*1.0/double(t));
            else distribution.push_back(0.0);
            for(unsigned int i=0; i<children_.size(); ++i)
            {
              size_t N = children_[i]->totalChildren() + 1;
              if(!sampleBorder)
              {
                double radius = children_[i]->maxRadius_;
                if(radius<=0.0) radius = children_[i]->_maxObservedRadius;
                if(radius<=0.0) radius = boundingRadius(gnat,children_[i]->pivot_,true);
                distribution.push_back(M_PI*powf(radius/std::max(gnat.tree_->_maxObservedRadius,1.0),c)*powf(1.0 - (double)N/(double)total,d));
                //std::cout<<"<"<<N<<"|"<<distribution.back()<<"|"<<children_[i]->maxRadius_<<">,";
              }
              else
              {
                //distribution.push_back(a*d*V/(double)N);
                exit(0);
              }
            }
            ompl::Profiler::End("GNAT - Child Distribution");
            ompl::Profiler::Begin("GNAT - Calculate Distribution");
            std::vector<size_t> r2 = gnat.rng_.discreteDistribution(distribution,1);
            ompl::Profiler::End("GNAT - Calculate Distribution");
            if(r2[0] == 0) return pivot_;
            else return children_[r2[0]-1]->sample(gnat,borderFraction,total);
          }
          else
          {
            if(data_.size() == 0) return pivot_;
            distribution.clear();
            ompl::Profiler::Begin("GNAT - Data Distribution");
            distribution.push_back(M_PI*powf(boundingRadius(gnat,pivot_,true)/std::max(gnat.tree_->_maxObservedRadius,1.0),c));
            for(size_t k=0; k<data_.size(); k++)
            {
              double radius = boundingRadius(gnat,data_[k],false);
              distribution.push_back(M_PI*powf(radius/std::max(gnat.tree_->_maxObservedRadius,1.0),c));
            }
            ompl::Profiler::End("GNAT - Data Distribution");
            ompl::Profiler::Begin("GNAT - Calculate Data Distribution");
            std::vector<size_t> r2 = gnat.rng_.discreteDistribution(distribution,1);
            ompl::Profiler::End("GNAT - Calculate Data Distribution");
            size_t sampleIndex = r2[0];
            if(sampleIndex == 0) return pivot_;
            return data_[sampleIndex-1];
          }
        }

        std::vector<Node*> getSkeleton(GNAT &gnat)
        {
          std::vector<Node*> result;
          if(data_.size() == 0)
          {
            for(unsigned int i=0; i<children_.size(); ++i)
            {
              std::vector<Node*> r;
              r = children_[i]->getSkeleton(gnat);
              result.insert(result.end(),r.begin(),r.end());
            }
          }
          else
          {
            result.push_back(this);
          }
          return result;
        }

        size_t totalChildren()
        {
          return _totalChildNodes;
        }

        void add(GNAT& gnat, const _T& data)
        {
          ompl::Profiler::Begin("GNAT - Add data");
          _totalChildNodes++;
          double r = gnat.distFun_(data,pivot_);
          if(r>_maxObservedRadius) 
          {
            _previousObservedRadius = _maxObservedRadius;
            _maxObservedRadius = r;
            _deltaRadius = _maxObservedRadius - _previousObservedRadius;
            if(_activity < 0.0)
              _activity = 1.0;
            else
              _activity += 1.0;
          }
          else
          {
            if(_activity > 1) _activity = 0;
            else _activity = _activity - 1;
            if(_activity<-_maxObservedRadius)
              gnat.removeLeaf(this);
          }
          if (children_.size()==0)
          {
            data_.push_back(data);
            gnat.size_++;
            gnat.addLeaf(this);
            if (needToSplit(gnat))
            {
              if (gnat.removed_.size() > 0)
                gnat.rebuildDataStructure();
              else
                split(gnat);
            }
          }
          else
          {
            std::vector<double> dist(children_.size());
            double minDist = std::numeric_limits<double>::infinity();
            int minInd = -1;

            for (unsigned int i=0; i<children_.size(); ++i)
              if ((dist[i] = gnat.distFun_(data, children_[i]->pivot_)) < minDist)
              {
                minDist = dist[i];
                minInd = i;
              }
            for (unsigned int i=0; i<children_.size(); ++i)
            {
              if (children_[i]->minRange_[minInd] > dist[i])
                children_[i]->minRange_[minInd] = dist[i];
              if (children_[i]->maxRange_[minInd] < dist[i])
                children_[i]->maxRange_[minInd] = dist[i];
            }
            if (minDist < children_[minInd]->minRadius_)
              children_[minInd]->minRadius_ = minDist;
            if (minDist > children_[minInd]->maxRadius_)
              children_[minInd]->maxRadius_ = minDist;

            children_[minInd]->add(gnat, data);
          }
          ompl::Profiler::End("GNAT - Add data");
        }

        bool needToSplit(const GNAT& gnat) const
        {
          unsigned int sz = data_.size();
          return sz > gnat.maxNumPtsPerLeaf_ && sz > degree_;
        }
        void split(GNAT& gnat)
        {
          gnat.removeLeaf(this);
          ompl::Profiler::Begin("GNAT - Split Node");
          std::vector<std::vector<double> > dists;
          std::vector<unsigned int> pivots;

          children_.reserve(degree_);
          gnat.pivotSelector_.kcenters(data_, degree_, pivots, dists);
          for(unsigned int i=0; i<pivots.size(); i++)
          {
            children_.push_back(new Node(degree_, gnat.maxNumPtsPerLeaf_, data_[pivots[i]]));
            children_.back()->_activity = 0.0;
            if(_activity > -_maxObservedRadius)
              gnat.addLeaf(children_.back());
          }
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
              child->_totalChildNodes++;
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
          // this does more than clear(); it also sets capacity to 0 and frees the memory
          std::vector<_T> tmp;
          data_.swap(tmp);
          // check if new leaves need to be split
          for (unsigned int i=0; i<degree_; ++i)
            if (children_[i]->needToSplit(gnat))
              children_[i]->split(gnat);
          ompl::Profiler::Begin("GNAT - Split Node");
        }

        // return true iff data was added to nbh.
        bool insertNeighborK(NearQueue& nbh, std::size_t k, const _T& data, const _T& key, double dist) const
        {
          if (nbh.size() < k)
          {
            nbh.push(std::make_pair(&data, dist));
            return true;
          }
          else if (dist < nbh.top().second ||
              (dist < std::numeric_limits<double>::epsilon() && data==key))
          {
            nbh.pop();
            nbh.push(std::make_pair(&data, dist));
            return true;
          }
          return false;
        }

        // for k=1, isPivot is true if the nearest neighbor is a pivot
        void nearestK(const GNAT& gnat, const _T &data, std::size_t k,
            NearQueue& nbh, NodeQueue& nodeQueue, bool& isPivot) const
        {
          for (unsigned int i=0; i<data_.size(); ++i)
            if (!gnat.isRemoved(data_[i]))
            {
              if (insertNeighborK(nbh, k, data_[i], data, gnat.distFun_(data, data_[i])))
                isPivot = false;
            }
          if (children_.size() > 0)
          {
            double dist;
            Node* child;
            std::vector<double> distToPivot(children_.size());
            std::vector<int> permutation(children_.size());

            for (unsigned int i=0; i<permutation.size(); ++i)
              permutation[i] = i;
            std::random_shuffle(permutation.begin(), permutation.end());

            for (unsigned int i=0; i<children_.size(); ++i)
              if (permutation[i] >= 0)
              {
                child = children_[permutation[i]];
                distToPivot[permutation[i]] = gnat.distFun_(data, child->pivot_);
                if (insertNeighborK(nbh, k, child->pivot_, data, distToPivot[permutation[i]]))
                  isPivot = true;
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
                    (distToPivot[permutation[i]] - dist <= child->maxRadius_ &&
                     distToPivot[permutation[i]] + dist >= child->minRadius_))
                  nodeQueue.push(std::make_pair(child, distToPivot[permutation[i]]));
              }
          }
        }

        void insertNeighborR(NearQueue& nbh, double r, const _T& data, double dist) const
        {
          if (dist <= r)
            nbh.push(std::make_pair(&data, dist));
        }

        void nearestR(const GNAT& gnat, const _T &data, double r, NearQueue& nbh, NodeQueue& nodeQueue) const
        {
          double dist = r; //note difference with nearestK

          for (unsigned int i=0; i<data_.size(); ++i)
            if (!gnat.isRemoved(data_[i]))
              insertNeighborR(nbh, r, data_[i], gnat.distFun_(data, data_[i]));
          if (children_.size() > 0)
          {
            Node* child;
            std::vector<double> distToPivot(children_.size());
            std::vector<int> permutation(children_.size());

            for (unsigned int i=0; i<permutation.size(); ++i)
              permutation[i] = i;
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
                if (distToPivot[i] - dist <= child->maxRadius_ &&
                    distToPivot[i] + dist >= child->minRadius_)
                  nodeQueue.push(std::make_pair(child, distToPivot[i]));
              }
          }
        }

        void list(const GNAT& gnat, std::vector<_T> &data) const
        {
          if (!gnat.isRemoved(pivot_))
            data.push_back(pivot_);
          for (unsigned int i=0; i<data_.size(); ++i)
            if(!gnat.isRemoved(data_[i]))
              data.push_back(data_[i]);
          for (unsigned int i=0; i<children_.size(); ++i)
            children_[i]->list(gnat, data);
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
        size_t _totalChildNodes;
        double _maxObservedRadius;
        double _previousObservedRadius;
        double _deltaRadius;
        double _activity;
        double _maxActivity;
    };

    std::map<Node*,double> _dataLeaves;

    void addLeaf(Node* a)
    {
      _dataLeaves[a] = a->getSamplingProbability(*this);
    }

    void removeLeaf(Node *a)
    {
      if(_dataLeaves.find(a) != _dataLeaves.end())
      {
        _dataLeaves.erase(a);
      }
    }

    /** \brief The data elements stored in this structure */
    Node                  *tree_;

    unsigned int           degree_;
    unsigned int           minDegree_;
    unsigned int           maxDegree_;
    unsigned int           maxNumPtsPerLeaf_;
    std::size_t            size_;
    std::size_t            removedCacheSize_;

    /** \brief The data structure used to split data into subtrees */
    ompl::GreedyKCenters<_T>     pivotSelector_;

    /** \brief Cache of removed elements */
    boost::unordered_set<const _T*> removed_;
    RNG rng_;

};

}
#endif
