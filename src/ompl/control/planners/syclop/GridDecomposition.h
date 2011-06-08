#ifndef GRIDDECOMPOSITION_H
#define GRIDDECOMPOSITION_H

#include <cstdlib>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/datastructures/Grid.h"

namespace ompl
{
    namespace control
    {
        class GridDecomposition : public Decomposition
        {

            public:
            /* In some areas of this class, we are assuming a 2-dimensional grid. This will change. */
            GridDecomposition(const int len, const int dim, const base::RealVectorBounds &b);

            virtual ~GridDecomposition();

            virtual void print() const;

            virtual double getRegionVolume(const int rid) const;

            /* Projecting a state into whatever space on which this decomposition is defined is a problem-specific issue.
             * Implementations of this function can call the protected overloaded version, which takes a coordinate with
             * the same dimension as the grid. */
            virtual int locateRegion(const base::State *s) = 0;

            /* This implementation requires time linear with the number of regions.
             * We can do constant time if we know the dimension offline (oopsmp-syclop has cases for 2 and 3),
             * but can we beat linear time with arbitrary dimension? */
            virtual void getNeighbors(const int rid, std::vector<int>& neighbors);

            protected:
            /* Using datastructures/grid may not even be necessary. */
            Grid<Region*> grid;

           /* Locate the region in the grid containing the point determined by coord. Since
            * we are assuming a 2-dimensional grid, coord.size() will be 2 for now. */
            virtual int locateRegion(const std::vector<double> coord);

            virtual bool areNeighbors(int r, int s);

            private:
            void buildGrid(const int n);

            /* Convert a region ID to a grid coordinate, which is a vector of length equivalent
             * to the dimension of the grid. */
            void regionToCoord(int rid, std::vector<int>& coord);

            const int length;
            const int dimension;
            double cellVolume;
        };
    }
}
#endif
