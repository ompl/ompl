/*!
 * Rapid Collision Detection for Multicopter Trajectories
 *
 * Copyright 2019 by Nathan Bucki <nathan_bucki@berkeley.edu>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include "Vec3.h"
#include <random>
namespace CommonMath {

//! A struct defining the properties of a plane.
/*!
 *  The plane is defined by a point on the plane and a unit vector normal to the plane.
 */
struct Boundary {

  //! Constructor.
  /*!
   * Note that no check is performed to ensure that the provided unit normal is actually a unit vector. We leave this to the user.
   *
   * @param p A point on the plane
   * @param n A unit vector normal to the plane
   */
  Boundary(Vec3 p, Vec3 n) {
    point = p;
    normal = n;
  }

  //! Default constructor.
  /*!
   * Initializes the point and unit normal vectors using the default Vec3 constructor, which initializes
   * each component to NaN.
   */
  Boundary() {
  }

  Vec3 point;  //!< A point on the plane
  Vec3 normal;  //!< A unit vector normal to the plane
};

//! An abstract class that defines the required properties of convex objects.
/*!
 *  Each convex object is required to implement two functions: one that returns a separating plane
 *  between a given point in 3D space and the convex object, and one that determines
 *  whether the given point is inside the convex object.
 */
class ConvexObj {
 public:
  virtual ~ConvexObj() {
    // Empty destructor
  }

  //! Find a separating plane between the convex object and given point.
  /*!
   * @param testPoint The coordinates of any point outside of the convex object
   * @return A Boundary struct defining a point on the boundary of the convex object
   * and a unit vector normal to the separating plane pointing away from the convex object
   */
  virtual Boundary GetTangentPlane(Vec3 testPoint) = 0;

  //! Check whether a given point is inside the convex object.
  /*!
   * @param testPoint The coordinates of any point
   * @return true if testPoint is inside or on the boundary of the convex object, false otherwise
   */
  virtual bool IsPointInside(Vec3 testPoint) = 0;

  virtual Vec3 GetRandomPositionInside(std::default_random_engine randomGen)= 0;

};

}