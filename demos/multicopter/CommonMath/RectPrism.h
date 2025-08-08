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
#include "ConvexObj.h"
#include "Rotation.h"

namespace CommonMath {

//! A rectangular prism.
/*!
 *  The rectangular prism is defined by the location of the center of the object, three side lengths,
 *  and a rotation between the the global coordinate frame and a coordinate frame aligned with the
 *  edges of the rectangular prism.
 */
class RectPrism : public ConvexObj {
 public:

  //! Constructor.
  /*!
   * Creates the rectangular prism. The local coordinate frame is defined such that it aligns
   * with the edges of the prism (i.e. the x-axis of the local coordinate frame is parallel to
   * the edge defined by sideLengths.x, the y-axis is parallel to the edge defined by sideLengths.y,
   * and the z-axis is parallel to the edge defined by sideLengths.z).
   *
   * @param center The center of the prism written in the global coordinate frame
   * @param sideLengths The length of each side of the prism
   * @param rotation The rotation between the local coordinate frame and global coordinate frame
   */
  RectPrism(Vec3 center, Vec3 sideLengths)
      : _centerPoint(center),
        _sideLengths(sideLengths){
    // Side lengths must be positive
    assert(sideLengths.x > 0 && sideLengths.y > 0 && sideLengths.z > 0);
  }

  //! Finds a separating plane between the prism and given point.
  /*!
   * The resulting Boundary struct is defined by the point on the boundary of the
   * rectangular prism closest to testPoint and a unit normal pointing from this point
   * to testPoint.
   *
   * @param testPoint The coordinates of any point outside of the prism
   * @return A Boundary struct defining a point on the boundary of the prism
   * and a unit vector normal to the separating plane pointing away from the prism
   */
  Boundary GetTangentPlane(Vec3 const testPoint) {
    // Write testPoint in the local coordinate frame
    Vec3 p = testPoint - _centerPoint;

    // Compute Euclidean projection onto cube
    Boundary bound;
    for (int i = 0; i < 3; i++) {
      if (p[i] < -_sideLengths[i] / 2) {
        bound.point[i] = -_sideLengths[i] / 2;
      } else if (p[i] > _sideLengths[i] / 2) {
        bound.point[i] = _sideLengths[i] / 2;
      } else {
        bound.point[i] = p[i];
      }
    }

    // Convert point back to global coordinates and get unit normal
    bound.point = bound.point + _centerPoint;
    bound.normal = (testPoint - bound.point).GetUnitVector();
    return bound;
  }

  //! Check whether a given point is inside or on the boundary of the prism.
  /*!
   * @param testPoint The coordinates of any point
   * @return true if testPoint is inside or on the boundary of the prism, false otherwise
   */
  bool IsPointInside(Vec3 const testPoint) {
    // Write testPoint in the local coordinate frame
    Vec3 p = testPoint - _centerPoint;

    // Check if testPoint is inside the rectangular prism
    return (fabs(p.x) <= _sideLengths.x / 2)
        && (fabs(p.y) <= _sideLengths.y / 2)
        && (fabs(p.z) <= _sideLengths.z / 2);
  }

  Vec3 GetRandomPositionInside(std::default_random_engine randomGen) {
    std::uniform_real_distribution<double> posX(-_sideLengths.x / 2,
                                                _sideLengths.x / 2);
    std::uniform_real_distribution<double> posY(-_sideLengths.y / 2,
                                                _sideLengths.y / 2);
    std::uniform_real_distribution<double> posZ(-_sideLengths.z / 2,
                                                _sideLengths.z / 2);

    Vec3 randomPos = Vec3(posX(randomGen), posY(randomGen), posZ(randomGen));
    return _centerPoint + randomPos;
  }

  //! Sets the location of the center of the prism in global coordinates.
  void SetCenterPoint(Vec3 center) {
    _centerPoint = center;
  }
  //! Sets the side lengths of the prism.
  void SetSideLengths(Vec3 sideLen) {
    _sideLengths = sideLen;
  }

 private:

  Vec3 _centerPoint;  //!< The center of the prism written in the global coordinate frame
  Vec3 _sideLengths;  //!< The total length of each side of the prism
};

}