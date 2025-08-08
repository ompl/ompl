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
#include <vector>
#include "Vec3.h"

namespace CommonMath {

//! Represents a qunitic polynomial in 3D.
/*!
 *  The polynomial is a function of time and is of the form
 *  c[0]*t^5 + c[1]*t^4 + c[2]*t^3 + c[3]*t^2 + c[4]*t + c[5]
 *  where each coefficient is a vector with three dimensions.
 *  The polynomial is only defined between given start and end times.
 */
class Trajectory {
 public:
  //! Constructor.
  /*!
   * @param coeffs The six 3D coefficients defining the polynomial.
   * The coefficients are ordered such that coeffs[0] corresponds to t^5,
   * coeffs[1] to t^4 and so forth.
   * @param startTime The earliest time at which the polynomial is defined
   * @param endTime The latest time at which the polynomial is defined
   */
  Trajectory(std::vector<Vec3> coeffs, double startTime, double endTime)
      : _coeffs(coeffs),
        _startTime(startTime),
        _endTime(endTime) {
    assert(coeffs.size() == 6);
    assert(startTime <= endTime);
  }

  //! Constructor.
  /*!
   * @param c0 Coefficients for t^5
   * @param c1 Coefficients for t^4
   * @param c2 Coefficients for t^3
   * @param c3 Coefficients for t^2
   * @param c4 Coefficients for t
   * @param c5 Constant terms
   * @param startTime The earliest time at which the polynomial is defined
   * @param endTime The latest time at which the polynomial is defined
   */
  Trajectory(Vec3 c0, Vec3 c1, Vec3 c2, Vec3 c3, Vec3 c4, Vec3 c5,
             double startTime, double endTime)
      : _coeffs { c0, c1, c2, c3, c4, c5 },
        _startTime(startTime),
        _endTime(endTime) {
    assert(startTime <= endTime);
  }

  //! Returns the 3D velocity of the polynomial at a given time.
  /*!
   * @param t Time at which to evaluate the polynomial (must be between startTime and endTime)
   * @return Position of trajectory at time t
   */
  Vec3 GetVelocity(double t) const {
    assert(t >= _startTime);
    assert(t <= _endTime);
    return _coeffs[0] * t * t * t * t * t + _coeffs[1] * t * t * t * t
        + _coeffs[2] * t * t * t + _coeffs[3] * t * t + _coeffs[4] * t
        + _coeffs[5];
  }

  //! Returns the 3D position of the polynomial at a given time.
  /*!
   * @param t Time at which to evaluate the polynomial (must be between startTime and endTime)
   * @return Position of trajectory at time t
   */
  Vec3 GetValue(double t) const {
    assert(t >= _startTime);
    assert(t <= _endTime);
    return _coeffs[0] * t * t * t * t * t + _coeffs[1] * t * t * t * t
        + _coeffs[2] * t * t * t + _coeffs[3] * t * t + _coeffs[4] * t
        + _coeffs[5];
  }
  
  
  
  
  
  
  
  

  //! Return the 3D vector of coefficients
  std::vector<Vec3> GetCoeffs() {
    return _coeffs;
  }

  //! Return the start time of the trajectory
  double GetStartTime() const {
    return _startTime;
  }

  //! Return the end time of the trajectory
  double GetEndTime() const {
    return _endTime;
  }

  //! Returns the coefficient of the time derivative of the trajectory
  /*!
   * @return A 5D vector of 3D coefficients representing the time derivative
   * of the trajectory, where derivCoeffs[0] is a 3D vector of coefficients
   * corresponding to t^4, derivCoeffs[1] corresponds to t^3, etc.
   */
  std::vector<Vec3> GetDerivativeCoeffs() const {
    std::vector<Vec3> derivCoeffs;
    derivCoeffs.reserve(5);
    for (int i = 0; i < 5; i++) {
      derivCoeffs.push_back((5 - i) * _coeffs[i]);
    }
    return derivCoeffs;
  }

  //! Returns the difference between this trajectory and a given trajectory.
  /*!
   * The trajectory that is returned is only defined at times where both
   * trajectories are defined (both trajectories must overlap in time). The
   * returned trajectory is the relative position of the two trajectories in
   * time.
   */
  Trajectory operator-(const Trajectory rhs) {
    double startTime = std::max(_startTime, rhs.GetStartTime());
    double endTime = std::min(_endTime, rhs.GetEndTime());
    return Trajectory(_coeffs[0] - rhs[0], _coeffs[1] - rhs[1],
                      _coeffs[2] - rhs[2], _coeffs[3] - rhs[3],
                      _coeffs[4] - rhs[4], _coeffs[5] - rhs[5], startTime,
                      endTime);
  }

  //! Returns the i-th 3D vector of coefficients.
  inline Vec3 operator[](int i) const {
    switch (i) {
      case 0:
        return _coeffs[0];
      case 1:
        return _coeffs[1];
      case 2:
        return _coeffs[2];
      case 3:
        return _coeffs[3];
      case 4:
        return _coeffs[4];
      case 5:
        return _coeffs[5];
      default:
        // We should never get here (index out of bounds)
        assert(false);
        return Vec3();  // Returns vector of NaNs
    }
  }

 private:
  std::vector<Vec3> _coeffs;
  double _startTime, _endTime;
};

}