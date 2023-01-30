/*!
 * Rapid Collision Detection for Multicopter Trajectories
 *
 * Copyright 2019 by Nathan Bucki <nathan_bucki@berkeley.edu>
 * Modified by Thies Lennart ALff <thies.lennart.alff@tuhh.de>
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
#include "convex_obj.hpp"

namespace trajectory_math {

//! A sphere.
/*!
 *  The sphere is defined by the location of its center and its radius.
 */
class Sphere : public ConvexObj {
 public:

  //! Constructor.
  /*!
   * Creates a sphere at a given location with a given radius.
   *
   * @param center The center of the sphere written in the global coordinate frame
   * @param radius The radius of the sphere
   */
  Sphere(const Eigen::Vector3d &center, double radius)
      : _centerPoint(center),
        _radius(radius) {
    // Radius must be positive
    assert(radius > 0);
  }

  //! Finds a separating plane between the sphere and given point.
  /*!
   * The resulting Boundary struct is defined by the point on the boundary of the
   * sphere closest to testPoint and a unit normal pointing from this point
   * to testPoint.
   *
   * @param testPoint The coordinates of any point outside of the sphere
   * @return A Boundary struct defining a point on the boundary of the sphere
   * and a unit vector normal to the separating plane pointing away from the sphere
   */
  Boundary GetTangentPlane(const Eigen::Vector3d &testPoint) {
    Boundary bound;
    bound.normal = (testPoint - _centerPoint).normalized();
    bound.point = (testPoint - _centerPoint).normalized() * _radius
        + _centerPoint;
    return bound;
  }

  //! Check whether a given point is inside or on the boundary of the sphere.
  /*!
   * @param testPoint The coordinates of any point
   * @return true if testPoint is inside or on the boundary of the sphere, false otherwise
   */
  bool IsPointInside(const Eigen::Vector3d &testPoint) {
    return (testPoint - _centerPoint).norm() <= _radius;
  }

 private:
  Eigen::Vector3d _centerPoint;  //!< The center of the sphere written in the global coordinate frame
  double _radius;  //!< The radius of the sphere

};
}
