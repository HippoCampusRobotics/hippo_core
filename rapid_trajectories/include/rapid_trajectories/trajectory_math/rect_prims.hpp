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
  RectPrism(const Eigen::Vector3d &center, const Eigen::Vector3d &sideLengths, const Eigen::Quaterniond &rotation)
      : _centerPoint(center),
        _sideLengths(sideLengths),
        _rotation(rotation),
        _rotationInv(rotation.inverse()) {
    // Side lengths must be positive
    assert(sideLengths.x() > 0 && sideLengths.y() > 0 && sideLengths.z() > 0);
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
  Boundary GetTangentPlane(const Eigen::Vector3d &testPoint) {
    // Write testPoint in the local coordinate frame
    Eigen::Vector3d p = _rotationInv * (testPoint - _centerPoint);

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
    bound.point = _rotation * bound.point + _centerPoint;
    bound.normal = (testPoint - bound.point).normalized();
    return bound;
  }

  //! Check whether a given point is inside or on the boundary of the prism.
  /*!
   * @param testPoint The coordinates of any point
   * @return true if testPoint is inside or on the boundary of the prism, false otherwise
   */
  bool IsPointInside(const Eigen::Vector3d &testPoint) {
    // Write testPoint in the local coordinate frame
    Eigen::Vector3d p = _rotationInv * (testPoint - _centerPoint);

    // Check if testPoint is inside the rectangular prism
    return (fabs(p.x()) <= _sideLengths.x() / 2)
        && (fabs(p.y()) <= _sideLengths.y() / 2)
        && (fabs(p.z()) <= _sideLengths.z() / 2);
  }

  //! Sets the location of the center of the prism in global coordinates.
  void SetCenterPoint(const Eigen::Vector3d &center) {
    _centerPoint = center;
  }
  //! Sets the side lengths of the prism.
  void SetSideLengths(const Eigen::Vector3d &sideLen) {
    _sideLengths = sideLen;
  }
  //! Sets the rotation between the local and global coordinate frames.
  void SetRotation(const Eigen::Quaterniond &rot) {
    _rotation = rot;
    _rotationInv = rot.inverse();
  }

 private:

  Eigen::Vector3d _centerPoint;  //!< The center of the prism written in the global coordinate frame
  Eigen::Vector3d _sideLengths;  //!< The total length of each side of the prism
  Eigen::Quaterniond _rotation;  //!< Rotation from local coordinate frame to global coordinate frame
  Eigen::Quaterniond _rotationInv;  //!< Rotation from global coordinate frame to local coordinate frame

};

}
