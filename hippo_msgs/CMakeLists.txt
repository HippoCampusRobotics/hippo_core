# Copyright (C) 2023 Thies Lennart Alff
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
# USA

cmake_minimum_required(VERSION 3.5)
project(hippo_msgs)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# needed for clang-tidy to work
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2)
add_compile_options(${CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AnchorPose.msg"
  "msg/AnchorPoses.msg"
  "msg/AngularVelocity.msg"
  "msg/BoolStamped.msg"
  "msg/DepthStamped.msg"
  "msg/EscRpms.msg"
  "msg/EscVoltages.msg"
  "msg/Float64Stamped.msg"
  "msg/Int64Stamped.msg"
  "msg/ModemOut.msg"
  "msg/NewtonGripperCommand.msg"
  "msg/Path.msg"
  "msg/PathFollowerDebug.msg"
  "msg/RangeMeasurement.msg"
  "msg/RangeMeasurementArray.msg"
  "srv/SetAxis.srv"
  "srv/SetPath.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
