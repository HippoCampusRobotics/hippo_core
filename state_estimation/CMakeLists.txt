cmake_minimum_required(VERSION 3.5)
project(state_estimation)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# needed for clang-tidy to work
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2)
add_compile_options(${CXX_FLAGS})

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
set(dependencies
  rclcpp
  Eigen3
  hippo_msgs
  geometry_msgs
  sensor_msgs
)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(ekf SHARED
  src/interface.cpp
  src/ekf/ekf_covariance.cpp
  src/ekf/ekf.cpp
  src/ekf/ekf_util.cpp
  src/util.cpp
)
ament_target_dependencies(ekf
  ${dependencies}
)

add_library(sensor_sim SHARED
  src/sensor_sim/baro.cpp
  src/sensor_sim/imu.cpp
  src/sensor_sim/sensor.cpp
  src/sensor_sim/sensor_sim.cpp
  src/sensor_sim/vision.cpp
)
ament_target_dependencies(sensor_sim
  ${dependencies}
)
target_link_libraries(sensor_sim
  ekf
)

add_executable(estimator
  src/estimator.cpp
)
target_link_libraries(estimator
  ekf
)

ament_target_dependencies(estimator ${dependencies})

# install(
#     DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
# )
install(TARGETS
  ekf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  estimator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_linter_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
  # ament_add_gtest(basic_test
  #   test/basic_test.cpp)
  # ament_target_dependencies(basic_test ${dependencies})
endif()

ament_export_dependencies(${dependencies})
ament_export_libraries(ekf)
ament_export_include_directories(include)
ament_package()
