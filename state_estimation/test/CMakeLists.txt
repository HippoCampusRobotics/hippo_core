ament_add_gtest(tests
  main.cpp ekf_basic.cpp
)
target_link_libraries(tests
  sensor_sim
  ekf
)
