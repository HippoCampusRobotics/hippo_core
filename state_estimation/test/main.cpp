#include <gtest/gtest.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  std::cout << "RUNNING STATE_ESTIMATION UNIT TESTS" << std::endl;
  return RUN_ALL_TESTS();
}
