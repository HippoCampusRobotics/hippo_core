#include <gtest/gtest.h>
#include <state_estimation/ekf.h>

#include <state_estimation/sensor_sim/sensor_sim.h>

class EkfBasicTest : public ::testing::Test {
 public:
  EkfBasicTest()
      : ::testing::Test(), ekf_{std::make_shared<Ekf>()}, sensor_sim_(ekf_){};

  void SetUp() override {
    ekf_->Init(0);
    sensor_sim_.RunSeconds(init_period_);
  }

  void TearDown() override {}

  std::shared_ptr<Ekf> ekf_{nullptr};
  SensorSim sensor_sim_;
  const double init_period_{4.0};
};

TEST_F(EkfBasicTest, tiltAlign) {
  EXPECT_TRUE(ekf_->AttitudeValid());
}
