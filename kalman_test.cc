#include "kalman.h"

#include "gtest/gtest.h"

class KalmanFilterTest : public ::testing::Test {
 protected:
  KalmanFilterTest() {}
  virtual ~KalmanFilterTest() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(KalmanFilterTest, FilterTest) {
  // v' = v
  // x' = x + vdt
  // => F = [ 1 0]
  //        [dt 1]
  kalman::Matrix mean(2, 1, {1.2, 0});
  kalman::Matrix covariance(2, 2, {3.5, 1.1,
                                   2.6, 4.5});
  double dt = 0.2;
  kalman::Matrix transition(2, 2, { 1, 0,
                                   dt, 1});
  kalman::Matrix process_error(2, 2, {1.0, 0,
                                      0.0, 1.0});
  kalman::KalmanFilter filter(mean, covariance);
  filter.Predict(transition, process_error);
  kalman::Matrix expected_mean(2, 1, {1.2, 0.24});
  kalman::Matrix expected_covariance(2, 2, {4.5, 1.8,
                                            3.3, 6.38});
  EXPECT_TRUE(filter.GetCurrentMean().Equals(expected_mean));
  EXPECT_TRUE(filter.GetCurrentCovariance().Equals(expected_covariance));
  kalman::Matrix reading(2, 1, {0.121, 0.023});
  kalman::Matrix reading_covariance(2, 2, {0.111, 0.55,
                                           0.71, 0.223});
  kalman::Matrix scaling = kalman::Matrix::Identity(2).ScalarMultiply(0.1);
  filter.UpdateWithCurrentReading(reading, reading_covariance, scaling);
  EXPECT_TRUE(filter.GetCurrentMean().Equals(
                  kalman::Matrix(2, 1, {1.19940935615474,
                                        0.240772570843335})));
  EXPECT_TRUE(filter.GetCurrentCovariance().Equals(
                  kalman::Matrix(2, 2, {4.29541204061034, 1.41302236622251,
                                        2.77059275366621, 6.05033237631572})));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}