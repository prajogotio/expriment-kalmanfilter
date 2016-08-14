#include "matrix.h"

#include "gtest/gtest.h"
#include <iostream>

class MatrixTest : public ::testing::Test {
 protected:
  MatrixTest() {}
  virtual ~MatrixTest() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(MatrixTest, AdditionTest) {
  kalman::Matrix m(2, 2, {1, 2, 3, -5}), n(2, 2, {1, 2, 3, 4});
  kalman::Matrix result = m.Add(n);
  kalman::Matrix expected(2, 2, {2, 4, 6, -1});
  EXPECT_TRUE(result.Equals(expected));
}

TEST_F(MatrixTest, MinusTest) {
  kalman::Matrix m(2, 2, {1, 2, 3, -5}), n(2, 2, {1, 2, 3, 4});
  kalman::Matrix result = m.Minus(n);
  kalman::Matrix expected(2, 2, {0, 0, 0, -9});
  EXPECT_TRUE(result.Equals(expected));
}

TEST_F(MatrixTest, MultiplicationTest) {
  kalman::Matrix m(2, 3, { 1,  2,  0.5,
                          -3, -1, -0.5});
  kalman::Matrix n(3, 4, { 1,   0, -1, 3,
                           2, 0.5,  1, 1,
                          -1, 0.5, -1, 2});
  kalman::Matrix result = m.Multiply(n);
  kalman::Matrix expected(2, 4, {4.5, 1.25, 0.5, 6,
                                 -4.5, -0.75, 2.5, -11});
  EXPECT_TRUE(result.Equals(expected));
}

TEST_F(MatrixTest, TransposeTest) {
  kalman::Matrix m(2, 3, { 1,  2,  0.5,
                          -3, -1, -0.5});
  kalman::Matrix result = m.Transpose();
  kalman::Matrix expected(3, 2, {1, -3,
                                 2, -1,
                                 0.5, -0.5});
  EXPECT_TRUE(result.Equals(expected));
}

TEST_F(MatrixTest, InvertTest2x2) {
  kalman::Matrix m(2, 2, { 1,  2,
                          -3, -1});
  kalman::Matrix result = m.Invert();
  kalman::Matrix expected(2, 2, {-1.0/5, -2.0/5,
                                 3.0/5, 1.0/5});
  EXPECT_TRUE(result.Equals(expected));
}

TEST_F(MatrixTest, InvertTest3x3) {
  kalman::Matrix m(3, 3, { 1,  2, 3,
                          -3, -1, 4,
                           0, -0.4, 1.2});
  kalman::Matrix result = m.Invert();
  kalman::Matrix expected(3, 3,
      {0.035714285714285726, -0.32142857142857145, 0.9821428571428572,
       0.32142857142857145, 0.10714285714285714, -1.1607142857142856,
       0.10714285714285712, 0.03571428571428572, 0.4464285714285714});
  EXPECT_TRUE(result.Equals(expected));
}

TEST_F(MatrixTest, InvertTest3x3IdentityK) {
  kalman::Matrix m = kalman::Matrix::Identity(3).ScalarMultiply(42.0);
  kalman::Matrix result = m.Invert();
  kalman::Matrix expected(kalman::Matrix::Identity(3).ScalarMultiply(1.0/42));
  EXPECT_TRUE(result.Equals(expected));
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}