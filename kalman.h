#ifndef KALMAN_KALMAN_H
#define KALMAN_KALMAN_H

#include "matrix.h"

namespace kalman {

class KalmanFilter {
 public:
  // 'initial_mean' refers to the initial state of the system, while
  // 'initial_covariance' refers to the initial error matrix of the state.
  KalmanFilter(const Matrix& initial_mean, const Matrix& initial_covariance)
      : mean_(initial_mean), covariance_(initial_covariance) {}

  // Performs the prediction stage of Kalman Filter. Suppose x and P are the
  // state and covariance matrix respectively, let 'transition' be F and
  // 'process_error' be Q, then we have:
  //   x := Fx
  //   P := FPF' + Q
  void Predict(const Matrix& transition, const Matrix& process_error);

  // Performs the update stage of Kalman Filter, using the newest 'reading'
  // value with 'reading_covariance' error. Suppose x and P are the state and
  // covariance matrix respectively, while 'reading' and 'reading_covariance'
  // are r and R. Also, let 'scaling' be H. Then we have:
  //    K := PH' inv(HPH' + R)  [kalman gain]
  //    P := P - KHP
  //    x := x + K (r - Hx)
  void UpdateWithCurrentReading(const Matrix& reading,
                                const Matrix& reading_covariance,
                                const Matrix& scaling);

  // Returns the current mean value of the state of the system.
  const Matrix& GetCurrentMean() const;

  // Returns the current covariance of the state of the system.
  const Matrix& GetCurrentCovariance() const;

 private:
  Matrix mean_;
  Matrix covariance_;
};

}  // namespace kalman

#endif  // KALMAN_KALMAN_H