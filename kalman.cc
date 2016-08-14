#include "kalman.h"

namespace kalman {

void KalmanFilter::Predict(const Matrix& transition,
                           const Matrix& process_error) {
  mean_ = transition.Multiply(mean_);
  covariance_ = transition.Multiply(covariance_)
                              .Multiply(transition.Transpose())
                              .Add(process_error);
}

void KalmanFilter::UpdateWithCurrentReading(
    const Matrix& reading, const Matrix& reading_covariance,
    const Matrix& scaling) {
  Matrix covariance_scaling_transpose = covariance_.Multiply(
                                            scaling.Transpose());
  Matrix kalman_gain = covariance_scaling_transpose.Multiply(
                           scaling.Multiply(covariance_scaling_transpose).Add(
                               reading_covariance).Invert());
  covariance_ = covariance_.Minus(kalman_gain.Multiply(scaling)
                                             .Multiply(covariance_));
  mean_ = mean_.Add(kalman_gain.Multiply(
              reading.Minus(scaling.Multiply(mean_))));
}

const Matrix& KalmanFilter::GetCurrentMean() const {
  return mean_;
}

const Matrix& KalmanFilter::GetCurrentCovariance() const {
  return covariance_;
}

}  // namespace kalman