#pragma once

#include <array>
#include <cstddef>

namespace tinyrobotics {

// Simple static matrix
template <size_t R, size_t C>
struct Matrix {
  std::array<float, R * C> data{};

  float& operator()(size_t r, size_t c) { return data[r * C + c]; }

  const float& operator()(size_t r, size_t c) const { return data[r * C + c]; }

  static Matrix<R, C> identity() {
    static_assert(R == C, "Identity only for square matrices");
    Matrix<R, C> m;
    for (size_t i = 0; i < R; ++i) m(i, i) = 1.0f;
    return m;
  }
};

// Matrix multiplication
template <size_t R, size_t C, size_t K>
Matrix<R, K> operator*(const Matrix<R, C>& a, const Matrix<C, K>& b) {
  Matrix<R, K> result;
  for (size_t i = 0; i < R; ++i)
    for (size_t j = 0; j < K; ++j)
      for (size_t k = 0; k < C; ++k) result(i, j) += a(i, k) * b(k, j);
  return result;
}

// Matrix addition
template <size_t R, size_t C>
Matrix<R, C> operator+(const Matrix<R, C>& a, const Matrix<R, C>& b) {
  Matrix<R, C> result;
  for (size_t i = 0; i < R * C; ++i) result.data[i] = a.data[i] + b.data[i];
  return result;
}

// Matrix subtraction
template <size_t R, size_t C>
Matrix<R, C> operator-(const Matrix<R, C>& a, const Matrix<R, C>& b) {
  Matrix<R, C> result;
  for (size_t i = 0; i < R * C; ++i) result.data[i] = a.data[i] - b.data[i];
  return result;
}

// Transpose
template <size_t R, size_t C>
Matrix<C, R> transpose(const Matrix<R, C>& m) {
  Matrix<C, R> result;
  for (size_t i = 0; i < R; ++i)
    for (size_t j = 0; j < C; ++j) result(j, i) = m(i, j);
  return result;
}

// Inverse for 1x1 and 2x2 only (embedded-friendly)
template <size_t N>
Matrix<N, N> inverse(const Matrix<N, N>& m);

// 1x1 inverse
template <>
inline Matrix<1, 1> inverse(const Matrix<1, 1>& m) {
  Matrix<1, 1> r;
  r(0, 0) = 1.0f / m(0, 0);
  return r;
}

// 2x2 inverse
template <>
inline Matrix<2, 2> inverse(const Matrix<2, 2>& m) {
  Matrix<2, 2> r;
  float det = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);

  r(0, 0) = m(1, 1) / det;
  r(0, 1) = -m(0, 1) / det;
  r(1, 0) = -m(1, 0) / det;
  r(1, 1) = m(0, 0) / det;

  return r;
}

/**
 * @class KalmanFilter
 * @brief A lightweight, fixed-size Kalman Filter for embedded state estimation.
 *
 * This template class implements a discrete-time, linear Kalman Filter for
 * real-time state estimation in embedded and robotics applications. It is
 * designed for efficiency and simplicity, using static, fixed-size matrices and
 * minimal dynamic memory. The filter estimates the internal state of a process
 * given noisy measurements and a mathematical model of the system dynamics.
 *
 * ## Features
 * - Fixed-size, stack-allocated matrices for predictable memory usage
 * - Suitable for microcontrollers and real-time systems
 * - Supports arbitrary state and measurement dimensions via template parameters
 * - Simple API: `predict()` for time update, `update(z)` for measurement update
 *
 * ## Mathematical Model
 * The Kalman Filter models the system as:
 *
 *   x_k   = F * x_{k-1} + w      (state transition)
 *   z_k   = H * x_k     + v      (measurement)
 *
 *   where:
 *     - x: state vector (NX x 1)
 *     - z: measurement vector (NZ x 1)
 *     - F: state transition matrix (NX x NX)
 *     - H: measurement matrix (NZ x NX)
 *     - Q: process noise covariance (NX x NX)
 *     - R: measurement noise covariance (NZ x NZ)
 *     - w, v: process and measurement noise (zero-mean, Gaussian)
 *
 * ## Usage Example
 * @code
 *   KalmanFilter<2, 1> kf;
 *   kf.Fmat = ...; // set state transition
 *   kf.H = ...;    // set measurement model
 *   kf.Q = ...;    // set process noise
 *   kf.R = ...;    // set measurement noise
 *   kf.x = ...;    // set initial state
 *   kf.P = ...;    // set initial covariance
 *   kf.predict();
 *   kf.update(z);
 * @endcode
 *
 * ## Template Parameters
 * @tparam NX Number of state variables (state vector dimension)
 * @tparam NZ Number of measurement variables (measurement vector dimension)
 *
 * ## Members
 * - x: State estimate vector (NX x 1)
 * - P: State covariance matrix (NX x NX)
 * - Fmat: State transition matrix (NX x NX)
 * - Q: Process noise covariance (NX x NX)
 * - H: Measurement matrix (NZ x NX)
 * - R: Measurement noise covariance (NZ x NZ)
 * - I: Identity matrix (NX x NX)
 *
 * ## Methods
 * - predict(): Performs the time update (prediction) step
 * - update(z): Performs the measurement update (correction) step
 *
 * ## Applications
 * - Sensor fusion (IMU, GPS, encoders)
 * - Robot localization
 * - Signal filtering and smoothing
 * - Any scenario requiring optimal state estimation from noisy data
 */
template <size_t NX, size_t NZ>
class KalmanFilter {
 public:
  Matrix<NX, 1> x;      // state
  Matrix<NX, NX> P;     // covariance
  Matrix<NX, NX> Fmat;  // state transition
  Matrix<NX, NX> Q;     // process noise

  Matrix<NZ, NX> H;  // measurement matrix
  Matrix<NZ, NZ> R;  // measurement noise

  Matrix<NX, NX> I = Matrix<NX, NX>::identity();

  // Prediction step
  void predict() {
    x = Fmat * x;
    P = Fmat * P * transpose(Fmat) + Q;
  }

  // Update step
  void update(const Matrix<NZ, 1>& z) {
    auto y = z - (H * x);                    // innovation
    auto S = H * P * transpose(H) + R;       // innovation covariance
    auto K = P * transpose(H) * inverse(S);  // Kalman gain

    x = x + (K * y);
    P = (I - (K * H)) * P;
  }
};

}  // namespace tinyrobotics