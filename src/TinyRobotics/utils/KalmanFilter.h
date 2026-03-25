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
 * @brief This class implements a simple Kalman Filter for state estimation. It
 * maintains the state vector (x), the covariance matrix (P), the state
 * transition matrix (F), the process noise covariance (Q), the measurement
 * matrix (H), and the measurement noise covariance (R). The predict() method
 * performs the prediction step of the Kalman Filter, while the update() method
 * incorporates new measurements to refine the state estimate. This
 * implementation is designed to be simple and efficient for use in embedded
 * systems, with fixed-size matrices and basic linear algebra operations. The
 * template parameters NX and NZ allow for flexibility in the size of the state
 * and measurement vectors, making it adaptable to various applications such as
 * localization, sensor fusion, or any scenario where state estimation is needed
 * based on noisy measurements.
 *
 * @tparam NX Number of state variables
 * @tparam NZ Number of measurement variables
 */
template <size_t NX, size_t NZ>
class KalmanFilter {
 public:
  Matrix<NX, 1> x;   // state
  Matrix<NX, NX> P;  // covariance
  Matrix<NX, NX> Fmat;  // state transition
  Matrix<NX, NX> Q;  // process noise

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