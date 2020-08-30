#ifndef MAT_MATH_HPP
#define MAT_MATH_HPP

#include "vec_math.hpp"

template <floating_point T, std::size_t N> using Mat = std::array<Vec<T, N>, N>;

template <floating_point T, std::size_t N>
constexpr auto matMul(const Mat<T, N> &a, const Mat<T, N> &b) {
  Mat<T, N> result;

  auto colToRow = []<std::size_t... i>(int col, const Mat<T, N> &m,
                                       std::index_sequence<i...>) {
    return Vec<T, N>{m[i][col]...};
  };

  auto dotWithCol = [&]<std::size_t... i>(int row, std::index_sequence<i...>) {
    ((result[row][i] =
          a[row].dot(colToRow(i, b, std::make_index_sequence<N>()))),
     ...);
  };

  [&]<std::size_t... i>(std::index_sequence<i...>) {
    ((dotWithCol(i, std::make_index_sequence<N>())), ...);
  }
  (std::make_index_sequence<N>());

  return result;
}

template <floating_point T, std::size_t N>
constexpr auto matVecMul(const Mat<T, N> &a, const Vec<T, N> &b) {
  Vec<T, N> result;

  [&]<std::size_t... i>(std::index_sequence<i...>) {
    ((result[i] = a[i].dot(b)), ...);
  }
  (std::make_index_sequence<N>());

  return result;
}

template <floating_point T, std::size_t N>
constexpr auto operator*(const Mat<T, N> &a, const Mat<T, N> &b) {
  return matMul(a, b);
}

template <floating_point T, std::size_t N>
constexpr auto operator*(const Mat<T, N> &a, const Vec<T, N> &b) {
  return matVecMul(a, b);
}

using Mat2f = Mat<float, 2>;
using Mat2d = Mat<double, 2>;
using Mat3f = Mat<float, 3>;
using Mat3d = Mat<double, 3>;
using Mat4f = Mat<float, 4>;
using Mat4d = Mat<double, 4>;

template <floating_point T> constexpr inline auto identityMat2() {
  return Mat<T, 2>{{{static_cast<T>(1), static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(1)}}};
}

template <floating_point T> constexpr inline auto identityMat3() {
  return Mat<T, 3>{{{static_cast<T>(1), static_cast<T>(0), static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(1), static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(1)}}};
}

template <floating_point T> constexpr inline auto identityMat4() {
  return Mat<T, 4>{{{static_cast<T>(1), static_cast<T>(0), static_cast<T>(0),
                     static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(1), static_cast<T>(0),
                     static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(1),
                     static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(0),
                     static_cast<T>(1)}}};
}

template <floating_point T> constexpr inline auto scaleMat2(T x, T y) {
  return Mat<T, 2>{{{x, static_cast<T>(0)}, {static_cast<T>(0), y}}};
}

template <floating_point T>
constexpr inline auto scaleMat2(const Vec<T, 2> &vec) {
  return scaleMat2(vec.x(), vec.y());
}

template <floating_point T> constexpr inline auto scaleMat3(T x, T y, T z) {
  return Mat<T, 3>{{{x, static_cast<T>(0), static_cast<T>(0)},
                    {static_cast<T>(0), y, static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(0), z}}};
}

template <floating_point T>
constexpr inline auto scaleMat3(const Vec<T, 3> &vec) {
  return scaleMat3(vec.x(), vec.y(), vec.z());
}

template <floating_point T>
constexpr inline auto scaleMat4(T x, T y, T z, T w) {
  return Mat<T, 4>{
      {{x, static_cast<T>(0), static_cast<T>(0), static_cast<T>(0)},
       {static_cast<T>(0), y, static_cast<T>(0), static_cast<T>(0)},
       {static_cast<T>(0), static_cast<T>(0), z, static_cast<T>(0)},
       {static_cast<T>(0), static_cast<T>(0), static_cast<T>(0), w}}};
}

template <floating_point T>
constexpr inline auto scaleMat4(const Vec<T, 4> &vec) {
  return scaleMat4(vec.x(), vec.y(), vec.z(), vec.w());
}

template <floating_point T> inline auto rotationMat3(T rotAmt) {
  const T c = std::cos(rotAmt);
  const T s = std::sin(rotAmt);
  return Mat<T, 3>{{{c, -s, static_cast<T>(0)},
                    {s, c, static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(1)}}};
}

template <floating_point T> inline auto xRotationMat4(T rotAmt) {
  const T c = std::cos(rotAmt);
  const T s = std::sin(rotAmt);
  return Mat<T, 4>{{{static_cast<T>(1), static_cast<T>(0), static_cast<T>(0),
                     static_cast<T>(0)},
                    {static_cast<T>(0), c, -s, static_cast<T>(0)},
                    {static_cast<T>(0), s, c, static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(0),
                     static_cast<T>(1)}}};
}

template <floating_point T> inline auto yRotationMat4(T rotAmt) {
  const T c = std::cos(rotAmt);
  const T s = std::sin(rotAmt);
  return Mat<T, 4>{{{c, static_cast<T>(0), s, static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(1), static_cast<T>(0),
                     static_cast<T>(0)},
                    {-s, static_cast<T>(0), c, static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(0),
                     static_cast<T>(1)}}};
}

template <floating_point T> inline auto zRotationMat4(T rotAmt) {
  const T c = std::cos(rotAmt);
  const T s = std::sin(rotAmt);
  return Mat<T, 4>{{{c, -s, static_cast<T>(0), static_cast<T>(0)},
                    {s, c, static_cast<T>(0), static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(1),
                     static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(0),
                     static_cast<T>(1)}}};
}

template <floating_point T> inline auto rotationMat4(T x, T y, T z) {
  const T cx = std::cos(x);
  const T sx = std::sin(x);
  const T cy = std::cos(y);
  const T sy = std::sin(y);
  const T cz = std::cos(z);
  const T sz = std::sin(z);

  return Mat<T, 4>{{{cy * cz, sx * sy * cz - cx * sz, sx * sz + cx * sy * cz,
                     static_cast<T>(0)},
                    {cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - sx * cz,
                     static_cast<T>(0)},
                    {-sy, sx * cy, cx * cy, static_cast<T>(0)},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(0),
                     static_cast<T>(1)}}};
}

template <floating_point T>
constexpr inline auto rotationMat4(const Vec<T, 3> &vec) {
  return rotationMat4(vec.x(), vec.y(), vec.z());
}

template <floating_point T> constexpr inline auto translateMat3(T x, T y) {
  return Mat<T, 3>{{{static_cast<T>(1), static_cast<T>(0), x},
                    {static_cast<T>(0), static_cast<T>(1), y},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(1)}}};
}

template <floating_point T>
constexpr inline auto translateMat3(const Vec<T, 2> &vec) {
  return translateMat3(vec.x(), vec.y());
}

template <floating_point T> constexpr inline auto translateMat4(T x, T y, T z) {
  return Mat<T, 4>{
      {{static_cast<T>(1), static_cast<T>(0), static_cast<T>(0), x},
       {static_cast<T>(0), static_cast<T>(1), static_cast<T>(0), y},
       {static_cast<T>(0), static_cast<T>(0), static_cast<T>(1), z},
       {static_cast<T>(0), static_cast<T>(0), static_cast<T>(0),
        static_cast<T>(1)}}};
}

template <floating_point T>
constexpr inline auto translateMat4(const Vec<T, 3> &vec) {
  return translateMat4(vec.x(), vec.y(), vec.z());
}

template <floating_point T> inline auto projectionMat(T a, T fov, T n, T f) {
  const T i = static_cast<T>(1) / a;
  const T t = static_cast<T>(1) / std::tan(fov / 2);
  const T q = f / (f - n);
  return Mat<T, 4>{
      {{i * t, static_cast<T>(0), static_cast<T>(0), static_cast<T>(0)},
       {static_cast<T>(0), t, static_cast<T>(0), static_cast<T>(0)},
       {static_cast<T>(0), static_cast<T>(0), q, -n * q},
       {static_cast<T>(0), static_cast<T>(0), static_cast<T>(1),
        static_cast<T>(0)}}};
}

template <floating_point T>
constexpr inline auto pointAtMat(const Vec<T, 3> &eye, const Vec<T, 3> &center,
                                 const Vec<T, 3> &up) {
  const auto f = (center - eye).unit();
  const auto s = f.cross(up).unit();
  const auto u = s.cross(f);

  return Mat<T, 4>{{{s.x(), u.x(), f.x(), eye.x()},
                    {s.y(), u.y(), f.y(), eye.y()},
                    {s.z(), u.z(), f.z(), eye.z()},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(0),
                     static_cast<T>(1)}}};
}

template <floating_point T>
constexpr inline auto lookAtMat(const Vec<T, 3> &eye, const Vec<T, 3> &center,
                                const Vec<T, 3> &up) {
  const auto f = (center - eye).unit();
  const auto s = f.cross(up).unit();
  const auto u = s.cross(f).unit();

  return Mat<T, 4>{{{s.x(), s.y(), s.z(), -s.dot(eye)},
                    {u.x(), u.y(), u.z(), -u.dot(eye)},
                    {f.x(), f.y(), f.z(), -f.dot(eye)},
                    {static_cast<T>(0), static_cast<T>(0), static_cast<T>(0),
                     static_cast<T>(1)}}};
}

#endif
