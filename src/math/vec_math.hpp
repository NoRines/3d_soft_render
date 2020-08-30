#ifndef VEC_MATH_HPP
#define VEC_MATH_HPP

#include <array>
#include <cmath>
#include <numeric>
#include <tuple>

template <typename T> concept floating_point = std::is_floating_point_v<T>;

template <typename T> concept vec_data = requires(T t) {
  {std::get<0>(t), t[0]};
};

constexpr floating_point auto sum(const vec_data auto &vec) {
  return std::apply([](auto &&... v) { return (v + ...); }, vec);
}

template <typename T, typename U>
concept op_possible = vec_data<T> and
                      (vec_data<U> or
                       std::is_arithmetic_v<std::remove_cvref_t<U>>);

template <typename binop = std::multiplies<void>, vec_data T, typename U = T,
          bool scalar = std::is_arithmetic_v<U>>
constexpr auto vecOp(const T &vec, const U &val) requires op_possible<T, U> {

  using E = std::conditional_t<scalar, std::array<U, std::tuple_size_v<T>>, U>;
  static_assert(std::tuple_size_v<T> == std::tuple_size_v<E>,
                "Vec size mismatch\n");

  return [&]<std::size_t... i>(std::index_sequence<i...>)->E {
    if constexpr (scalar) {
      return {binop{}(std::get<i>(vec), val)...};
    } else {
      return {binop{}(std::get<i>(vec), std::get<i>(val))...};
    }
  }
  (std::make_index_sequence<std::tuple_size_v<E>>());
}

template <typename T> concept is_vec = requires(T t) { {t.data}; }
and vec_data<decltype(std::remove_cvref_t<T>::data)>
        and std::tuple_size_v<decltype(std::remove_cvref_t<T>::data)> >= 2;

constexpr inline auto operator+(const is_vec auto &a, const is_vec auto &b) {
  return std::remove_reference_t<decltype(a)>(
      vecOp<std::plus<void>>(a.data, b.data));
}
constexpr inline auto operator-(const is_vec auto &a, const is_vec auto &b) {
  return std::remove_reference_t<decltype(a)>(
      vecOp<std::minus<void>>(a.data, b.data));
}
constexpr inline auto operator*(const is_vec auto &a, const is_vec auto &b) {
  return std::remove_reference_t<decltype(a)>(
      vecOp<std::multiplies<void>>(a.data, b.data));
}
constexpr inline auto operator*(const is_vec auto &a,
                                const floating_point auto &b) {
  return std::remove_reference_t<decltype(a)>(
      vecOp<std::multiplies<void>>(a.data, b));
}
constexpr inline auto operator*(const floating_point auto &a,
                                const is_vec auto &b) {
  return std::remove_reference_t<decltype(b)>(
      vecOp<std::multiplies<void>>(b.data, a));
}
constexpr inline auto operator/(const is_vec auto &a, const is_vec auto &b) {
  return std::remove_reference_t<decltype(a)>(
      vecOp<std::divides<void>>(a.data, b.data));
}
constexpr inline auto operator/(const is_vec auto &a,
                                const floating_point auto &b) {
  return std::remove_reference_t<decltype(a)>(
      vecOp<std::divides<void>>(a.data, b));
}
constexpr inline auto operator-(const is_vec auto &a) {
  return a * static_cast<decltype(a.data[0])>(-1);
}

template <floating_point T, typename CRTP> class VecBase {
private:
  inline CRTP &self() { return static_cast<CRTP &>(*this); }
  inline const CRTP &self() const { return static_cast<const CRTP &>(*this); }

public:
  constexpr inline T &operator[](std::size_t i) { return self().data[i]; }
  constexpr inline const T &operator[](std::size_t i) const {
    return self().data[i];
  }

  constexpr inline CRTP &operator+=(const CRTP &other) {
    return self() = self() + other;
  }
  constexpr inline CRTP &operator-=(const CRTP &other) {
    return self() = self() - other;
  }
  constexpr inline CRTP &operator*=(const CRTP &other) {
    return self() = self() * other;
  }
  constexpr inline CRTP &operator*=(T other) { return self() = self() * other; }
  constexpr inline CRTP &operator/=(const CRTP &other) {
    return self() = self() / other;
  }
  constexpr inline CRTP &operator/=(T other) { return self() = self() / other; }

  constexpr inline T dot(const CRTP &other) const {
    return sum(vecOp<std::multiplies<void>>(self().data, other.data));
  }
  constexpr inline T length() const { return std::sqrt(dot(self())); }
  constexpr inline CRTP unit() const { return self() / length(); }
};

template <floating_point T, std::size_t N>
struct Vec : public VecBase<T, Vec<T, N>> {
  constexpr Vec() = default;
  constexpr Vec(std::array<T, N> &&d) : data(d) {}
  std::array<T, N> data;
};

template <floating_point T> struct Vec<T, 2> : public VecBase<T, Vec<T, 2>> {
  constexpr Vec() = default;
  constexpr Vec(T x, T y) : data{x, y} {}
  constexpr Vec(std::array<T, 2> &&d) : data(d) {}
  std::array<T, 2> data;

  constexpr inline T &x() { return std::get<0>(data); }
  constexpr inline T &y() { return std::get<1>(data); }

  constexpr inline T x() const { return std::get<0>(data); }
  constexpr inline T y() const { return std::get<1>(data); }

  constexpr inline T &u() { return std::get<0>(data); }
  constexpr inline T &v() { return std::get<1>(data); }

  constexpr inline T u() const { return std::get<0>(data); }
  constexpr inline T v() const { return std::get<1>(data); }
};

template <floating_point T> struct Vec<T, 3> : public VecBase<T, Vec<T, 3>> {
  constexpr Vec() = default;
  constexpr Vec(T x, T y, T z) : data{x, y, z} {}
  constexpr Vec(std::array<T, 3> &&d) : data(d) {}
  std::array<T, 3> data;

  constexpr inline T &x() { return std::get<0>(data); }
  constexpr inline T &y() { return std::get<1>(data); }
  constexpr inline T &z() { return std::get<2>(data); }

  constexpr inline T x() const { return std::get<0>(data); }
  constexpr inline T y() const { return std::get<1>(data); }
  constexpr inline T z() const { return std::get<2>(data); }

  constexpr inline T &r() { return std::get<0>(data); }
  constexpr inline T &g() { return std::get<1>(data); }
  constexpr inline T &b() { return std::get<2>(data); }

  constexpr inline T r() const { return std::get<0>(data); }
  constexpr inline T g() const { return std::get<1>(data); }
  constexpr inline T b() const { return std::get<2>(data); }

  constexpr inline Vec cross(const Vec &other) const {
    return Vec(y() * other.z() - z() * other.y(),
               z() * other.x() - x() * other.z(),
               x() * other.y() - y() * other.x());
  }
};

template <floating_point T> struct Vec<T, 4> : public VecBase<T, Vec<T, 4>> {
  constexpr Vec() = default;
  constexpr Vec(T x, T y, T z, T w) : data{x, y, z, w} {}
  constexpr Vec(std::array<T, 4> &&d) : data(d) {}
  std::array<T, 4> data;

  constexpr inline T &x() { return std::get<0>(data); }
  constexpr inline T &y() { return std::get<1>(data); }
  constexpr inline T &z() { return std::get<2>(data); }
  constexpr inline T &w() { return std::get<3>(data); }

  constexpr inline T x() const { return std::get<0>(data); }
  constexpr inline T y() const { return std::get<1>(data); }
  constexpr inline T z() const { return std::get<2>(data); }
  constexpr inline T w() const { return std::get<3>(data); }
};

template <std::size_t N, std::size_t T> concept is_gr_eq_than = (N >= T);

template <typename T, std::size_t N>
constexpr inline Vec<T, 2>
toVec2(const Vec<T, N> &v) requires is_gr_eq_than<N, 2> {
  return {v[0], v[1]};
}

template <typename T, std::size_t N>
constexpr inline Vec<T, 3>
toVec3(const Vec<T, N> &v) requires is_gr_eq_than<N, 3> {
  return {v[0], v[1], v[2]};
}

template <typename T>
constexpr inline Vec<T, 3> toVec3(const Vec<T, 2> &v, T z) {
  return {v[0], v[1], z};
}

template <typename T, std::size_t N>
constexpr inline Vec<T, 4>
toVec4(const Vec<T, N> &v) requires is_gr_eq_than<N, 4> {
  return {v[0], v[1], v[2], v[3]};
}

template <typename T>
constexpr inline Vec<T, 4> toVec4(const Vec<T, 2> &v, T z, T w) {
  return {v[0], v[1], z, w};
}

template <typename T>
constexpr inline Vec<T, 4> toVec4(const Vec<T, 3> &v, T w) {
  return {v[0], v[1], v[2], w};
}

using Vec2f = Vec<float, 2>;
using Vec2d = Vec<double, 2>;
using Vec3f = Vec<float, 3>;
using Vec3d = Vec<double, 3>;
using Vec4f = Vec<float, 4>;
using Vec4d = Vec<double, 4>;

#endif
