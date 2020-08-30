#ifndef TRIANGLE_DRAWER_HPP
#define TRIANGLE_DRAWER_HPP

#include "rasterizer.hpp"

template <typename T> class Slope {
public:
  constexpr Slope() = default;
  constexpr Slope(T from, T to, int steps)
      : current(from), step((to - from) / (float)steps) {}

  constexpr T get() const { return current; }
  constexpr void advance() { current += step; }

private:
  T current;
  T step;
};

template <typename F, typename T>
concept callable_by_tuple = requires(F f, T t) {
  {std::apply(f, t)};
};

template <std::size_t xindex = 0, std::size_t yindex = 1,
          std::size_t zindex = 2, bool propPerspectiveCorrect = true,
          is_tuple<2> T>
constexpr void
drawTriangle(const T &p0, const T &p1, const T &p2,
             auto &&plt) requires callable_by_tuple<decltype(plt), T> {

  constexpr std::size_t T_SIZE = std::tuple_size_v<T>;
  // using SlopeData = std::array<Slope, T_SIZE - 1>;
  // using Prop = Slope;
  // using PropData = std::array<Prop, T_SIZE - 2>;

  rastTriangle(
      &p0, &p1, &p2,
      [](const auto &p) {
        return std::tuple(std::get<xindex>(p), std::get<yindex>(p));
      },
      [](const auto *from, const auto *to, int steps) {
        auto createSlope = [&]<std::size_t i>(std::index_sequence<i>) {
          constexpr std::size_t p = i + (i >= yindex);
          // const float f = (float)std::get<p>(*from);
          // const float t = (float)std::get<p>(*to);
          const auto f = std::get<p>(*from);
          const auto t = std::get<p>(*to);

          if constexpr (p == xindex || !propPerspectiveCorrect) {
            // return Slope(f, t, steps);
            return Slope((float)f, (float)t, steps);
          } else if constexpr (p == zindex) {
            return Slope(1.0f / f, 1.0f / t, steps);
          } else {
            return Slope(f / std::get<zindex>(*from), t / std::get<zindex>(*to),
                         steps);
          }
        };

        auto createSlopeData = [&]<std::size_t... i>(
            std::index_sequence<i...>) {
          // return SlopeData{createSlope(std::index_sequence<i>())...};
          return std::tuple(createSlope(std::index_sequence<i>())...);
        };

        return createSlopeData(
            // std::make_index_sequence<std::tuple_size_v<SlopeData>>());
            std::make_index_sequence<T_SIZE - 1>());
      },
      //[&plt](SlopeData &ls, SlopeData &rs, int y) {
      [&plt](auto &ls, auto &rs, int y) {
        constexpr std::size_t slope_xindex = xindex - (xindex >= yindex);

        const int startx = (int)std::get<slope_xindex>(ls).get();
        const int endx = (int)std::get<slope_xindex>(rs).get();
        const int xSteps = endx - startx;

        auto createProp = [&]<std::size_t i>(std::index_sequence<i>) {
          constexpr std::size_t p = i + (i >= slope_xindex);
          // const float f = std::get<p>(ls).get();
          // const float t = std::get<p>(rs).get();
          const auto f = std::get<p>(ls).get();
          const auto t = std::get<p>(rs).get();
          // return Prop(f, t, xSteps);
          return Slope(f, t, xSteps);
        };

        auto createPropData = [&]<std::size_t... i>(std::index_sequence<i...>) {
          // return PropData{createProp(std::index_sequence<i>())...};
          return std::tuple(createProp(std::index_sequence<i>())...);
        };

        auto props = createPropData(
            // std::make_index_sequence<std::tuple_size_v<PropData>>());
            std::make_index_sequence<T_SIZE - 2>());

        auto getParameter = [&]<std::size_t i>(int x, std::index_sequence<i>) {
          constexpr std::size_t p = i - (i >= xindex) - (i >= yindex);
          constexpr std::size_t z =
              zindex - (zindex >= xindex) - (zindex >= yindex);

          if constexpr (i == xindex) {
            return x;
          } else if constexpr (i == yindex) {
            return y;
          } else if constexpr (!propPerspectiveCorrect) {
            return std::get<p>(props).get();
          } else if constexpr (i == zindex) {
            return 1.0f / std::get<z>(props).get();
          } else {
            return std::get<p>(props).get() / std::get<z>(props).get();
          }
        };

        auto callPlt = [&]<std::size_t... i>(int x, std::index_sequence<i...>) {
          plt(getParameter(x, std::index_sequence<i>())...);
        };

        auto indexes = std::make_index_sequence<T_SIZE>();

        for (int x = startx; x < endx; x++) {
          callPlt(x, indexes);
          // for (auto &p : props)
          //  p.advance();
          [&props]<std::size_t... i>(std::index_sequence<i...>) {
            ((std::get<i>(props).advance()), ...);
          }
          (std::make_index_sequence<std::tuple_size_v<decltype(props)>>());
        }

        // for (auto &s : ls)
        //  s.advance();
        [&ls]<std::size_t... i>(std::index_sequence<i...>) {
          ((std::get<i>(ls).advance()), ...);
        }
        (std::make_index_sequence<
            std::tuple_size_v<std::remove_cvref_t<decltype(ls)>>>());
        // for (auto &s : rs)
        //  s.advance();
        [&rs]<std::size_t... i>(std::index_sequence<i...>) {
          ((std::get<i>(rs).advance()), ...);
        }
        (std::make_index_sequence<
            std::tuple_size_v<std::remove_cvref_t<decltype(rs)>>>());
      });
}

#endif
