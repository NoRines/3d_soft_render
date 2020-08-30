#ifndef RASTERIZE_HPP
#define RASTERIZE_HPP

#include <algorithm>

template <typename T, std::size_t minSz = 1, std::size_t maxSz = ~std::size_t{}>
concept is_tuple = requires(T t) {
  {std::get<0>(t)};
}
and(std::tuple_size_v<std::remove_cvref_t<T>> - minSz) <= (maxSz - minSz);

template <is_tuple<2> T>
void rastTriangle(
    const T *p0, const T *p1, const T *p2, auto &&getXY, auto &&makeSlope,
    auto &&drawScanLine) requires std::invocable<decltype(getXY), const T &>
    and is_tuple<decltype(getXY(*p0)), 2, 2>
        and std::invocable<decltype(makeSlope), const T *, const T *, int>
            and std::invocable<decltype(drawScanLine),
                               decltype(makeSlope(p0, p1, 0)) &,
                               decltype(makeSlope(p0, p1, 0)) &, int> {

  auto [x0, y0, x1, y1, x2, y2] =
      std::tuple_cat(getXY(*p0), getXY(*p1), getXY(*p2));

  if (std::tie(y1, x1) < std::tie(y0, x0)) {
    std::swap(y1, y0);
    std::swap(x1, x0);
    std::swap(p1, p0);
  }

  if (std::tie(y2, x2) < std::tie(y0, x0)) {
    std::swap(y2, y0);
    std::swap(x2, x0);
    std::swap(p2, p0);
  }

  if (std::tie(y2, x2) < std::tie(y1, x1)) {
    std::swap(y2, y1);
    std::swap(x2, x1);
    std::swap(p2, p1);
  }

  if (y2 == y0) {
    return;
  }

  const bool shortside = (y0 - y2) * (x1 - x0) + (x2 - x0) * (y1 - y0) < 0;

  using slopeT =
      std::invoke_result_t<decltype(makeSlope), const T *, const T *, int>;
  std::array<slopeT, 2> sides;

  sides[!shortside] = makeSlope(p0, p2, y2 - y0);
  sides[shortside] = makeSlope(p0, p1, y1 - y0);

  for (int y = y0; y < y1; y++) {
    drawScanLine(sides[0], sides[1], y);
  }

  sides[shortside] = makeSlope(p1, p2, y2 - y1);

  for (int y = y1; y < y2; y++) {
    drawScanLine(sides[0], sides[1], y);
  }
}

#endif
