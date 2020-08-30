#ifndef UTILS_HPP
#define UTILS_HPP

#include <array>
#include <chrono>
#include <thread>

namespace mf::util {

template <typename Dest, typename Src, std::size_t N, std::size_t... I>
auto _convert_array_to_impl(const std::array<Src, N> &src,
                           std::index_sequence<I...>) {
  return std::array<Dest, N>{{static_cast<Dest>(src[I])...}};
}

template <typename Dest, typename Src, std::size_t N>
auto convert_array_to(const std::array<Src, N> &src) {
  return _convert_array_to_impl<Dest>(src, std::make_index_sequence<N>{});
}

// Move to other file
template <typename ClockResolution, unsigned int targetFps>
std::size_t limit_fps() {
  using clock =
      std::conditional_t<std::chrono::high_resolution_clock::is_steady,
                         std::chrono::high_resolution_clock,
                         std::chrono::steady_clock>;

  constexpr auto second = std::chrono::seconds(1); // Definera en sekund
  constexpr auto targetFrameTime = std::chrono::nanoseconds(second) / targetFps;

  static auto previousTime = clock::now();

  auto currentTime = clock::now();

  auto workTime = currentTime - previousTime;

  if (workTime < targetFrameTime) {
    std::this_thread::sleep_for(targetFrameTime - workTime);
  }

  previousTime = clock::now();

  auto sleepTime = previousTime - currentTime;

  return std::chrono::duration_cast<ClockResolution>(sleepTime + workTime)
      .count();
}

} // namespace mf::util

#endif
