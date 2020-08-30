#ifndef WINDOW_STATE_HPP
#define WINDOW_STATE_HPP

#include <array>

namespace mf::wstat {
void setShownState(bool s);
void setDimensions(int w, int h);
void setCloseState(bool s);
bool shownState();
std::array<int, 2> dimensions();
bool closed();
} // namespace mf::wstat

#endif
