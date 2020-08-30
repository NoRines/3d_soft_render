#include "window_state.hpp"

namespace mf::wstat {

static bool g_windowShown = false;
static std::array<int, 2> g_windowDimensions = {0, 0};
static bool g_windowClosed = false;

void setShownState(bool s) { g_windowShown = s; }

void setDimensions(int w, int h) {
  g_windowDimensions[0] = w;
  g_windowDimensions[1] = h;
}

void setCloseState(bool s) { g_windowClosed = s; }

bool shownState() { return g_windowShown; }

std::array<int, 2> dimensions() { return g_windowDimensions; }

bool closed() { return g_windowClosed; }

} // namespace mf::wstat
