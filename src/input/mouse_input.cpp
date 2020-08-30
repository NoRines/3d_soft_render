#include "mouse_input.hpp"
#include <iostream>

namespace mf::minpt {

static std::array<int, 2> g_mousePos{};
static std::array<int, 2> g_scrollState{};
static std::array<bool, 10> g_buttonMap{};
static std::array<bool, 10> g_oldButtonMap{};

void updateOldButtonMap() {
  g_oldButtonMap = g_buttonMap;
  g_scrollState = {0, 0};
}

void setMousePos(int x, int y) {
  g_mousePos[0] = x;
  g_mousePos[1] = y;
}

std::array<int, 2> mousePos() { return g_mousePos; }

void setButtonState(int button, bool state) { g_buttonMap[button] = state; }

bool buttonDown(int button) { return g_buttonMap[button]; }

bool buttonPressed(int button) {
  return g_buttonMap[button] && !g_oldButtonMap[button];
}

void setScrollSate(int xVal, int yVal) {
  g_scrollState[0] = xVal;
  g_scrollState[1] = yVal;
}

std::array<int, 2> mouseWheelState() { return g_scrollState; }

} // namespace mf::minpt
