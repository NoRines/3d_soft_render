#include "controller_input.hpp"

#include <iostream>
#include <unordered_map>
#include <vector>

#include <SDL2/SDL.h>

using ButtonMap = std::array<bool, 256>;
using AxisMap = std::array<float, 256>;

static std::vector<int> g_unusedControllers;

static std::unordered_map<int, SDL_GameController *> g_controllers;
static std::unordered_map<int, ButtonMap> g_buttonMaps;
static std::unordered_map<int, AxisMap> g_axisMaps;

static std::unordered_map<int, ButtonMap> g_oldButtonMaps;

std::size_t mf::cinpt::numControllers() { return g_controllers.size(); }

std::size_t mf::cinpt::numUnusedControllers() {
  return g_unusedControllers.size();
}

std::optional<int> mf::cinpt::getController() {
  if (g_unusedControllers.empty()) {
    return std::nullopt;
  }

  int res = g_unusedControllers.back();
  g_unusedControllers.pop_back();
  return res;
}

void mf::cinpt::updateOldButtonMaps() { g_oldButtonMaps = g_buttonMaps; }

void mf::cinpt::openController(int i) {
  auto controller = SDL_GameControllerOpen(i);
  i = SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controller));

  if (!controller) {
    std::cerr << "Failed to open controller: " << i << '\n';
  } else {
    g_unusedControllers.push_back(i);
    g_controllers[i] = controller;
    g_buttonMaps[i] = {};
    g_axisMaps[i] = {};
  }
}

void mf::cinpt::closeController(int i) {
  auto cit = g_controllers.find(i);
  auto bit = g_buttonMaps.find(i);
  auto ait = g_axisMaps.find(i);

  if (cit == g_controllers.end() || bit == g_buttonMaps.end() ||
      ait == g_axisMaps.end()) {
    return;
  }

  SDL_GameControllerClose(g_controllers[i]);
  g_controllers.erase(cit);
  g_buttonMaps.erase(bit);
  g_axisMaps.erase(ait);
}

void mf::cinpt::setButtonState(int controller, unsigned char button,
                               bool state) {
  auto it = g_buttonMaps.find(controller);

  if (it == g_buttonMaps.end()) {
    return;
  }

  it->second[button] = state;
}

bool mf::cinpt::buttonDown(int controller, unsigned char button) {
  if (auto it = g_buttonMaps.find(controller); it != g_buttonMaps.end()) {
    return it->second[button];
  }
  return false;
}

bool mf::cinpt::buttonPressed(int controller, unsigned char button) {
  if (auto cit = g_buttonMaps.find(controller); cit != g_buttonMaps.end()) {
    if (cit->second[button]) {
      if (auto ocit = g_oldButtonMaps.find(controller);
          ocit != g_oldButtonMaps.end()) {
        return !ocit->second[button];
      }
    }
  }
  return false;
}

void mf::cinpt::setAxisState(int controller, unsigned char axis, int state) {
  auto it = g_axisMaps.find(controller);

  if (it == g_axisMaps.end()) {
    return;
  }

  it->second[axis] = std::clamp(
      (state / static_cast<float>(32768 + 32767)) * 2.0f, -1.0f, 1.0f);
}

float mf::cinpt::sampleAxisState(int controller, unsigned char axis) {
  if (auto it = g_axisMaps.find(controller); it != g_axisMaps.end()) {
    return it->second[axis];
  }
  return 0.0f;
}
