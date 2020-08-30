#include "event_handling.hpp"

#include <iostream>

#include <SDL2/SDL.h>

#include "controller_input.hpp"
#include "keyboard_input.hpp"
#include "mouse_input.hpp"
#include "window_state.hpp"

static void handleWindowEvent(SDL_WindowEvent *wevent) {
  switch (wevent->event) {
  case SDL_WINDOWEVENT_SHOWN: {
    std::cout << "SDL_WINDOWEVENT_SHOWN\n";
    mf::wstat::setShownState(true);
  } break;
  case SDL_WINDOWEVENT_HIDDEN: {
    std::cout << "SDL_WINDOWEVENT_HIDDEN\n";
    mf::wstat::setShownState(false);
  } break;
  case SDL_WINDOWEVENT_EXPOSED: {
    std::cout << "SDL_WINDOWEVENT_EXPOSED\n";
  } break;
  case SDL_WINDOWEVENT_MOVED: {
    std::cout << "SDL_WINDOWEVENT_MOVED\n";
  } break;
  case SDL_WINDOWEVENT_RESIZED: {
    std::cout << "SDL_WINDOWEVENT_RESIZED\n";
    mf::wstat::setDimensions(wevent->data1, wevent->data2);
  } break;
  case SDL_WINDOWEVENT_SIZE_CHANGED: {
    std::cout << "SDL_WINDOWEVENT_SIZE_CHANGED\n";
  } break;
  case SDL_WINDOWEVENT_MINIMIZED: {
    std::cout << "SDL_WINDOWEVENT_MINIMIZED\n";
  } break;
  case SDL_WINDOWEVENT_MAXIMIZED: {
    std::cout << "SDL_WINDOWEVENT_MAXIMIZED\n";
  } break;
  case SDL_WINDOWEVENT_RESTORED: {
    std::cout << "SDL_WINDOWEVENT_RESTORED\n";
  } break;
  case SDL_WINDOWEVENT_ENTER: {
    std::cout << "SDL_WINDOWEVENT_ENTER\n";
  } break;
  case SDL_WINDOWEVENT_LEAVE: {
    std::cout << "SDL_WINDOWEVENT_LEAVE\n";
  } break;
  case SDL_WINDOWEVENT_FOCUS_GAINED: {
    std::cout << "SDL_WINDOWEVENT_FOCUS_GAINED\n";
  } break;
  case SDL_WINDOWEVENT_FOCUS_LOST: {
    std::cout << "SDL_WINDOWEVENT_FOCUS_LOST\n";
  } break;
  case SDL_WINDOWEVENT_CLOSE: {
    std::cout << "SDL_WINDOWEVENT_CLOSE\n";
    mf::wstat::setCloseState(true);
  } break;
  case SDL_WINDOWEVENT_TAKE_FOCUS: {
    std::cout << "SDL_WINDOWEVENT_TAKE_FOCUS\n";
  } break;
  case SDL_WINDOWEVENT_HIT_TEST: {
    std::cout << "SDL_WINDOWEVENT_HIT_TEST\n";
  } break;
  }
}

static void handleKeyUpEvent(SDL_KeyboardEvent *kevent) {
  std::cout << "Key: " << kevent->keysym.sym << " Up\n";
  mf::kinpt::setKeyState(static_cast<unsigned int>(kevent->keysym.sym), false);
}

static void handleKeyDownEvent(SDL_KeyboardEvent *kevent) {
  std::cout << "Key: " << kevent->keysym.sym << " Down\n";
  mf::kinpt::setKeyState(static_cast<unsigned int>(kevent->keysym.sym), true);
}

static void handleMouseMotionEvent(SDL_MouseMotionEvent *mevent) {
  std::cout << "Mouse (" << mevent->x << ", " << mevent->y << ")\n";
  mf::minpt::setMousePos(mevent->x, mevent->y);
}

static void handleMouseButtonUpEvent(SDL_MouseButtonEvent *bevent) {
  std::cout << "MouseBtn: " << static_cast<int>(bevent->button) << " Up at ("
            << bevent->x << ", " << bevent->y << ")\n";
  mf::minpt::setButtonState(bevent->button, false);
}

static void handleMouseButtonDownEvent(SDL_MouseButtonEvent *bevent) {
  std::cout << "MouseBtn: " << static_cast<int>(bevent->button) << " Down at ("
            << bevent->x << ", " << bevent->y << ")\n";
  mf::minpt::setButtonState(bevent->button, true);
}

static void handleMouseWheelEvent(SDL_MouseWheelEvent *wevent) {
  std::cout << "MouseWheel: (" << wevent->x << ", " << wevent->y << ")\n";
  mf::minpt::setScrollSate(wevent->x, wevent->y);
}

static void handleControllerAxisMotionEvent(SDL_ControllerAxisEvent *cevent) {
  std::cout << "Controller: " << cevent->which << " axis: " << (int)cevent->axis
            << " value: " << cevent->value << '\n';
  mf::cinpt::setAxisState(cevent->which, cevent->axis, cevent->value);
}

static void handleControllerButtonDownEvent(SDL_ControllerButtonEvent *cevent) {
  std::cout << "Controller: " << cevent->which
            << " button: " << (int)cevent->button << " down\n";
  mf::cinpt::setButtonState(cevent->which, cevent->button, true);
}

static void handleControllerButtonUpEvent(SDL_ControllerButtonEvent *cevent) {
  std::cout << "Controller: " << cevent->which
            << " button: " << (int)cevent->button << " up\n";
  mf::cinpt::setButtonState(cevent->which, cevent->button, false);
}

static void
handleControllerDeviceAddedEvent(SDL_ControllerDeviceEvent *cevent) {
  std::cout << "Controller: " << cevent->which << " added\n";
  mf::cinpt::openController(cevent->which);
}

static void
handleControllerDeviceRemovedEvent(SDL_ControllerDeviceEvent *cevent) {
  std::cout << "Controller: " << cevent->which << " removed\n";
  mf::cinpt::closeController(cevent->which);
}

static void
handleControllerDeviceRemappedEvent(SDL_ControllerDeviceEvent *cevent) {
  std::cout << "Controller: " << cevent->which << " remapped\n";
}

void mf::evnt::handleEvent(SDL_Event *event) {
  switch (event->type) {

  case SDL_KEYUP: {
    handleKeyUpEvent(&event->key);
  } break;

  case SDL_KEYDOWN: {
    handleKeyDownEvent(&event->key);
  } break;

  case SDL_MOUSEMOTION: {
    handleMouseMotionEvent(&event->motion);
  } break;

  case SDL_MOUSEBUTTONUP: {
    handleMouseButtonUpEvent(&event->button);
  } break;

  case SDL_MOUSEBUTTONDOWN: {
    handleMouseButtonDownEvent(&event->button);
  } break;

  case SDL_MOUSEWHEEL: {
    handleMouseWheelEvent(&event->wheel);
  } break;

  case SDL_CONTROLLERAXISMOTION: {
    handleControllerAxisMotionEvent(&event->caxis);
  } break;

  case SDL_CONTROLLERBUTTONDOWN: {
    handleControllerButtonDownEvent(&event->cbutton);
  } break;

  case SDL_CONTROLLERBUTTONUP: {
    handleControllerButtonUpEvent(&event->cbutton);
  } break;

  case SDL_CONTROLLERDEVICEADDED: {
    handleControllerDeviceAddedEvent(&event->cdevice);
  } break;

  case SDL_CONTROLLERDEVICEREMOVED: {
    handleControllerDeviceRemovedEvent(&event->cdevice);
  } break;

  case SDL_CONTROLLERDEVICEREMAPPED: {
    handleControllerDeviceRemappedEvent(&event->cdevice);
  } break;

  case SDL_WINDOWEVENT: {
    handleWindowEvent(&event->window);
  } break;

  case SDL_QUIT: {
    mf::wstat::setCloseState(true);
  } break;
  }
}
