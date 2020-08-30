#ifndef MOUSE_INPUT_HPP
#define MOUSE_INPUT_HPP

#include <array>

namespace mf::minpt {

void updateOldButtonMap();
void setMousePos(int x, int y);
std::array<int, 2> mousePos();
void setButtonState(int button, bool state);
bool buttonDown(int button);
bool buttonPressed(int button);
void setScrollSate(int xVal, int yVal);
std::array<int, 2> mouseWheelState();

} // namespace mf::minpt

#endif
