#ifndef CONTROLLER_INPUT_HPP
#define CONTROLLER_INPUT_HPP

#include <optional>

namespace mf::cinpt {
std::size_t numControllers();
std::size_t numUnusedControllers();
std::optional<int> getController();
void updateOldButtonMaps();
void openController(int i);
void closeController(int i);
void setButtonState(int controller, unsigned char button, bool state);
bool buttonDown(int controller, unsigned char button);
bool buttonPressed(int controller, unsigned char button);
void setAxisState(int controller, unsigned char axis, int state);
float sampleAxisState(int controller, unsigned char axis);
} // namespace mf::cinpt

#endif
