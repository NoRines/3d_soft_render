#ifndef KEYBOARD_INPUT_HPP
#define KEYBOARD_INPUT_HPP


namespace mf::kinpt {
void updateOldKeyMap();
void setKeyState(unsigned int key, bool state);
bool keyDown(unsigned int key);
bool keyPressed(unsigned int key);
}


#endif
