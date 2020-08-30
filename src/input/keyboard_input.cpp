#include "keyboard_input.hpp"

#include <unordered_map>



static std::unordered_map<unsigned int, bool> g_keyMap;
static std::unordered_map<unsigned int, bool> g_oldKeyMap;


void mf::kinpt::updateOldKeyMap()
{
	g_oldKeyMap = g_keyMap;
}


void mf::kinpt::setKeyState(unsigned int key, bool state)
{
	g_keyMap[key] = state;
}


bool mf::kinpt::keyDown(unsigned int key)
{
	auto it = g_keyMap.find(key);
	if(it == g_keyMap.end()) {
		return false;
	}
	return it->second;
}


bool mf::kinpt::keyPressed(unsigned int key)
{
	auto it = g_keyMap.find(key);

	if(it == g_keyMap.end()) {
		return false;
	}

	auto itOld = g_oldKeyMap.find(key);
	if(itOld == g_oldKeyMap.end()) {
		return it->second;
	}

	return it->second && !itOld->second;
}
