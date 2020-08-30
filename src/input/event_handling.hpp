#ifndef EVENT_HANDLING_HPP
#define EVENT_HANDLING_HPP

union SDL_Event;

namespace mf::evnt {
void handleEvent(SDL_Event *event);
}

#endif
