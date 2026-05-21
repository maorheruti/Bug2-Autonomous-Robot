#pragma once

#include <WebServer.h>

void wifi_manager_init(const char* defaultSsid, const char* defaultPass);
void wifi_manager_begin_with_fallback();
void wifi_manager_loop();
void wifi_manager_register_routes(WebServer& server);
