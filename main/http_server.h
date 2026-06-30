#pragma once
#include <esp_err.h>

/* Serwer HTTP: dashboard sterujący oraz API JSON (czujniki, silniki, diody,
 * buzzer, autonomia, skan LIDAR, logi). Implementacja w http_server.c. */

esp_err_t http_server_start(void);
void      http_server_stop(void);
