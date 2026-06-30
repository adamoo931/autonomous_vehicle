#pragma once

#include <stdbool.h>
#include <stddef.h>

/* Łączność WiFi w trybie stacji (STA). Dane sieci (SSID/hasło) w config.h. */

/* Inicjalizuje WiFi i blokuje do uzyskania adresu IP lub wyczerpania prób. */
void wifi_init_sta(void);

bool wifi_is_connected(void);

/* Zapisuje bieżący adres IP jako tekst, np. "192.168.1.42". */
void wifi_get_ip_str(char *buf, size_t len);
