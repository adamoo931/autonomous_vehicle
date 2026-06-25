#pragma once

#include <stdbool.h>
#include <stddef.h>

void wifi_init_sta(void);         // Blokuje do uzyskania IP
bool wifi_is_connected(void);
void wifi_get_ip_str(char *buf, size_t len);  // np. "192.168.1.42"
