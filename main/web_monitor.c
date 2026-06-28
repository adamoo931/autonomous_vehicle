#include "web_monitor.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// Rozmiar bufora logow (ostatnie ~8 KB tekstu)
#define LOG_BUF_CAP   8192

static char            s_buf[LOG_BUF_CAP];
static size_t          s_head  = 0;   // nastepna pozycja zapisu
static size_t          s_count = 0;   // liczba waznych bajtow (<= CAP)
static portMUX_TYPE    s_mux   = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t  s_prev_vprintf = NULL;

// Dopisuje pojedynczy bajt do bufora kolowego (pod spinlockiem).
static inline void buf_put(char c) {
    s_buf[s_head] = c;
    s_head = (s_head + 1) % LOG_BUF_CAP;
    if (s_count < LOG_BUF_CAP) s_count++;
}

// Hook logowania: formatuje, usuwa kody ANSI, zapisuje do bufora,
// a takze przekazuje dalej (na wypadek gdyby UART jednak dzialal).
static int monitor_vprintf(const char *fmt, va_list args) {
    char line[256];
    va_list cp;
    va_copy(cp, args);
    int n = vsnprintf(line, sizeof(line), fmt, cp);
    va_end(cp);

    if (n > 0) {
        int len = (n < (int)sizeof(line)) ? n : (int)sizeof(line) - 1;
        portENTER_CRITICAL(&s_mux);
        for (int i = 0; i < len; i++) {
            char c = line[i];
            // Pomin sekwencje ANSI: ESC '[' ... <litera>
            if (c == '\033') {
                i++;
                if (i < len && line[i] == '[') {
                    i++;
                    while (i < len &&
                           !((line[i] >= 'A' && line[i] <= 'Z') ||
                             (line[i] >= 'a' && line[i] <= 'z'))) {
                        i++;
                    }
                }
                continue; // pomin tez znak konczacy
            }
            if (c == '\r') continue;       // bez CR
            buf_put(c);
        }
        portEXIT_CRITICAL(&s_mux);
    }

    // Przekaz do oryginalnego vprintf (UART), jesli istnial.
    if (s_prev_vprintf) {
        return s_prev_vprintf(fmt, args);
    }
    return n;
}

void web_monitor_init(void) {
    s_prev_vprintf = esp_log_set_vprintf(monitor_vprintf);
    ESP_LOGI("WEBMON", "Web monitor aktywny – logi dostepne pod /api/logs");
}

size_t web_monitor_dump(char *out, size_t maxlen) {
    if (maxlen == 0) return 0;
    portENTER_CRITICAL(&s_mux);
    size_t count = s_count;
    size_t start = (s_head + LOG_BUF_CAP - s_count) % LOG_BUF_CAP;
    size_t to_copy = (count < maxlen - 1) ? count : maxlen - 1;
    // Pomijamy najstarsze bajty, jesli bufor wiekszy niz docelowy.
    if (count > to_copy) {
        start = (start + (count - to_copy)) % LOG_BUF_CAP;
    }
    for (size_t i = 0; i < to_copy; i++) {
        out[i] = s_buf[(start + i) % LOG_BUF_CAP];
    }
    portEXIT_CRITICAL(&s_mux);
    out[to_copy] = '\0';
    return to_copy;
}

void web_monitor_clear(void) {
    portENTER_CRITICAL(&s_mux);
    s_head  = 0;
    s_count = 0;
    portEXIT_CRITICAL(&s_mux);
}
