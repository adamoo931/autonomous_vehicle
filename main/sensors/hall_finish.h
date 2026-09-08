#pragma once
#include <stdbool.h>

/*
 * Cyfrowy czujnik Halla mety — moduł 49E + komparator LM393 + potencjometr
 * progu. Wyjście DO modułu podłączone do PIN_HALL_FINISH_DO (GPIO, wejście
 * cyfrowe z podciąganiem). Moduł zasilany z 3,3 V, więc DO pracuje w logice
 * 3,3 V i wchodzi wprost na pin ESP32.
 *
 * DOBÓR PROGU BEZ MIERNIKA:
 *   1. Uruchom robota, otwórz dashboard (karta "Halla mety").
 *   2. Obserwuj pole "DO surowy" (0/1). Zbliżaj magnes mety do czujnika i
 *      kręć potencjometrem na module, aż DO zmienia stan przy pożądanym
 *      dystansie.
 *   3. Przełącznikiem "meta = DO LOW" ustaw polaryzację tak, aby pole "Meta"
 *      pokazywało WYKRYTO tylko z magnesem przy czujniku.
 *
 * Nic nie jest zapisywane w NVS — po restarcie polaryzacja wraca do
 * HALL_FINISH_DO_ACTIVE_LOW z config.h, próg trzyma potencjometr sprzętowo.
 */

/* Konfiguruje pin DO jako wejście cyfrowe z pull-up. */
void hall_finish_init(void);

/* Surowy stan pinu DO: 0 lub 1 (do dobierania progu potencjometrem). */
int hall_finish_do_raw(void);

/* Meta wykryta = (DO surowy == stan aktywny). Stan aktywny ustawia
 * hall_finish_set_active_low(). */
bool hall_finish_detected(void);

/* Zatrzask impulsu DO. Moduł cyfrowy 49E+LM393 daje KRÓTKI impuls DO w chwili
 * przejeżdżania nad magnesem (czasem < czas debounce), a gdy pojazd staje na
 * taśmie - często już za magnesem, DO wraca do spoczynku. hall_finish_poll()
 * (wołane co takt pętli autonomii) znaczy czas ostatniej aktywności DO;
 * hall_finish_seen_recently() mówi, czy impuls był w ciągu ostatnich
 * within_ms - dzięki temu meta jest wykrywana mimo krótkiego impulsu w ruchu.
 * hall_finish_clear_latch() kasuje zatrzask (na starcie przejazdu). */
void hall_finish_poll(unsigned int now_ms);
bool hall_finish_seen_recently(unsigned int now_ms, unsigned int within_ms);
void hall_finish_clear_latch(void);

/* Polaryzacja: true = "meta" gdy DO w stanie LOW (typowe moduły LM393),
 * false = gdy DO w stanie HIGH. Ustawiane z dashboardu; nie zapisywane w NVS. */
void hall_finish_set_active_low(bool active_low);
bool hall_finish_get_active_low(void);

/* Reakcja na Hall mety podczas jazdy RĘCZNEJ (ton finiszu + zielona dioda +
 * tryb szukania ciepła pirometrem w main.c). W trybie autonomicznym bez
 * znaczenia — tam sygnał mety odgrywany jest na DECYZJĘ autonomii
 * (autonomy_finish_reached()), nie na surowy odczyt Halla. Domyślnie
 * wyłączona; przełączana z dashboardu; nie zapisywana w NVS. */
void hall_finish_set_manual_enabled(bool enabled);
bool hall_finish_get_manual_enabled(void);
