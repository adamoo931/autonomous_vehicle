#include "autonomy.h"
#include "config.h"
#include "motor_driver.h"
#include "pyrometer.h"
#include "ina219.h"
#include "line_sensor.h"
#include "hall_finish.h"
#include "imu.h"
#include "lidar.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>
#include <limits.h>

static const char *TAG = "AUTO";

/* =====================================================================
 *  Autonomia - wersja minimalna (etap 1, napisana od zera)
 *
 *  Zadanie skryptu jazdy:
 *    1. po wlaczeniu jedzie na wprost ze stala predkoscia,
 *    2. po wykryciu linii toru (ktorykolwiek z TRZECH czujnikow
 *       odbiciowych CNY70 podpietych pod ADC ADS1115: przod-L, tyl-L,
 *       tyl-P; czwarty, cyfrowy na GPIO, swiadomie pomijany - falszywe
 *       odczyty) pojazd STAJE i przez ~LINE_WAIT_MS czeka na sygnal z
 *       czujnika Halla,
 *    3. jesli w tym czasie Hall zamelduje mete (SS495A -> ADS1115,
 *       finish_detected) -> to meta: stop ostateczny, koniec przejazdu,
 *    4. jesli Hall nie zadziala -> to nie meta, tylko zwykle przeciecie
 *       tasmy wyznaczajacej tor: pojazd cofa sie przez ~LINE_BACKUP_MS,
 *       stoi jeszcze ~LINE_POSTBACKUP_PAUSE_MS, po czym znow rusza na
 *       wprost. Wykrywanie linii NIE jest wyciszane po cofnieciu - jesli
 *       pojazd ponownie najedzie na te sama tasme, test (stop/oczekiwanie
 *       na Hall/cofniecie) powtarza sie od nowa. To celowe: ma sluzyc do
 *       weryfikacji poprawnosci odczytow czujnikow odbiciowych (pojazd ma
 *       sie "zapetlac" na tej samej linii, dopoki operator nie przerwie
 *       autonomii recznie).
 *
 *  Swiadomie nie ma tu omijania przeszkod, korekty kursu ani danych z
 *  LIDAR-u - logika czujnikow pozostaje nietknieta, ten modul tylko z
 *  niej korzysta. Kazda reczna komenda silnikow wylacza autonomie
 *  (kill-switch w http_server.c). Potwierdzenie mety wylacza autonomie -
 *  stan "Zatrzymany (meta)" zostaje widoczny na dashboardzie do czasu
 *  ponownego wlaczenia.
 *
 *  LOG PRZEJAZDU: co STATUS_LOG_MS do RAM zapisywany jest kompaktowy
 *  rekord. Krok 1 dolozyl komplet sygnalow decyzyjnych: 3 osie zyroskopu,
 *  scalkowany kurs wzgledny (surowy - patrz nizej), 8 sektorow LIDAR i
 *  napiecia 3 czujnikow linii; dochodza tez stan, moc silnikow i
 *  temperatury z pirometru. Bufor zerowany na starcie kazdego przejazdu,
 *  do pobrania jako CSV pod GET /api/autonomy/log.csv.
 * ===================================================================== */

/* Moc silnikow [% mocy] przy jezdzie na wprost i przy cofaniu (ta wersja:
 * bez ruchu obrotowego) - stala moc przez caly czas ruchu, bez impulsu
 * rozruchowego. Ustawiana z dashboardu (s_speed_pct, patrz
 * autonomy_set_speed_pct()); SPEED_PCT_DEFAULT to wartosc startowa po
 * uruchomieniu ESP32. Gdy w przyszlosci dojdzie logika obrotu/skretu w
 * miejscu, powinna uzywac wyzszej mocy (~50%) - jedno kolo stojace daje
 * wiecej tarcia niz jazda na wprost oboma kolami. */
#define SPEED_PCT_DEFAULT   35
#define SPEED_PCT_MIN         0
#define SPEED_PCT_MAX       100

/* Test mety po wykryciu linii. Meta to magnes pod ta sama tasma, ktora
 * wyznacza granice toru, wiec sama linia nie odroznia mety od zwyklego
 * przeciecia tasmy - robi to dopiero czujnik Halla. */
#define LINE_WAIT_MS            1500  /* postoj na linii - oczekiwanie na sygnal Hall */
#define LINE_BACKUP_MS          2000  /* czas cofania, gdy linia to nie meta */
#define LINE_POSTBACKUP_PAUSE_MS 500  /* postoj po cofnieciu, zanim pojazd znow ruszy */

/* Linia startowa jest tez oznaczona tasma odblaciowa, przez ktora pojazd
 * musi przejechac na wprost zaraz po starcie - bez tego pierwszy krok
 * ST_CRUISE natychmiast "wykrywalby linie" na wlasnej linii startowej.
 * Przez ten czas PO ZAKONCZENIU KALIBRACJI zyra (jednorazowo, tylko na
 * poczatku przejazdu - patrz s_run_cruise_start_ms) czujniki linii sa
 * ignorowane. Czasowe, nie dystansowe, bo odometria jest zepsuta. */
#define START_LINE_IGNORE_MS    3000

/* Krok 5b: strona krawedzi wywnioskowana z tego, KTORY czujnik linii
 * zareagowal jako pierwszy. Zwalidowane logiem (przejazdwzdluzlinii.csv):
 * przy realnej krawedzi po PRAWEJ stronie robota, we WSZYSTKICH kontaktach
 * zglosil sie WYLACZNIE tyl-prawy (bez przod-lewy/tyl-lewy). Kombinacje z
 * udzialem przod-lewy (jedyny dzialajacy czujnik z przodu) sa geometrycznie
 * niejednoznaczne - traktowane jako nieznane, bez korekty kierunku. */
typedef enum { EDGE_SIDE_UNKNOWN, EDGE_SIDE_LEFT, EDGE_SIDE_RIGHT } edge_side_t;

#define EDGE_TURN_AWAY_DEG      20.0f  /* dodatkowe "wycelowanie" kursu od strony
                                         * trafionej krawedzi po odbiciu (patrz Krok 5b) */

/* Krok 5d: debounce detekcji linii i Halla - N kolejnych taktow petli (20 Hz)
 * z warunkiem prawdziwym, zanim uznamy go za realny. Zdiagnozowane logiem
 * (przejazdwzdluz2.csv): przy braku debounce pojedynczy skok ADC na kanale
 * linii (cala magistrala ADS1115 dzieli te sama podatnosc na zaklocenia
 * silnikow co zyroskop) potrafil na jeden takt (50 ms) uzbroic okno kontaktu
 * z Hallem bez realnego najechania na tasme - niewidoczne w logu CSV (probki
 * co 300 ms), ale spojne z obserwowanym "fake" zdarzeniem. 3 takty = ~150 ms,
 * znikome wzgledem realnego kontaktu trwajacego sekundy. */
#define LINE_DEBOUNCE_COUNT     3
#define HALL_DEBOUNCE_COUNT     3

/* Okresy petli sterowania i logowania. */
#define LOOP_MS          50      /* 20 Hz - petla sterowania */
#define STATUS_LOG_MS   300      /* ~3,3 Hz - log konsoli + rekord CSV */

/* --- Krok 2: estymator kursu z zyroskopu ---
 * Na starcie przejazdu ~BIAS_CAL_MS bezruchu -> usrednienie gyro_z = bias
 * (w tescie Kroku 1 zmierzono ~0,8 °/s). Potem calkowanie (gyro_z - bias),
 * z EMA na predkosc katowa (szum ±10 °/s od pracujacych silnikow).
 * Znak potwierdzony w Kroku 1: +gyro_z = obrot w LEWO (CCW) = +kurs. */
#define BIAS_CAL_MS           1000   /* czas usredniania biasu na starcie [ms] */
#define BIAS_CAL_MAX_SPREAD    4.0f  /* max-min gyro_z podczas kal. [°/s]; wiecej => ostrzezenie */
#define HEADING_SIGN         (+1.0f) /* odwroc na -1, jesli kierunek kursu wyjdzie odwrotny */
#define HEADING_LPF_ALPHA      0.20f /* EMA na predkosc katowa przed calkowaniem (Krok 4b: 0.30 -> 0.20,
                                       * mocniejsze tlumienie szumu - patrz dryf na dlugich przejazdach) */
#define HEADING_DT_MAX_S       0.30f /* dluzsza przerwa w petli => pomijamy calkowanie (unik skoku) */

/* --- Krok 4b: powolna adaptacja biasu zyra podczas USTALONEJ jazdy prostej.
 * Diagnoza z dlugich przejazdow (>30 s bez kontaktu z tasma): kurs_deg
 * trzyma cel poprawnie w logu, ale fizyczny pojazd i tak znosi w prawo -
 * bo regulator zeruje blad w SAMYM ESTYMATORZE, ktory ma rezydualny,
 * wolno zmienny bias nieuchwycony przez jednorazowa kalibracje na starcie
 * (dryf termiczny/wibracyjny obecny tylko w ruchu). Gdy blad kursu jest
 * maly ORAZ filtrowana predkosc katowa jest mala (ustalona jazda prosto,
 * nie manewr/popchniecie) - prawdziwa predkosc obrotowa POWINNA byc ~0;
 * jesli zyro czyta co innego, to bias - powoli dociagamy s_gyro_bias w
 * jego strone (stala czasowa ~25 s przy 20 Hz, nie zjada prawdziwych
 * korekt). To nie zastepuje Kroku 5 (re-zerowanie na kontakcie z krawedzia
 * - jedyne prawdziwe, bezdryfowe odniesienie) - tylko spowalnia dryf
 * miedzy kontaktami. */
#define BIAS_ADAPT_ERR_MAX      3.0f   /* [°] max |blad kursu| by uznac za ustalona jazde */
#define BIAS_ADAPT_RATE_MAX_DPS 3.0f   /* [°/s] max |gyro_z_filt| by uznac za ustalona jazde (nie manewr) */
#define BIAS_ADAPT_RATE         0.002f /* wspolczynnik adaptacji biasu na takt petli (20 Hz) */

/* --- Krok 3: utrzymanie kursu (regulator P) w ST_CRUISE ---
 * turn = KP_HEADING * blad_kursu + DRIVE_TRIM, ograniczone do +/-TURN_MAX;
 * dodawane roznicowo do kol (turn>0 => skret w LEWO: lewe kolo wolniej).
 * DRIVE_TRIM to staly offset feed-forward kompensujacy konstrukcyjny znos
 * (~1,6 °/s w prawo przy 35 %) - zmierzony w tescie Kroku 3. */
#define KP_HEADING     1.8f     /* [% roznicy mocy / stopien bledu kursu] (Krok 3b: 1.0 -> 1.8) */
#define TURN_MAX      25         /* [%] limit roznicy mocy kol od regulatora */
#define DRIVE_TRIM     3.0f     /* [%] staly offset roznicowy (+ = w lewo); Krok 3b: zmierzony
                                 * turn ustalony ~+3 % kompensujacy konstrukcyjny znos w prawo */

/* --- Krok 4: zadany kurs (cel regulatora) wzgledem kierunku startowego.
 * + = w LEWO (patrz test Kroku 1). Domyslnie HEADING_TARGET_DEFAULT ~ namiar
 * start->meta (cel jest ~18° w lewo od osi toru); ustawiany z dashboardu.
 * Do testow jazdy prosto ustaw 0. Nie zapisywany w NVS. */
#define HEADING_TARGET_DEFAULT   18.0f
#define HEADING_TARGET_MAX       90.0f  /* zakres przycinania [+/-°] */

/* Bufor logu przejazdu (RAM, do pobrania jako CSV). Rekord ~50 B (Krok 5e:
 * +Hall; Krok 6: +korytarz); 1100 * 50 ~= 55 KB - wciaz w granicach zapasu
 * sterty zmierzonego przy Kroku 1 (~62 KB wolne w segmencie DRAM). Przy
 * 300 ms daje ok. 5,5 min nagrywania. */
#define AUTO_LOG_MAX   1100

/* --- Krok 1/6: sektory LIDAR + orientacja glowicy ---
 * LID_MIRROR=1 ustalone w tescie Kroku 1: surowy kat LIDAR rosnie w strone
 * fizycznie PRAWEJ, wiec bez odbicia sektory L/P byly zamienione. Offset
 * przodu (s_lid_front_deg, 0 = przod robota) jest w Kroku 6 nastawialny z
 * dashboardu - kalibracja to iteracja z tools/lidar_map.py. LID_MIRROR
 * zostaje stala (binarne, potwierdzone). */
#define LID_MIRROR         1
#define LID_FRONT_DEG_DEF  0     /* domyslny offset przodu [st] */
static volatile int s_lid_front_deg = LID_FRONT_DEG_DEF;

enum { SEC_FRONT, SEC_FRONT_L, SEC_LEFT, SEC_REAR_L,
       SEC_REAR,  SEC_REAR_R,  SEC_RIGHT, SEC_FRONT_R, SEC_COUNT };
static const int s_sec_center[SEC_COUNT] = {  0,  45,  90, 135, 180, -135, -90, -45 };
static const int s_sec_half[SEC_COUNT]   = { 15,  20,  20,  20,  15,   20,  20,  20 };

/* Min. odleglosc [mm] w luku wzgledem przodu robota; 0 = brak echa (otwarte). */
static uint16_t lidar_arc_mm(int rel_center, int half) {
    int s = LID_MIRROR ? -1 : 1;
    int c = (s_lid_front_deg + s * rel_center) % 360;
    if (c < 0) c += 360;
    return lidar_min_in_arc(c, half);
}

/* --- Krok 6: kontrola korytarza przed przeszkoda (LIDAR) ---
 * W ST_CRUISE, przed regulatorem kursu: jesli korytarz na szerokosc robota
 * na wprost jest zablokowany blizej niz s_front_stop_mm -> STOP (na razie
 * TYLKO zatrzymanie, bez omijania - to Krok 7). Auto-wznowienie, gdy
 * korytarz sie oczysci. Sprawdzamy min. odleglosc na 3 promieniach: srodek
 * + obie krawedzie pojazdu (kat krawedzi liczony geometrycznie z polowy
 * szerokosci + zapasu), zeby lapac tez przeszkody nie idealnie centralne.
 * Prog s_front_stop_mm nastawialny z dashboardu. */
#define VEHICLE_WIDTH_MM        250
#define CORRIDOR_MARGIN_MM       40    /* zapas po kazdej stronie [mm] */
#define CORRIDOR_HALF_MM        (VEHICLE_WIDTH_MM / 2 + CORRIDOR_MARGIN_MM)
#define CORRIDOR_RAY_HALF_DEG    5     /* polowa luku pojedynczego promienia */
#define D_OPEN_MM             4000     /* brak echa => tak daleko (kierunek otwarty) */
#define FRONT_STOP_MM_DEF      300     /* domyslny prog zatrzymania [mm] (dostrojone w tescie Kroku 7) */
#define FRONT_STOP_MM_MIN     150
#define FRONT_STOP_MM_MAX    1500
#define OBSTACLE_DEBOUNCE_COUNT 3      /* takty petli, jak przy linii */
#define DEG_TO_RAD_F          0.017453292f

static volatile int s_front_stop_mm  = FRONT_STOP_MM_DEF;
static volatile int s_corridor_mm    = D_OPEN_MM;   /* ostatnio policzony min. korytarza (telemetria) */

static inline int open_mm(uint16_t v) { return v == 0 ? D_OPEN_MM : (int)v; }

/* Min. odleglosc [mm] w korytarzu na szerokosc robota wokol kierunku
 * center_deg (wzgl. przodu, + = w lewo) przy zadanym lookahead: srodek +
 * obie krawedzie pojazdu (kat krawedzi = atan(polowa_szer/lookahead)).
 * Krok 7: uogolnione o kierunek - ten sam test uzywany do skanu szczelin. */
static int corridor_min_at(int center_deg, int lookahead_mm) {
    if (lookahead_mm < 50) lookahead_mm = 50;
    float edge_rad = atanf((float)CORRIDOR_HALF_MM / (float)lookahead_mm);
    int   edge_deg = (int)(edge_rad / DEG_TO_RAD_F + 0.5f);
    int m = open_mm(lidar_arc_mm(center_deg,            CORRIDOR_RAY_HALF_DEG));
    int l = open_mm(lidar_arc_mm(center_deg + edge_deg, CORRIDOR_RAY_HALF_DEG));
    int r = open_mm(lidar_arc_mm(center_deg - edge_deg, CORRIDOR_RAY_HALF_DEG));
    if (l < m) m = l;
    if (r < m) m = r;
    return m;
}

/* Korytarz na wprost (kierunek 0). */
static int corridor_min_mm(int lookahead_mm) { return corridor_min_at(0, lookahead_mm); }

/* =====================================================================
 *  Krok 7: omijanie przeszkody metoda "follow-the-gap"
 *
 *  Gdy korytarz na wprost jest zablokowany (jak w Kroku 6), zamiast tylko
 *  stac (ST_OBSTACLE) pojazd:
 *    1. ST_AVOID_SCAN  - stojac, skanuje LIDAR-em kierunki wzgledne w luku
 *       +/-s_scan_max_deg co AVOID_SCAN_STEP_DEG i dla kazdego liczy
 *       corridor_min_at() na AVOID_LOOKAHEAD_MM. "Przejezdny" = przeswit
 *       >= s_front_stop_mm + AVOID_CLEAR_MARGIN_MM. Z przejezdnych wybiera
 *       ten NAJBLIZSZY namiarowi na cel (heading_err), remis rozstrzyga
 *       wiekszy przeswit. Brak przejezdnego -> ucieczka z pulapki (Krok 8:
 *       ST_TRAP_BACK / ST_TRAP_TURN / ST_TRAP_ESCAPE), a po jej wyczerpaniu
 *       -> ST_OBSTACLE (stoj).
 *    2. ST_AVOID_TURN  - obrot w miejscu (moc AVOID_TURN_PCT) do kursu
 *       szczeliny (s_avoid_heading_deg, we wspolrzednych estymatora), z
 *       tolerancja AVOID_TURN_TOL_DEG i limitem czasu AVOID_TURN_MAX_MS.
 *    3. ST_AVOID_SETTLE - krotki bezruch (AVOID_SETTLE_MS) po obrocie, zeby
 *       filtr EMA zyra dogonil rzeczywistosc i ustala sie resztkowa rotacja,
 *       zanim kurs szczeliny zostanie "zamrozony" jako cel przejazdu (Krok 7b:
 *       przy szybkim obrocie 50 %/~65 st/s estymator gubil ~10-15 st).
 *    4. ST_AVOID_PASS  - jazda na wprost s_avoid_pass_ms, trzymajac kurs
 *       szczeliny regulatorem P (bez adaptacji biasu), zeby REALNIE minac
 *       przeszkode. Kontrola korytarza dziala dalej: kolejna przeszkoda w
 *       trakcie -> powrot do ST_AVOID_SCAN (do AVOID_MAX_RESCANS razy).
 *    5. Po przejezdzie -> ST_CRUISE. s_heading_target_deg (prawdziwy cel)
 *       nie jest ruszany, wiec regulator sam wraca na namiar start->meta -
 *       ale przez AVOID_RECOVERY_MS z ograniczonym skretem (AVOID_RECOVERY_
 *       TURN_MAX), zeby lagodnie wracac na kurs i NIE ocierac sie o dopiero
 *       co minieta przeszkode (Krok 7b).
 *
 *  Wszystko czasowe (odometria zepsuta). W ST_AVOID_* linia toru ma
 *  priorytet - kontakt przerywa manewr i uruchamia test mety (Hall). */
#define AVOID_SCAN_MAX_DEG_DEF   80
#define AVOID_SCAN_MAX_DEG_MIN   30
#define AVOID_SCAN_MAX_DEG_MAX  120
#define AVOID_SCAN_STEP_DEG      10    /* rozdzielczosc katowa skanu */
#define AVOID_LOOKAHEAD_MM      700    /* zasieg oceny szczeliny [mm] */
#define AVOID_CLEAR_MARGIN_MM   100    /* Krok 7d: wymagany przeswit PONAD prog STOP dla
                                        * "wygodnej" szczeliny [mm] (150 -> 100: przy prog STOP
                                        * 300 mm i przeszkodach ~300-400 mm prog 450 byl
                                        * nieosiagalny -> ciagle falszywe wejscia w ucieczke,
                                        * test_16) */
#define AVOID_CREEP_MARGIN_MM    30    /* Krok 7d: przeswit PONAD prog STOP przy ktorym jedziemy
                                        * ciasna szczelina POWOLI zamiast szukac ucieczki [mm].
                                        * MNIEJSZY niz AVOID_CLEAR_MARGIN - to niższy prog. */
#define AVOID_CREEP_SPEED_NUM     2    /* Krok 7d: predkosc w trybie "creep" = s_speed_pct * 2/3 */
#define AVOID_CREEP_SPEED_DEN     3
#define AVOID_SIDE_STICK_COST  100000  /* Krok 7c: kara w f. kosztu za zmiane strony obejscia
                                        * w trakcie epizodu - praktycznie blokada (strona raz
                                        * obrana obowiazuje do konca epizodu), ale wciaz mozliwe
                                        * przejscie na 2. strone, gdy na obranej nie ma NIC. */
#define AVOID_TURN_TOL_DEG        6.0f /* dopuszczalny blad kursu po obrocie */
#define AVOID_TURN_PCT           45    /* Krok 7b: 50 -> 45 - wolniejszy obrot (mniejszy blad zyra),
                                        * ale wciaz z zapasem momentu na czysty obrot w miejscu */
#define AVOID_TURN_MAX_MS      4500    /* limit czasu obrotu - inaczej STOP */
#define AVOID_SETTLE_MS         300    /* Krok 7b: bezruch po obrocie, przed przejazdem */
#define AVOID_SETTLE_MIN_DEG    25     /* Krok 7b: ponizej tego obrotu nie ma po co ustalac
                                        * (krotki obrot = maly blad zyra) - jedziemy od razu */
#define AVOID_PASS_MS_DEF      2500    /* czas jazdy "przez szczeline" [ms] */
#define AVOID_PASS_MS_MIN      500
#define AVOID_PASS_MS_MAX     6000
#define AVOID_RECOVERY_MS     1800     /* Krok 7b: okno lagodnego powrotu na kurs po przejezdzie */
#define AVOID_RECOVERY_TURN_MAX  12    /* Krok 7b: limit skretu w tym oknie (norm. TURN_MAX=25) */
#define AVOID_RECOVERY_ERR_MAX 45.0f   /* Krok 7d: powyzej tego bledu kursu w oknie powrotu
                                        * skret NIE jest ograniczany (po obrocie w pulapce trzeba
                                        * odkrecic ~180 st - lagodny limit tylko wydluzal bladzenie) */
#define AVOID_MAX_RESCANS        4     /* ile razy w jednym epizodzie ponawiac skan */
#define AVOID_RESCAN_INTERVAL_MS 1200 /* w ST_OBSTACLE: co ile ponawiac skan */

static volatile int s_scan_max_deg  = AVOID_SCAN_MAX_DEG_DEF;
static volatile int s_avoid_pass_ms = AVOID_PASS_MS_DEF;
static float        s_avoid_heading_deg = 0.0f;  /* kurs szczeliny (wspolrzedne estymatora) */
static int          s_avoid_rescans = 0;         /* licznik ponowien skanu w epizodzie */
static int          s_avoid_last_side = 0;        /* Krok 7b: strona ostatniego obejscia: -1=P, +1=L, 0=brak */
static bool         s_avoid_turn_big = false;     /* Krok 7b: czy obrot wymaga fazy ustalania kursu */
static bool         s_avoid_creep = false;        /* Krok 7d: biezacy przejazd w trybie "creep" (wolno) */
static uint32_t     s_avoid_recovery_until = 0;   /* Krok 7b: ms, do kiedy ograniczony skret w ST_CRUISE */

/* =====================================================================
 *  Krok 8: ucieczka z pulapki (U / L / slepy naroznik)
 *
 *  Gdy skan szczelin (ST_AVOID_SCAN) nie znajduje NIC przejezdnego w
 *  +/-s_scan_max_deg, zamiast od razu stac (ST_OBSTACLE):
 *   1. ST_TRAP_BACK  - cofnij sie TRAP_BACK_MS (o ile z tylu jest wolne,
 *      TRAP_BACK_MIN_REAR_MM) i skanuj ponownie z dalszej pozycji - sciany
 *      pulapki obejmuja wtedy mniejszy kat i szczelina czesto sie pojawia.
 *      Do TRAP_MAX_BACKUPS prob.
 *   2. ST_TRAP_TURN  - dalej nic: pelny obrot LIDAR-em (co TRAP_SWEEP_STEP_DEG
 *      na 360 st) -> najszerszy otwarty kierunek (zwykle "usta" pulapki,
 *      czesto z TYLU, gdzie brak echa = D_OPEN_MM). Obrot w miejscu do niego.
 *   3. ST_TRAP_ESCAPE - jazda tym kursem TRAP_ESCAPE_MS (kontrola korytarza
 *      dalej aktywna: przeszkoda -> ST_AVOID_SCAN). Potem ST_CRUISE z DLUGIM
 *      oknem lagodnego powrotu (TRAP_RECOVERY_MS) i ustawiona strona obejscia
 *      = strona ucieczki, zeby regulator wracal na cel LUKIEM obok pulapki,
 *      a nie prosto w nia.
 *  Jesli i to nie pomoze (TRAP_MAX_ESCAPES prob) -> ST_OBSTACLE (jak Krok
 *  6/7: stoj, skanuj co 1,2 s - czekaj az scena sie zmieni). Linia toru
 *  dalej priorytetowo. Wszystko czasowe (odometria zepsuta). */
#define TRAP_BACK_MS           1200   /* czas cofania przed ponownym skanem */
#define TRAP_BACK_MIN_REAR_MM   350   /* nie cofaj sie, jesli z tylu blizej niz to */
#define TRAP_MAX_BACKUPS          3
#define TRAP_SWEEP_STEP_DEG      15    /* rozdzielczosc pelnego obrotu LIDAR */
#define TRAP_SWEEP_ARC_HALF      12    /* polowa luku pojedynczego promienia sweep */
#define TRAP_OPEN_MIN_MM        900   /* najszerszy kierunek musi miec chociaz tyle */
#define TRAP_TURN_TOL_DEG         8.0f
#define TRAP_TURN_MAX_MS       6000    /* obrot do "ust" moze byc az ~180 st */
#define TRAP_ESCAPE_MS         3500    /* jazda na wyjscie - dluzej niz zwykly przejazd */
#define TRAP_RECOVERY_MS       3500    /* dlugie okno lagodnego powrotu po ucieczce */
#define TRAP_MAX_ESCAPES         3     /* po tylu nieudanych probach -> ST_OBSTACLE */

static int      s_trap_backups = 0;
static int      s_trap_escapes = 0;

/* Stany maszyny sterujacej. */
typedef enum {
    ST_IDLE,          /* wylaczony / bezczynny */
    ST_GYRO_CAL,      /* start przejazdu - bezruch, usrednianie biasu gyro_z (Krok 2) */
    ST_CRUISE,        /* jazda na wprost */
    ST_OBSTACLE,      /* przeszkoda w korytarzu - STOP, brak szczeliny / czeka (Krok 6/7) */
    ST_AVOID_SCAN,    /* Krok 7: skan LIDAR-em w poszukiwaniu przejezdnej szczeliny */
    ST_AVOID_TURN,    /* Krok 7: obrot w miejscu do kursu wybranej szczeliny */
    ST_AVOID_SETTLE,  /* Krok 7b: bezruch po obrocie - ustalenie estymatora kursu */
    ST_AVOID_PASS,    /* Krok 7: jazda przez szczeline, by minac przeszkode */
    ST_TRAP_BACK,     /* Krok 8: brak szczeliny - cofnij sie i skanuj ponownie */
    ST_TRAP_TURN,     /* Krok 8: obrot w miejscu ku najszerszemu otwartemu kierunkowi */
    ST_TRAP_ESCAPE,   /* Krok 8: jazda na wyjscie z pulapki */
    ST_LINE_WAIT,     /* linia wykryta - postoj i oczekiwanie na potwierdzenie mety Hallem */
    ST_LINE_BACKUP,   /* to nie meta - cofanie */
    ST_LINE_PAUSE,    /* krotki postoj po cofnieciu, przed ponowna proba jazdy */
    ST_STOP_FINISH,   /* meta potwierdzona Hallem - stop ostateczny */
} st_t;

static volatile bool s_enabled = false;
static volatile st_t s_state   = ST_IDLE;
static TaskHandle_t  s_task     = NULL;

static uint32_t s_state_t = 0;   /* ms wejscia w biezacy stan */
static bool     s_stopped = false;
static uint32_t s_log_t   = 0;   /* ms ostatniego szczegolowego logu */

/* Zadany kurs wzgledny dla regulatora utrzymania kursu w ST_CRUISE
 * (Krok 3/4). + = w lewo. Ustawiany z dashboardu (autonomy_set_heading_target_deg). */
static volatile float s_heading_target_deg = HEADING_TARGET_DEFAULT;

/* Krok 5b: strona krawedzi ostatniego kontaktu z tasma (patrz classify_edge_side). */
static edge_side_t s_line_hit_side = EDGE_SIDE_UNKNOWN;

/* Krok 5d: liczniki debounce (patrz LINE_DEBOUNCE_COUNT/HALL_DEBOUNCE_COUNT). */
static int s_line_debounce = 0;
static int s_hall_debounce = 0;

/* Krok 6: liczniki debounce dla wykrycia/oczyszczenia przeszkody w korytarzu. */
static int s_obst_block_count = 0;
static int s_obst_clear_count = 0;

/* Moc silnikow [%] przy jezdzie na wprost/do tylu - ustawiana z
 * dashboardu (patrz autonomy_set_speed_pct()). */
static volatile int s_speed_pct = SPEED_PCT_DEFAULT;

/* --- Krok 2: estymator kursu z zyroskopu (nadal NIE uzywany w sterowaniu -
 * to Krok 3; na razie tylko obserwowalnosc). Calkowanie (gyro_z_filt - bias)
 * z rzeczywistym dt (z now_ms). Liczone w petli NIEZALEZNIE od s_enabled,
 * zeby dalo sie je czytac z dashboardu bez ruszania silnikow; kurs zerowany
 * i bias mierzony na starcie kazdego przejazdu (ST_GYRO_CAL). */
static volatile float s_heading_deg   = 0.0f;   /* scalkowany kurs wzgledny [°], (-180,180] */
static volatile float s_gyro_z_dps    = 0.0f;   /* ostatni surowy gyro_z [°/s] */
static volatile float s_gyro_z_filt   = 0.0f;   /* gyro_z po EMA [°/s] */
static volatile float s_gyro_bias     = 0.0f;   /* zmierzony bias gyro_z [°/s] (0 przed 1. kalibracja) */
static uint32_t       s_head_last_ms  = 0;

/* Akumulatory kalibracji biasu (ST_GYRO_CAL). */
static float    s_bias_sum = 0.0f;
static int      s_bias_n   = 0;
static float    s_bias_min = 0.0f;
static float    s_bias_max = 0.0f;

/* Chwila zakonczenia kalibracji zyra (pierwsze wejscie w ST_CRUISE w danym
 * przejezdzie) - punkt odniesienia dla START_LINE_IGNORE_MS. Ustawiane raz
 * na przejazd, NIE przy kazdym powrocie do ST_CRUISE po odbiciu. */
static uint32_t s_run_cruise_start_ms = 0;

/* Log przejazdu w RAM (eksportowany jako CSV). */
static autonomy_log_rec_t s_log[AUTO_LOG_MAX];
static uint32_t s_log_n = 0;
static uint32_t s_run_t0 = 0;
static bool     s_log_full_warned = false;

/* Czas trwania i zuzyta energia biezacego/ostatniego przejazdu. */
static uint32_t s_run_elapsed_ms = 0;
static float    s_run_energy_mwh = 0.0f;

static const char *state_name(st_t s) {
    switch (s) {
        case ST_IDLE:        return "Bezczynny";
        case ST_GYRO_CAL:    return "Kalibracja zyroskopu";
        case ST_CRUISE:      return "Jazda";
        case ST_OBSTACLE:    return "Przeszkoda (stop)";
        case ST_AVOID_SCAN:  return "Omijanie - skan szczelin";
        case ST_AVOID_TURN:  return "Omijanie - obrot";
        case ST_AVOID_SETTLE: return "Omijanie - ustalanie kursu";
        case ST_AVOID_PASS:  return "Omijanie - przejazd";
        case ST_TRAP_BACK:   return "Pulapka - cofanie";
        case ST_TRAP_TURN:   return "Pulapka - obrot ku wyjsciu";
        case ST_TRAP_ESCAPE: return "Pulapka - wyjscie";
        case ST_LINE_WAIT:   return "Linia - sprawdzam mete";
        case ST_LINE_BACKUP: return "Cofanie (nie meta)";
        case ST_LINE_PAUSE:  return "Postoj po cofnieciu";
        case ST_STOP_FINISH: return "Zatrzymany (meta)";
        default:             return "?";
    }
}

static inline uint32_t now_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
static inline void enter(st_t s) { s_state = s; s_state_t = now_ms(); }

/* Krok 3: blad kursu wzgledem dowolnego zadanego kursu, znormalizowany do
 * (-180,180]. Dodatni => zadany kurs jest w lewo od biezacego => skret w lewo. */
static inline float heading_err_of(float target_deg) {
    float e = target_deg - s_heading_deg;
    while (e > 180.0f)   e -= 360.0f;
    while (e <= -180.0f) e += 360.0f;
    return e;
}

/* Blad wzgledem prawdziwego celu przejazdu (regulator ST_CRUISE). */
static inline float heading_err(void) { return heading_err_of(s_heading_target_deg); }

/* Zamraza czas trwania biezacego przejazdu (dashboard ma pokazywac czas
 * do tego momentu, nie licznik biegnacy dalej). Wolane tylko przy
 * s_enabled==true, wiec nadpisuje raz na przejazd. */
static inline void stop_run(void) { s_run_elapsed_ms = now_ms() - s_run_t0; }

/* --- Krok 1: migawka telemetrii ---
 * Zbierana raz na takt logu i uzywana zarowno do rekordu CSV, jak i do
 * linii statusu w konsoli - zeby nie liczyc luku LIDAR / nie czytac
 * czujnikow dwa razy. Wszystkie odczyty sa nieblokujace (dane z cache). */
typedef struct {
    imu_data_t         imu;
    line_sensor_data_t line;
    int8_t             hall_do;    /* surowy stan pinu DO cyfrowego Halla mety (0/1) */
    bool               hall_hit;   /* meta wg polaryzacji DO (hall_finish_detected) */
    int16_t            sec_mm[SEC_COUNT];
    int16_t            corridor_mm; /* Krok 6: min. korytarza na wprost [mm] */
    float              heading_deg;
    float              gyro_z_filt;
} telem_t;

static telem_t telem_snapshot(void) {
    telem_t t;
    t.imu     = imu_get_last();
    t.line    = line_sensor_read();
    t.hall_do  = (int8_t)hall_finish_do_raw();
    t.hall_hit = hall_finish_detected();
    for (int i = 0; i < SEC_COUNT; i++)
        t.sec_mm[i] = (int16_t)lidar_arc_mm(s_sec_center[i], s_sec_half[i]);
    t.corridor_mm = (int16_t)corridor_min_mm(s_front_stop_mm);
    t.heading_deg = s_heading_deg;
    t.gyro_z_filt = s_gyro_z_filt;
    return t;
}

/* Dopisuje jeden rekord do logu przejazdu (RAM). Wolane co STATUS_LOG_MS. */
static void record_sample(uint32_t now, pyrometer_data_t pd, const telem_t *tl) {
    if (s_log_n >= AUTO_LOG_MAX) {
        if (!s_log_full_warned) {
            s_log_full_warned = true;
            ESP_LOGW(TAG, "Bufor logu przejazdu pelny (%d rekordow) - dalsze probki odrzucane.",
                     AUTO_LOG_MAX);
        }
        return;
    }
    autonomy_log_rec_t *r = &s_log[s_log_n++];
    r->t_ms         = now - s_run_t0;
    r->state        = (uint8_t)s_state;
    r->motor_l      = (int8_t)motor_get_left_speed();
    r->motor_r      = (int8_t)motor_get_right_speed();
    r->gyro_x_x10   = (int16_t)(tl->imu.gyro_x * 10.0f);
    r->gyro_y_x10   = (int16_t)(tl->imu.gyro_y * 10.0f);
    r->gyro_z_x10   = (int16_t)(tl->imu.gyro_z * 10.0f);
    r->gyro_zf_x10  = (int16_t)(tl->gyro_z_filt * 10.0f);
    r->gyro_bias_x10 = (int16_t)(s_gyro_bias * 10.0f);
    r->heading_x10  = (int16_t)(tl->heading_deg * 10.0f);
    for (int i = 0; i < SEC_COUNT; i++) r->lidar_mm[i] = tl->sec_mm[i];
    r->corridor_mm  = tl->corridor_mm;
    r->line_fr_mv   = (int16_t)(tl->line.front_right_v * 1000.0f);
    r->line_fl_mv   = (int16_t)(tl->line.front_left_v  * 1000.0f);
    r->line_bl_mv   = (int16_t)(tl->line.back_left_v   * 1000.0f);
    r->line_br_mv   = (int16_t)(tl->line.back_right_v  * 1000.0f);
    r->hall_do      = tl->hall_do;
    r->hall_hit     = tl->hall_hit ? 1 : 0;
    r->obj_temp_x10 = (int16_t)(pd.object_temp  * 10.0f);
    r->amb_temp_x10 = (int16_t)(pd.ambient_temp * 10.0f);
}

/* Wykrycie linii toru na potrzeby autonomii - wszystkie CZTERY czujniki
 * odbiciowe CNY70 sa teraz analogowe przez ADS1115 (A0 przod-prawy, A1
 * przod-lewy, A2 tyl-lewy, A3 tyl-prawy). Dawny cyfrowy przod-prawy z GPIO
 * (i jego falszywe odczyty) zniknal razem z przeniesieniem czujnika Halla
 * mety na wyjscie cyfrowe - kanal A0 zwolnil sie dla pelnoprawnego czujnika
 * linii, wiec przod-prawy wraca do uwzgledniania (poprawia wykrycie krawedzi
 * z przodu, ktore wczesniej "widzialo" tylko lewy rog). */
static inline bool track_line_detected(void) {
    line_sensor_data_t d = line_sensor_read();
    return d.front_left || d.front_right || d.back_left || d.back_right;
}

/* Krok 5b: patrz definicja EDGE_TURN_AWAY_DEG - klasyfikacja strony
 * krawedzi z wzorca czujnikow linii, ktore zareagowaly. */
static edge_side_t classify_edge_side(line_sensor_data_t d) {
    if (d.back_right && !d.back_left && !d.front_left) return EDGE_SIDE_RIGHT;
    if (d.back_left  && !d.back_right && !d.front_left) return EDGE_SIDE_LEFT;
    return EDGE_SIDE_UNKNOWN;
}

static const char *edge_side_name(edge_side_t s) {
    switch (s) {
        case EDGE_SIDE_LEFT:  return "lewa";
        case EDGE_SIDE_RIGHT: return "prawa";
        default:              return "nieznana";
    }
}

/* Krok 5d/7: detekcja linii z debounce (wspolna dla ST_CRUISE i stanow
 * omijania). Zwraca true dopiero po LINE_DEBOUNCE_COUNT kolejnych taktach z
 * linia; aktualizuje s_line_debounce (zerowany, gdy linii brak). */
static bool line_confirmed(void) {
    if (track_line_detected()) {
        s_line_debounce++;
    } else {
        s_line_debounce = 0;
    }
    if (s_line_debounce >= LINE_DEBOUNCE_COUNT) {
        s_line_debounce = 0;
        return true;
    }
    return false;
}

/* Krok 7: skan szczelin. Przeglada kierunki wzgledne w luku +/-s_scan_max_deg
 * co AVOID_SCAN_STEP_DEG; kierunek jest "przejezdny", gdy corridor_min_at()
 * na AVOID_LOOKAHEAD_MM daje przeswit >= s_front_stop_mm + AVOID_CLEAR_MARGIN_MM.
 * Z przejezdnych wybiera najnizszy koszt: |rel - namiar|*100 - przeswit/10.
 *
 * Krok 7b: jesli wprost jest juz przejezdnie - jedziemy wprost (0) bez
 * obracania.
 *
 * Krok 7c: "namiar" to:
 *   - PIERWSZY skan epizodu (s_avoid_last_side == 0): rzeczywisty namiar na
 *     cel (heading_err) - decyduje, w ktora strone obchodzimy przeszkode;
 *   - KOLEJNE skany (re-skan w trakcie przejazdu, s_avoid_last_side != 0):
 *     0, czyli "na wprost wzgledem biezacego kursu" - nic sie nie przerzuca
 *     do celu, tylko przewlekamy sie do przodu po OBRANEJ stronie (kara
 *     AVOID_SIDE_STICK_COST praktycznie blokuje przejscie na 2. strone).
 *     Rzeczywisty cel odzyskujemy dopiero w oknie powrotu po przejezdzie.
 *   Bez tego estymator kursu (rozhustany po szybkich obrotach) rozhustywal
 *     tez wybor strony -> migotanie L/P miedzy skanami (test_10).
 *
 * Krok 7d: prog przeswitu "przejezdnosci" przekazywany jest jako need_mm -
 * ST_AVOID_SCAN wola najpierw z progiem "wygodnym" (STOP + AVOID_CLEAR_MARGIN),
 * a gdy nic nie przejdzie - z progiem "creep" (STOP + AVOID_CREEP_MARGIN) i
 * jedzie wtedy wolniej. Dopiero gdy i to zawiedzie -> ucieczka z pulapki.
 *
 * Zwraca kierunek wzgledny [st] (+ = w lewo) albo INT_MIN, gdy zaden nie
 * przechodzi. */
static int avoid_pick_gap(int need_mm) {
    int need = need_mm;

    /* Krok 7b: wprost wolne -> nie obracaj sie po nic. */
    if (corridor_min_at(0, AVOID_LOOKAHEAD_MM) >= need) return 0;

    int goal;
    if (s_avoid_last_side == 0) {              /* Krok 7c: pierwszy skan epizodu */
        goal = (int)lroundf(heading_err());
        if (goal >  s_scan_max_deg) goal =  s_scan_max_deg;
        if (goal < -s_scan_max_deg) goal = -s_scan_max_deg;
    } else {
        goal = 0;                             /* Krok 7c: re-skan - trzymaj sie przodu i strony */
    }

    int  best_rel  = INT_MIN;
    long best_cost = LONG_MAX;
    for (int rel = -s_scan_max_deg; rel <= s_scan_max_deg; rel += AVOID_SCAN_STEP_DEG) {
        int clr = corridor_min_at(rel, AVOID_LOOKAHEAD_MM);
        if (clr < need) continue;
        /* Blizej namiaru = duzo wazniejsze (x100); przeswit tylko jako remis. */
        long cost = (long)abs(rel - goal) * 100 - clr / 10;
        /* Krok 7b/7c: trzymaj sie raz obranej strony obejscia. */
        if (s_avoid_last_side < 0 && rel > 0) cost += AVOID_SIDE_STICK_COST;
        if (s_avoid_last_side > 0 && rel < 0) cost += AVOID_SIDE_STICK_COST;
        if (cost < best_cost) { best_cost = cost; best_rel = rel; }
    }
    return best_rel;
}

/* Krok 8: najszerszy otwarty kierunek w pelnym obrocie (co TRAP_SWEEP_STEP_DEG).
 * Zwraca kat wzgledny [st] (+ = w lewo), a przez out_mm - przeswit w nim.
 * Brak echa (open_mm -> D_OPEN_MM) liczy sie jako maksymalnie otwarte, wiec
 * "usta" pulapki (gdzie nie ma sciany) wygrywaja. */
static int trap_open_dir(int *out_mm) {
    int best_deg = 0, best_mm = -1;
    for (int d = -180; d < 180; d += TRAP_SWEEP_STEP_DEG) {
        int m = open_mm(lidar_arc_mm(d, TRAP_SWEEP_ARC_HALF));
        if (m > best_mm) { best_mm = m; best_deg = d; }
    }
    if (out_mm) *out_mm = best_mm;
    return best_deg;
}

/* Krok 2: rozpoczyna kalibracje biasu gyro_z - zeruje akumulatory i kurs,
 * wchodzi w ST_GYRO_CAL (silniki stoja, patrz obsluga stanu). Czujnik Halla
 * mety jest teraz cyfrowy z progiem sprzetowym (potencjometr), wiec nie ma
 * juz kalibracji napiecia spoczynkowego. */
static void start_gyro_cal(void) {
    s_bias_sum      = 0.0f;
    s_bias_n        = 0;
    s_bias_min      =  1e9f;
    s_bias_max      = -1e9f;
    s_heading_deg   = 0.0f;
    s_line_hit_side = EDGE_SIDE_UNKNOWN;
    s_line_debounce = 0;
    s_hall_debounce = 0;
    s_obst_block_count = 0;
    s_obst_clear_count = 0;
    s_avoid_rescans    = 0;
    s_avoid_last_side  = 0;
    s_avoid_creep      = false;
    s_avoid_recovery_until = 0;
    s_trap_backups     = 0;
    s_trap_escapes     = 0;
    enter(ST_GYRO_CAL);
}

/* Konczy przejazd: zatrzymuje silniki, zamraza czas, wylacza autonomie i
 * ustawia stan koncowy (widoczny na dashboardzie do nastepnego startu). */
static void finish_run(st_t stop_state, const char *reason) {
    ESP_LOGI(TAG, "%s - zatrzymuje pojazd i koncze przejazd.", reason);
    motor_stop();
    enter(stop_state);
    stop_run();
    s_enabled = false;
}

/* Glowna petla sterowania autonomii. */
static void autonomy_task(void *arg) {
    (void)arg;

    while (1) {
        /* --- Krok 2: estymator kursu z zyroskopu - liczony ZAWSZE, takze przy
         * wylaczonej autonomii (obserwowalnosc bez ruszania silnikow).
         * Filtr EMA na predkosc katowa (szum od silnikow), potem calkowanie
         * (gyro_z_filt - bias) z rzeczywistym dt. Podczas ST_GYRO_CAL kursu
         * nie calkujemy (bias jeszcze nieznany) - filtr biegnie dalej. */
        {
            uint32_t   hnow = now_ms();
            imu_data_t im   = imu_get_last();
            s_gyro_z_dps  = im.gyro_z;
            s_gyro_z_filt += HEADING_LPF_ALPHA * (im.gyro_z - s_gyro_z_filt);
            if (s_head_last_ms != 0 && s_state != ST_GYRO_CAL) {
                float dt = (hnow - s_head_last_ms) / 1000.0f;
                if (dt > 0.0f && dt < HEADING_DT_MAX_S) {
                    float h = s_heading_deg
                            + HEADING_SIGN * (s_gyro_z_filt - s_gyro_bias) * dt;
                    while (h > 180.0f)   h -= 360.0f;
                    while (h <= -180.0f) h += 360.0f;
                    s_heading_deg = h;
                }
            }
            s_head_last_ms = hnow;
        }

        /* Tryb wylaczony: pilnuj, by silniki staly. Stan koncowy
         * ST_STOP_FINISH zostaje widoczny na dashboardzie do ponownego
         * wlaczenia (zeby bylo wiadomo, ze pojazd dojechal do mety);
         * stany przejsciowe kasujemy do "Bezczynny". */
        if (!s_enabled) {
            if (s_state != ST_STOP_FINISH) s_state = ST_IDLE;
            if (!s_stopped) { motor_stop(); s_stopped = true; }
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }
        s_stopped = false;
        uint32_t now = now_ms();

        pyrometer_data_t pd = pyrometer_get_last();
        ina219_data_t    pw = ina219_get_last();

        /* Calkowanie energii: moc [mW] * czas [h] = energia [mWh].
         * Zakladany krok czasowy to LOOP_MS. */
        s_run_energy_mwh += pw.power_mw * (LOOP_MS / 3600000.0f);

        /* Szczegolowy log statusu (konsola) oraz rekord do CSV. */
        if (now - s_log_t >= STATUS_LOG_MS) {
            s_log_t = now;
            telem_t tl = telem_snapshot();
            ESP_LOGI(TAG,
                "[%s] L=%d%% R=%d%% | kurs=%.1f cel=%.1f err=%.1f | gyroZ raw/filt=%.1f/%.1f bias=%.2f /s | "
                "korytarz=%d/prog=%d mm omij_cel=%.1f | LIDAR P/PL/L/TL/T/TP/R/PP=%d/%d/%d/%d/%d/%d/%d/%d mm | "
                "linia PP/PL/TL/TP=%d/%d/%d/%d mV | hallDO=%d wykryto=%d",
                state_name(s_state), motor_get_left_speed(), motor_get_right_speed(),
                tl.heading_deg, s_heading_target_deg, heading_err(),
                tl.imu.gyro_z, tl.gyro_z_filt, s_gyro_bias,
                tl.corridor_mm, s_front_stop_mm, s_avoid_heading_deg,
                tl.sec_mm[SEC_FRONT], tl.sec_mm[SEC_FRONT_L], tl.sec_mm[SEC_LEFT],
                tl.sec_mm[SEC_REAR_L], tl.sec_mm[SEC_REAR], tl.sec_mm[SEC_REAR_R],
                tl.sec_mm[SEC_RIGHT], tl.sec_mm[SEC_FRONT_R],
                (int)(tl.line.front_right_v * 1000.0f), (int)(tl.line.front_left_v * 1000.0f),
                (int)(tl.line.back_left_v * 1000.0f), (int)(tl.line.back_right_v * 1000.0f),
                (int)tl.hall_do, (int)tl.hall_hit);
            record_sample(now, pd, &tl);
        }

        /* Krok 5c: Hall (teraz cyfrowy, hall_finish_detected()) sprawdzany
         * TYLKO w oknie tuz po kontakcie z tasma (ST_LINE_WAIT/ST_LINE_BACKUP/
         * ST_LINE_PAUSE), NIE w trakcie zwyklej jazdy. Magnes lezy pod ta sama
         * tasma co granica toru - bez kontaktu z tasma Hall nie widzi
         * prawdziwej mety, wiec zadzialanie poza tym oknem to niemal na pewno
         * zaklocenie (test_13: przy manewrach o duzym poborze pradu odczyt
         * potrafil "plywac"). Bramkowanie oknem kontaktu + debounce odcina te
         * falszywe trafienia. */
        bool hall_window = (s_state == ST_LINE_WAIT || s_state == ST_LINE_BACKUP || s_state == ST_LINE_PAUSE);
        if (hall_window && hall_finish_detected()) {
            /* Krok 5d: debounce - patrz HALL_DEBOUNCE_COUNT. */
            s_hall_debounce++;
        } else {
            s_hall_debounce = 0;
        }
        if (s_hall_debounce >= HALL_DEBOUNCE_COUNT) {
            s_hall_debounce = 0;
            finish_run(ST_STOP_FINISH, "Meta potwierdzona czujnikiem Halla (okno kontaktu z tasma)");
            vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
            continue;
        }

        switch (s_state) {

        case ST_IDLE:
            /* Nie powinno wystapic przy s_enabled (set_enabled wchodzi
             * prosto w ST_GYRO_CAL) - fallback: zrob kalibracje i jedz. */
            start_gyro_cal();
            break;

        case ST_GYRO_CAL: {
            /* Bezruch - usredniaj gyro_z. Silniki stoja. */
            motor_stop();
            imu_data_t im = imu_get_last();
            s_bias_sum += im.gyro_z;
            s_bias_n++;
            if (im.gyro_z < s_bias_min) s_bias_min = im.gyro_z;
            if (im.gyro_z > s_bias_max) s_bias_max = im.gyro_z;
            if (now - s_state_t >= BIAS_CAL_MS) {
                s_gyro_bias = (s_bias_n > 0) ? (s_bias_sum / (float)s_bias_n) : 0.0f;
                float spread = s_bias_max - s_bias_min;
                if (spread > BIAS_CAL_MAX_SPREAD)
                    ESP_LOGW(TAG, "Kalibracja zyra: duzy rozrzut %.1f °/s (robot sie ruszal?) - "
                                  "bias %.2f moze byc niedokladny.", spread, s_gyro_bias);
                else
                    ESP_LOGI(TAG, "Kalibracja zyra OK: bias gyro_z = %.2f °/s "
                                  "(rozrzut %.1f, %d probek). Kurs wyzerowany, cel=%.1f°.",
                             s_gyro_bias, spread, s_bias_n, s_heading_target_deg);

                s_heading_deg = 0.0f;
                s_run_cruise_start_ms = now;   /* start okna ignorowania tasmy na linii startowej */
                enter(ST_CRUISE);
            }
            break;
        }

        case ST_CRUISE:
            /* Wykrywanie linii jest zawsze "uzbrojone" - takze zaraz po
             * cofnieciu i ponownym najechaniu na te sama tasme. Robione
             * celowo: to ma sluzyc do powtarzalnego sprawdzania odczytow
             * czujnikow odbiciowych (stop -> test Halla -> cofniecie ->
             * ponowny stop na tej samej linii), nie do omijania jej.
             * Wyjatek: pierwsze START_LINE_IGNORE_MS po kalibracji zyra
             * (jednorazowo na starcie przejazdu) - pojazd musi przejechac
             * prosto przez tasme na samej linii startowej, wiec czujniki
             * linii sa w tym oknie ignorowane. */
            {
                bool past_start_window = (now - s_run_cruise_start_ms >= START_LINE_IGNORE_MS);
                /* Krok 5d: debounce - patrz LINE_DEBOUNCE_COUNT. */
                if (past_start_window && track_line_detected()) {
                    s_line_debounce++;
                } else {
                    s_line_debounce = 0;
                }
                if (s_line_debounce >= LINE_DEBOUNCE_COUNT) {
                    s_line_debounce = 0;
                    /* Krok 5b: zapamietaj, ktory czujnik zareagowal - uzyte przy
                     * powrocie do jazdy (ST_LINE_PAUSE), zeby skrecic OD trafionej
                     * krawedzi zamiast slepo wracac na ten sam kurs. */
                    s_line_hit_side = classify_edge_side(line_sensor_read());
                    ESP_LOGI(TAG, "Linia wykryta (strona=%s) - stop, czekam %d ms na potwierdzenie mety (Hall).",
                             edge_side_name(s_line_hit_side), LINE_WAIT_MS);
                    motor_stop();
                    enter(ST_LINE_WAIT);
                    break;
                }
            }

            /* Krok 6/7: kontrola korytarza przed przeszkoda. Aktywna ZAWSZE
             * (nie ma wyjatku startowego jak przy tasmie - przeszkoda to
             * przeszkoda). Debounce jak przy linii - LIDAR na pojedynczym
             * sektorze potrafi skoczyc. Krok 7: po potwierdzeniu przeszkody
             * przechodzimy do skanu szczelin (omijanie), nie do samego STOP. */
            {
                int cmin = corridor_min_mm(s_front_stop_mm);
                s_corridor_mm = cmin;
                if (cmin < s_front_stop_mm) {
                    s_obst_block_count++;
                } else {
                    s_obst_block_count = 0;
                }
                if (s_obst_block_count >= OBSTACLE_DEBOUNCE_COUNT) {
                    s_obst_block_count = 0;
                    s_obst_clear_count = 0;
                    s_avoid_rescans    = 0;
                    s_trap_backups     = 0;   /* Krok 8: nowy epizod - licz proby od zera */
                    s_trap_escapes     = 0;
                    ESP_LOGI(TAG, "Przeszkoda w korytarzu (%d mm < prog %d mm) - szukam szczeliny.",
                             cmin, s_front_stop_mm);
                    motor_stop();
                    enter(ST_AVOID_SCAN);
                    break;
                }
            }

            /* Krok 3: jazda z utrzymaniem zadanego kursu regulatorem P.
             * turn>0 => skret w lewo (lewe kolo wolniej, prawe szybciej).
             * motor_set_* przycina do [-100,100] we wlasnym zakresie. */
            {
                float err = heading_err();

                /* Krok 7b: okno lagodnego powrotu po ominieciu przeszkody -
                 * mocno ograniczony skret (i bez adaptacji biasu), zeby po
                 * przejezdzie wracac na kurs celu szerokim, spokojnym lukiem,
                 * nie ocierajac sie o dopiero co minieta przeszkode. Kontrola
                 * korytarza dziala dalej - realne zblizenie i tak zatrzyma.
                 * Krok 7d: ograniczenie skretu obowiazuje TYLKO przy malym
                 * bledzie kursu; po obrocie w pulapce (blad ~180 st) trzeba
                 * odkrecic pelna moca, inaczej robot dlugo bladzi bokiem. */
                bool in_recovery_window = (s_avoid_recovery_until != 0 && now < s_avoid_recovery_until);
                bool recovering = in_recovery_window && fabsf(err) < AVOID_RECOVERY_ERR_MAX;
                if (!in_recovery_window) s_avoid_recovery_until = 0;

                /* Krok 4b: adaptacja biasu - tylko w ustalonej jezdzie prostej
                 * (maly blad kursu I mala filtrowana predkosc katowa), zeby
                 * nie mylic prawdziwych korekt/manewrow z rezydualnym biasem. */
                if (!recovering &&
                    fabsf(err) < BIAS_ADAPT_ERR_MAX && fabsf(s_gyro_z_filt) < BIAS_ADAPT_RATE_MAX_DPS) {
                    s_gyro_bias += BIAS_ADAPT_RATE * (s_gyro_z_filt - s_gyro_bias);
                }

                float tmax = recovering ? (float)AVOID_RECOVERY_TURN_MAX : (float)TURN_MAX;
                float turn = KP_HEADING * err + DRIVE_TRIM;
                if (turn >  tmax) turn =  tmax;
                if (turn < -tmax) turn = -tmax;
                int td = (int)lroundf(turn);   /* zaokraglenie, nie obciecie - unika martwej strefy +/-1% */
                motor_set_left (s_speed_pct - td);
                motor_set_right(s_speed_pct + td);
            }
            break;

        case ST_OBSTACLE:
            /* Krok 6/7: STOP - brak przejezdnej szczeliny (albo przekroczony
             * limit ponowien w epizodzie omijania). Dwa wyjscia:
             *  - korytarz na wprost sie oczysci (przeszkoda usunieta) -> jazda,
             *  - co AVOID_RESCAN_INTERVAL_MS ponow skan szczelin (przeszkoda
             *    mogla sie zmienic). Ucieczka z pulapek U/L to Krok 8. */
            motor_stop();
            {
                int cmin = corridor_min_mm(s_front_stop_mm);
                s_corridor_mm = cmin;
                if (cmin >= s_front_stop_mm) {
                    s_obst_clear_count++;
                } else {
                    s_obst_clear_count = 0;
                }
                if (s_obst_clear_count >= OBSTACLE_DEBOUNCE_COUNT) {
                    s_obst_clear_count = 0;
                    s_obst_block_count = 0;
                    s_avoid_rescans    = 0;
                    s_avoid_last_side  = 0;
                    ESP_LOGI(TAG, "Korytarz oczyszczony (%d mm >= prog %d mm) - jade dalej.",
                             cmin, s_front_stop_mm);
                    enter(ST_CRUISE);
                    break;
                }
                if (now - s_state_t >= AVOID_RESCAN_INTERVAL_MS) {
                    s_obst_clear_count = 0;
                    s_avoid_rescans    = 0;
                    s_avoid_last_side  = 0;   /* Krok 7c: nowy skan - wolna reka co do strony */
                    ESP_LOGI(TAG, "Przeszkoda nadal (%d mm) - ponawiam skan szczelin.", cmin);
                    enter(ST_AVOID_SCAN);
                }
            }
            break;

        case ST_AVOID_SCAN: {
            /* Krok 7: stojac, wybierz przejezdna szczeline najblizsza celowi.
             * Linia toru ma priorytet - kontakt przerywa omijanie. */
            motor_stop();
            if (line_confirmed()) {
                s_line_hit_side = classify_edge_side(line_sensor_read());
                ESP_LOGI(TAG, "Omijanie: linia w trakcie skanu (strona=%s) - przerywam, test mety.",
                         edge_side_name(s_line_hit_side));
                motor_stop();
                enter(ST_LINE_WAIT);
                break;
            }
            /* Krok 7d: dwustopniowo. Najpierw szukamy "wygodnej" szczeliny
             * (STOP + AVOID_CLEAR_MARGIN). Jak nie ma - szukamy "creep":
             * ciasniejszej (STOP + AVOID_CREEP_MARGIN), przez ktora przejedziemy
             * WOLNIEJ. Dopiero brak i takiej -> ucieczka z pulapki. Bez tego w
             * gestym polu przeszkod (przeswit ~300-400 mm) nic nie przechodzilo
             * i robot ciagle wpadal w ucieczke (test_16). */
            int gap = avoid_pick_gap(s_front_stop_mm + AVOID_CLEAR_MARGIN_MM);
            s_avoid_creep = false;
            if (gap == INT_MIN) {
                gap = avoid_pick_gap(s_front_stop_mm + AVOID_CREEP_MARGIN_MM);
                if (gap != INT_MIN) {
                    s_avoid_creep = true;
                    ESP_LOGI(TAG, "Omijanie: brak wygodnej szczeliny - jade ciasna WOLNO (creep).");
                }
            }
            if (gap == INT_MIN) {
                /* Krok 8: brak szczeliny w +/-scan -> proba wyjscia z pulapki.
                 * Najpierw kilka razy cofnij sie i skanuj z dalszej pozycji;
                 * potem obrot ku najszerszemu otwartemu kierunkowi ("usta"
                 * pulapki) i jazda na wyjscie. Po TRAP_MAX_ESCAPES nieudanych
                 * probach - STOP (ST_OBSTACLE) jak w Kroku 7. */
                if (s_trap_escapes >= TRAP_MAX_ESCAPES) {
                    ESP_LOGW(TAG, "Pulapka: %d prob wyjscia bez efektu - STOP.", s_trap_escapes);
                    enter(ST_OBSTACLE);
                    break;
                }
                if (s_trap_backups < TRAP_MAX_BACKUPS) {
                    int rear = open_mm(lidar_arc_mm(180, 20));
                    if (rear >= TRAP_BACK_MIN_REAR_MM) {
                        s_trap_backups++;
                        ESP_LOGI(TAG, "Pulapka: brak szczeliny - cofam sie (%d/%d), tyl wolny %d mm.",
                                 s_trap_backups, TRAP_MAX_BACKUPS, rear);
                        enter(ST_TRAP_BACK);
                        break;
                    }
                    ESP_LOGI(TAG, "Pulapka: brak szczeliny, tyl zablokowany (%d mm) - obrot ku wyjsciu.", rear);
                } else {
                    ESP_LOGI(TAG, "Pulapka: %d prob cofania bez efektu - obrot ku wyjsciu.", s_trap_backups);
                }
                /* Krok 8: najszerszy otwarty kierunek w pelnym obrocie -> cel obrotu. */
                {
                    int open_here = 0;
                    int bdeg = trap_open_dir(&open_here);
                    if (open_here < TRAP_OPEN_MIN_MM) {
                        ESP_LOGW(TAG, "Pulapka: brak wyraznie otwartego kierunku (max %d mm w %+d st) - STOP.",
                                 open_here, bdeg);
                        s_trap_escapes++;
                        enter(ST_OBSTACLE);
                        break;
                    }
                    s_avoid_heading_deg = s_heading_deg + (float)bdeg;
                    while (s_avoid_heading_deg > 180.0f)   s_avoid_heading_deg -= 360.0f;
                    while (s_avoid_heading_deg <= -180.0f) s_avoid_heading_deg += 360.0f;
                    /* strona ucieczki - trzymanie sie jej w pozniejszym omijaniu (Krok 7c) */
                    if      (bdeg >  20 && bdeg <  160) s_avoid_last_side =  1;
                    else if (bdeg < -20 && bdeg > -160) s_avoid_last_side = -1;
                    else                               s_avoid_last_side =  0;
                    ESP_LOGI(TAG, "Pulapka: najszersze wyjscie rel=%+d st (%d mm), kurs docelowy %.1f st - obrot.",
                             bdeg, open_here, s_avoid_heading_deg);
                    enter(ST_TRAP_TURN);
                    break;
                }
            }
            s_avoid_heading_deg = s_heading_deg + (float)gap;
            while (s_avoid_heading_deg > 180.0f)   s_avoid_heading_deg -= 360.0f;
            while (s_avoid_heading_deg <= -180.0f) s_avoid_heading_deg += 360.0f;
            s_obst_block_count = 0;
            /* Krok 7b: zapamietaj strone obejscia (do trzymania sie jej przy
             * kolejnych skanach w tym epizodzie). 0 = szczelina praktycznie
             * na wprost -> pomijamy obrot i ustalanie, jedziemy od razu. */
            if (gap >  AVOID_SCAN_STEP_DEG / 2) s_avoid_last_side =  1;
            else if (gap < -AVOID_SCAN_STEP_DEG / 2) s_avoid_last_side = -1;
            if (fabsf((float)gap) <= AVOID_TURN_TOL_DEG) {
                ESP_LOGI(TAG, "Omijanie: szczelina na wprost (rel=%+d st) - przejazd %d ms.",
                         gap, s_avoid_pass_ms);
                enter(ST_AVOID_PASS);
                break;
            }
            s_avoid_turn_big = (abs(gap) >= AVOID_SETTLE_MIN_DEG);
            ESP_LOGI(TAG, "Omijanie: szczelina rel=%+d st (kurs docelowy %.1f st, przeswit >= %d mm) - obrot.",
                     gap, s_avoid_heading_deg, s_front_stop_mm + AVOID_CLEAR_MARGIN_MM);
            enter(ST_AVOID_TURN);
            break;
        }

        case ST_AVOID_TURN: {
            /* Krok 7: obrot w miejscu do kursu szczeliny. */
            if (line_confirmed()) {
                s_line_hit_side = classify_edge_side(line_sensor_read());
                ESP_LOGI(TAG, "Omijanie: linia w trakcie obrotu (strona=%s) - przerywam, test mety.",
                         edge_side_name(s_line_hit_side));
                motor_stop();
                enter(ST_LINE_WAIT);
                break;
            }
            float err = heading_err_of(s_avoid_heading_deg);
            if (fabsf(err) <= AVOID_TURN_TOL_DEG) {
                motor_stop();
                if (s_avoid_turn_big) {
                    ESP_LOGI(TAG, "Omijanie: obrot zakonczony (kurs %.1f st, blad %.1f st) - ustalanie %d ms.",
                             s_heading_deg, err, AVOID_SETTLE_MS);
                    enter(ST_AVOID_SETTLE);
                } else {
                    ESP_LOGI(TAG, "Omijanie: obrot zakonczony (kurs %.1f st, blad %.1f st) - przejazd %d ms.",
                             s_heading_deg, err, s_avoid_pass_ms);
                    enter(ST_AVOID_PASS);
                }
                break;
            }
            if (now - s_state_t >= AVOID_TURN_MAX_MS) {
                ESP_LOGW(TAG, "Omijanie: obrot nie osiagnal kursu w %d ms (blad %.1f st) - STOP.",
                         AVOID_TURN_MAX_MS, err);
                motor_stop();
                s_avoid_last_side = 0;
                enter(ST_OBSTACLE);
                break;
            }
            int p = AVOID_TURN_PCT;
            if (err > 0.0f) { motor_set_left(-p); motor_set_right( p); }  /* w lewo (CCW) */
            else            { motor_set_left( p); motor_set_right(-p); }  /* w prawo (CW) */
            break;
        }

        case ST_AVOID_SETTLE:
            /* Krok 7b: bezruch po obrocie. Filtr EMA zyra ma staly czasowy
             * rzedu kilku taktow, a po szybkim obrocie przez chwile jeszcze
             * "nadganialby" kurs w trakcie przejazdu, przez co s_avoid_heading_
             * deg jako cel bylby przekrzywiony. Krotki postoj pozwala kursowi
             * sie ustabilizowac. Linia toru dalej priorytetowo. */
            motor_stop();
            if (line_confirmed()) {
                s_line_hit_side = classify_edge_side(line_sensor_read());
                ESP_LOGI(TAG, "Omijanie: linia w trakcie ustalania (strona=%s) - przerywam, test mety.",
                         edge_side_name(s_line_hit_side));
                motor_stop();
                enter(ST_LINE_WAIT);
                break;
            }
            if (now - s_state_t >= AVOID_SETTLE_MS) {
                ESP_LOGI(TAG, "Omijanie: kurs ustalony (%.1f st, cel szczeliny %.1f st) - przejazd %d ms.",
                         s_heading_deg, s_avoid_heading_deg, s_avoid_pass_ms);
                enter(ST_AVOID_PASS);
            }
            break;

        case ST_AVOID_PASS: {
            /* Krok 7: jazda przez szczeline, by REALNIE minac przeszkode.
             * Linia toru ma priorytet; korytarz kontrolowany dalej. */
            if (line_confirmed()) {
                s_line_hit_side = classify_edge_side(line_sensor_read());
                ESP_LOGI(TAG, "Omijanie: linia w trakcie przejazdu (strona=%s) - przerywam, test mety.",
                         edge_side_name(s_line_hit_side));
                motor_stop();
                enter(ST_LINE_WAIT);
                break;
            }
            {
                int cmin = corridor_min_mm(s_front_stop_mm);
                s_corridor_mm = cmin;
                if (cmin < s_front_stop_mm) {
                    s_obst_block_count++;
                } else {
                    s_obst_block_count = 0;
                }
                if (s_obst_block_count >= OBSTACLE_DEBOUNCE_COUNT) {
                    s_obst_block_count = 0;
                    motor_stop();
                    if (++s_avoid_rescans > AVOID_MAX_RESCANS) {
                        ESP_LOGW(TAG, "Omijanie: %d ponowien bez efektu - STOP.", s_avoid_rescans - 1);
                        s_avoid_rescans   = 0;
                        s_avoid_last_side = 0;
                        enter(ST_OBSTACLE);
                    } else {
                        ESP_LOGI(TAG, "Omijanie: przeszkoda w trakcie przejazdu (%d mm) - ponowny skan (%d/%d).",
                                 cmin, s_avoid_rescans, AVOID_MAX_RESCANS);
                        enter(ST_AVOID_SCAN);
                    }
                    break;
                }
            }
            if (now - s_state_t >= (uint32_t)s_avoid_pass_ms) {
                s_avoid_rescans = 0;
                s_avoid_last_side = 0;
                /* Krok 7b: lagodny powrot na kurs celu (ograniczony skret przez
                 * AVOID_RECOVERY_MS) - patrz ST_CRUISE. */
                s_avoid_recovery_until = now + AVOID_RECOVERY_MS;
                ESP_LOGI(TAG, "Omijanie: przejazd zakonczony - lagodny powrot na kurs celu (%.1f st).",
                         s_heading_target_deg);
                enter(ST_CRUISE);
                break;
            }
            {
                float err  = heading_err_of(s_avoid_heading_deg);
                float turn = KP_HEADING * err + DRIVE_TRIM;
                if (turn >  (float)TURN_MAX) turn =  (float)TURN_MAX;
                if (turn < -(float)TURN_MAX) turn = -(float)TURN_MAX;
                int td  = (int)lroundf(turn);
                /* Krok 7d: w trybie "creep" (ciasna szczelina) jedziemy wolniej,
                 * ale nie mniej niz 20% - inaczej naped moze stanac. */
                int spd = s_speed_pct;
                if (s_avoid_creep) {
                    spd = s_speed_pct * AVOID_CREEP_SPEED_NUM / AVOID_CREEP_SPEED_DEN;
                    if (spd < 20) spd = 20;
                }
                motor_set_left (spd - td);
                motor_set_right(spd + td);
            }
            break;
        }

        case ST_TRAP_BACK:
            /* Krok 8: cofanie przed ponownym skanem. Linia toru priorytetowo;
             * dodatkowo pilnujemy, zeby nie wcofac sie w cos z tylu. */
            if (line_confirmed()) {
                s_line_hit_side = classify_edge_side(line_sensor_read());
                ESP_LOGI(TAG, "Pulapka: linia w trakcie cofania (strona=%s) - przerywam, test mety.",
                         edge_side_name(s_line_hit_side));
                motor_stop();
                enter(ST_LINE_WAIT);
                break;
            }
            {
                int rear = open_mm(lidar_arc_mm(180, 20));
                if (rear < TRAP_BACK_MIN_REAR_MM) {
                    ESP_LOGI(TAG, "Pulapka: z tylu zrobilo sie ciasno (%d mm) - koncze cofanie, ponowny skan.",
                             rear);
                    motor_stop();
                    s_avoid_last_side = 0;
                    enter(ST_AVOID_SCAN);
                    break;
                }
            }
            if (now - s_state_t < TRAP_BACK_MS) {
                motor_set_left(-s_speed_pct);
                motor_set_right(-s_speed_pct);
            } else {
                motor_stop();
                s_avoid_last_side = 0;   /* skan z nowej pozycji - wolna reka co do strony */
                ESP_LOGI(TAG, "Pulapka: cofnieto - ponowny skan szczelin.");
                enter(ST_AVOID_SCAN);
            }
            break;

        case ST_TRAP_TURN: {
            /* Krok 8: obrot w miejscu ku najszerszemu otwartemu kierunkowi
             * (cel = s_avoid_heading_deg, policzony w ST_AVOID_SCAN). */
            if (line_confirmed()) {
                s_line_hit_side = classify_edge_side(line_sensor_read());
                ESP_LOGI(TAG, "Pulapka: linia w trakcie obrotu (strona=%s) - przerywam, test mety.",
                         edge_side_name(s_line_hit_side));
                motor_stop();
                enter(ST_LINE_WAIT);
                break;
            }
            float err = heading_err_of(s_avoid_heading_deg);
            if (fabsf(err) <= TRAP_TURN_TOL_DEG) {
                motor_stop();
                s_trap_escapes++;
                ESP_LOGI(TAG, "Pulapka: obrot ku wyjsciu zakonczony (kurs %.1f st) - wyjscie %d ms (proba %d/%d).",
                         s_heading_deg, TRAP_ESCAPE_MS, s_trap_escapes, TRAP_MAX_ESCAPES);
                enter(ST_TRAP_ESCAPE);
                break;
            }
            if (now - s_state_t >= TRAP_TURN_MAX_MS) {
                ESP_LOGW(TAG, "Pulapka: obrot ku wyjsciu nie osiagnal kursu w %d ms - STOP.", TRAP_TURN_MAX_MS);
                motor_stop();
                s_trap_escapes++;
                enter(ST_OBSTACLE);
                break;
            }
            int p = AVOID_TURN_PCT;
            if (err > 0.0f) { motor_set_left(-p); motor_set_right( p); }
            else            { motor_set_left( p); motor_set_right(-p); }
            break;
        }

        case ST_TRAP_ESCAPE: {
            /* Krok 8: jazda na wyjscie z pulapki, trzymajac zamrozony kurs
             * "ust". Kontrola korytarza dalej aktywna - napotkana przeszkoda
             * przelacza w normalne omijanie (ST_AVOID_SCAN). Linia priorytetowo. */
            if (line_confirmed()) {
                s_line_hit_side = classify_edge_side(line_sensor_read());
                ESP_LOGI(TAG, "Pulapka: linia w trakcie wyjscia (strona=%s) - przerywam, test mety.",
                         edge_side_name(s_line_hit_side));
                motor_stop();
                enter(ST_LINE_WAIT);
                break;
            }
            /* Krotkie ustalenie kursu po (mozliwe duzym) obrocie. */
            if (now - s_state_t < AVOID_SETTLE_MS) {
                motor_stop();
                s_avoid_heading_deg = s_heading_deg;
                break;
            }
            {
                int cmin = corridor_min_mm(s_front_stop_mm);
                s_corridor_mm = cmin;
                if (cmin < s_front_stop_mm) {
                    s_obst_block_count++;
                } else {
                    s_obst_block_count = 0;
                }
                if (s_obst_block_count >= OBSTACLE_DEBOUNCE_COUNT) {
                    s_obst_block_count = 0;
                    motor_stop();
                    ESP_LOGI(TAG, "Pulapka: przeszkoda na drodze wyjscia (%d mm) - normalne omijanie.", cmin);
                    enter(ST_AVOID_SCAN);
                    break;
                }
            }
            if (now - s_state_t >= (uint32_t)(AVOID_SETTLE_MS + TRAP_ESCAPE_MS)) {
                s_trap_backups   = 0;
                s_trap_escapes   = 0;
                s_avoid_rescans  = 0;
                s_avoid_last_side = 0;
                s_avoid_recovery_until = now + TRAP_RECOVERY_MS;
                ESP_LOGI(TAG, "Pulapka: wyjscie zakonczone - lagodny powrot na kurs celu (%.1f st).",
                         s_heading_target_deg);
                enter(ST_CRUISE);
                break;
            }
            {
                float err  = heading_err_of(s_avoid_heading_deg);
                float turn = KP_HEADING * err + DRIVE_TRIM;
                if (turn >  (float)TURN_MAX) turn =  (float)TURN_MAX;
                if (turn < -(float)TURN_MAX) turn = -(float)TURN_MAX;
                int td = (int)lroundf(turn);
                motor_set_left (s_speed_pct - td);
                motor_set_right(s_speed_pct + td);
            }
            break;
        }

        case ST_LINE_WAIT:
            /* Stoj i odliczaj. Meta (Hall) obsluzona wyzej, priorytetowo -
             * tutaj rozstrzygamy tylko przypadek "Hall nie zadzialal". */
            motor_stop();
            if (now - s_state_t >= LINE_WAIT_MS) {
                ESP_LOGI(TAG, "Brak sygnalu Hall przez %d ms - to nie meta. Cofam %d ms.",
                         LINE_WAIT_MS, LINE_BACKUP_MS);
                enter(ST_LINE_BACKUP);
            }
            break;

        case ST_LINE_BACKUP:
            if (now - s_state_t < LINE_BACKUP_MS) {
                motor_set_left(-s_speed_pct);
                motor_set_right(-s_speed_pct);
            } else {
                motor_stop();
                enter(ST_LINE_PAUSE);
            }
            break;

        case ST_LINE_PAUSE:
            /* Krotki postoj po cofnieciu, zanim pojazd znow ruszy na wprost. */
            motor_stop();
            if (now - s_state_t >= LINE_POSTBACKUP_PAUSE_MS) {
                /* Krok 5a: re-zerowanie estymatora kursu do zadanej wartosci.
                 * Diagnoza z dlugich przejazdow (dwa pozornie identyczne
                 * przejazdy przy tym samym celu - jeden dojechal do mety,
                 * drugi zniosl w prawo i nie dojechal, mimo ze kurs_deg w obu
                 * logach wygladal podobnie): regulator zeruje blad w SAMYM
                 * estymatorze, ale bledy poprzecznego polozenia (calka bledu
                 * kursu po drodze) nie sa w ogole widoczne w kurs_deg i moga
                 * sie kumulowac miedzy kontaktami z tasma bez ograniczenia.
                 * Kontakt z tasma + cofniecie to jedyny dostepny nam moment
                 * "resetu" - zakladamy, ze pojazd jest z powrotem mniej wiecej
                 * na kursie i czyscimy nagromadzony blad estymatora.
                 *
                 * Krok 5b: dodatkowo, jesli strona kontaktu byla jednoznaczna
                 * (patrz classify_edge_side - zwalidowane logiem
                 * przejazdwzdluzlinii.csv), estymator jest re-zerowany NIE
                 * dokladnie do celu, tylko z odchyleniem "od" trafionej
                 * krawedzi. Regulator P widzi to jako chwilowy blad kursu i
                 * sam wykona skret od krawedzi, po czym w naturalny sposob
                 * wroci do prawdziwego celu w miare jak estymator dogoni
                 * rzeczywistosc - bez zadnej dodatkowej logiki sterowania.
                 * Bez tego pojazd wracal w te sama krawedz po ~1,5-2 s (patrz
                 * log: 3 odbicia pod rzad w ciagu 15 s przy krawedzi po
                 * prawej). Gdy strona nieznana (np. sam przod-lewy) - bez
                 * odchylenia, jak w Kroku 5a. */
                float edge_bias = 0.0f;
                if      (s_line_hit_side == EDGE_SIDE_RIGHT) edge_bias = -EDGE_TURN_AWAY_DEG; /* estymator "za nisko" -> regulator skreca w lewo, od prawej krawedzi */
                else if (s_line_hit_side == EDGE_SIDE_LEFT)  edge_bias =  EDGE_TURN_AWAY_DEG; /* odwrotnie - skret w prawo, od lewej krawedzi */
                ESP_LOGI(TAG, "Postoj po cofnieciu zakonczony - re-zeruje kurs do celu (%.1f°, bylo %.1f°), "
                              "strona=%s, odchylenie=%.0f° - jade dalej.",
                         s_heading_target_deg, s_heading_deg, edge_side_name(s_line_hit_side), edge_bias);
                s_heading_deg   = s_heading_target_deg + edge_bias;
                s_line_hit_side = EDGE_SIDE_UNKNOWN;
                enter(ST_CRUISE);
            }
            break;

        case ST_STOP_FINISH:
        default:
            motor_stop();
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}

void autonomy_init(void) {
    if (s_task) return;
    xTaskCreatePinnedToCore(autonomy_task, "autonomy", 4096, NULL, 6, &s_task, 1);
    ESP_LOGI(TAG, "Modul autonomii gotowy (wylaczony). Predkosc jazdy=%d%%.", s_speed_pct);
}

void autonomy_set_enabled(bool enable) {
    if (enable) {
        s_stopped = false;
        /* Krok 2: najpierw ~1 s bezruchu na pomiar biasu gyro_z, potem jazda. */
        start_gyro_cal();
        /* Nowy przejazd => nowy log (poprzedni, jesli nie pobrany, zostaje nadpisany). */
        s_log_n           = 0;
        s_log_full_warned = false;
        s_run_t0          = now_ms();
        s_run_elapsed_ms  = 0;
        s_run_energy_mwh  = 0.0f;
        s_log_t           = 0;          /* wymus natychmiastowy pierwszy rekord */
        s_enabled = true;
        ESP_LOGI(TAG, "Autonomia WLACZONA - kalibracja zyra (%d ms bezruchu), potem jazda; log wyzerowany.",
                 BIAS_CAL_MS);
    } else {
        /* Zamrazamy czas przejazdu tylko przy realnym przejsciu wl.->wyl.
         * (handler bywa wolany tez, gdy autonomia jest juz wylaczona -
         * np. przy kazdym recznym ruchu silnika, patrz http_server.c). */
        if (s_enabled) stop_run();
        s_enabled = false;
        motor_stop();
        if (s_state != ST_STOP_FINISH) enter(ST_IDLE);
        ESP_LOGI(TAG, "Autonomia WYLACZONA.");
    }
}

bool autonomy_is_enabled(void) { return s_enabled; }

bool autonomy_finish_reached(void) { return s_state == ST_STOP_FINISH; }

const char *autonomy_state_str(void) { return state_name(s_state); }

/* --- Krok 1/2: podglad estymatora kursu i sektorow LIDAR. --- */
float autonomy_get_heading_deg(void)      { return s_heading_deg; }
float autonomy_get_gyro_z_dps(void)       { return s_gyro_z_dps; }   /* surowy */
float autonomy_get_gyro_z_filt_dps(void)  { return s_gyro_z_filt; }
float autonomy_get_gyro_bias_dps(void)    { return s_gyro_bias; }
float autonomy_get_heading_target_deg(void) { return s_heading_target_deg; }

void autonomy_set_heading_target_deg(float deg) {
    if (deg >  HEADING_TARGET_MAX) deg =  HEADING_TARGET_MAX;
    if (deg < -HEADING_TARGET_MAX) deg = -HEADING_TARGET_MAX;
    s_heading_target_deg = deg;
    ESP_LOGI(TAG, "Zadany kurs (cel regulatora) = %.1f° (+ = w lewo).", deg);
}

void autonomy_get_lidar_sectors_mm(int16_t out[8]) {
    for (int i = 0; i < SEC_COUNT; i++)
        out[i] = (int16_t)lidar_arc_mm(s_sec_center[i], s_sec_half[i]);
}

/* --- Krok 6: kalibracja/parametry LIDAR (nastawialne z dashboardu). --- */
int autonomy_get_lid_front_deg(void)  { return s_lid_front_deg; }
int autonomy_get_front_stop_mm(void)  { return s_front_stop_mm; }
int autonomy_get_corridor_mm(void)    { return s_corridor_mm; }

void autonomy_set_lid_front_deg(int deg) {
    deg %= 360;
    if (deg < 0) deg += 360;
    s_lid_front_deg = deg;
    ESP_LOGI(TAG, "Offset przodu LIDAR = %d st.", deg);
}

void autonomy_set_front_stop_mm(int mm) {
    if (mm < FRONT_STOP_MM_MIN) mm = FRONT_STOP_MM_MIN;
    if (mm > FRONT_STOP_MM_MAX) mm = FRONT_STOP_MM_MAX;
    s_front_stop_mm = mm;
    ESP_LOGI(TAG, "Prog STOP przed przeszkoda = %d mm.", mm);
}

/* --- Krok 7: parametry omijania (nastawialne z dashboardu). --- */
int autonomy_get_scan_max_deg(void)  { return s_scan_max_deg; }
int autonomy_get_avoid_pass_ms(void) { return s_avoid_pass_ms; }

void autonomy_set_scan_max_deg(int deg) {
    if (deg < AVOID_SCAN_MAX_DEG_MIN) deg = AVOID_SCAN_MAX_DEG_MIN;
    if (deg > AVOID_SCAN_MAX_DEG_MAX) deg = AVOID_SCAN_MAX_DEG_MAX;
    s_scan_max_deg = deg;
    ESP_LOGI(TAG, "Zakres skanu szczelin = +/-%d st.", deg);
}

void autonomy_set_avoid_pass_ms(int ms) {
    if (ms < AVOID_PASS_MS_MIN) ms = AVOID_PASS_MS_MIN;
    if (ms > AVOID_PASS_MS_MAX) ms = AVOID_PASS_MS_MAX;
    s_avoid_pass_ms = ms;
    ESP_LOGI(TAG, "Czas jazdy przez szczeline = %d ms.", ms);
}

void autonomy_set_speed_pct(int pct) {
    if (pct < SPEED_PCT_MIN) pct = SPEED_PCT_MIN;
    if (pct > SPEED_PCT_MAX) pct = SPEED_PCT_MAX;
    s_speed_pct = pct;
    ESP_LOGI(TAG, "Predkosc jazdy autonomicznej (na wprost/do tylu) ustawiona na %d%%.", pct);
}

int autonomy_get_speed_pct(void) { return s_speed_pct; }

float autonomy_get_run_time_s(void) {
    uint32_t ms = s_enabled ? (now_ms() - s_run_t0) : s_run_elapsed_ms;
    return (float)ms / 1000.0f;
}

float autonomy_get_run_energy_mwh(void) { return s_run_energy_mwh; }

/* API logu przejazdu. */
uint32_t autonomy_log_count(void) { return s_log_n; }

bool autonomy_log_get(uint32_t idx, autonomy_log_rec_t *out) {
    if (!out || idx >= s_log_n) return false;
    *out = s_log[idx];
    return true;
}

const char *autonomy_log_state_name(uint8_t state) { return state_name((st_t)state); }
