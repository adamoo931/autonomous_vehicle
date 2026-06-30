# Autonomiczny pojazd (ESP32 / ESP-IDF)

Firmware autonomicznego pojazdu na ESP32. Pojazd reaktywnie omija przeszkody
(LIDAR LD06/LD14P), wykrywa utknięcie (odometria) i dojeżdża do celu
termicznego (gorący obiekt wykrywany pirometrem MLX90614). Sterowanie i podgląd
telemetrii odbywa się przez wbudowany dashboard WWW.

Czujniki: IMU ICM-20948, pirometr MLX90614, monitor prądu INA219, czujnik
temperatury/wilgotności SHT40, czujniki linii CNY70, enkodery Halla (odometria),
LIDAR LD06/LD14P. Aktuatory: dwa silniki DC (mostek TB6612FNG), buzzer, diody.

## Wymagania

- **ESP-IDF 6.0.1** (zalecana dokładnie ta wersja; minimalna to 4.1).
- Płytka ESP32 oraz sprzęt pojazdu (czujniki, sterownik silników).
- Python 3 (do opcjonalnego narzędzia `tools/lidar_map.py`).

## 1. Instalacja ESP-IDF

Jeśli nie masz jeszcze ESP-IDF, zainstaluj je zgodnie z oficjalną instrukcją.
W skrócie (Linux/macOS):

```bash
mkdir -p ~/esp && cd ~/esp
git clone -b v6.0.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
```

Przed każdą pracą z projektem trzeba załadować środowisko ESP-IDF do bieżącej
powłoki (definiuje m.in. polecenie `idf.py`):

```bash
. ~/esp/esp-idf/export.sh
```

## 2. Pobranie projektu i zależności

```bash
git clone <adres-repozytorium> autonomous_vehicle
cd autonomous_vehicle
```

Projekt korzysta z biblioteki **cJSON**, zadeklarowanej jako zależność w
`main/idf_component.yml`. Menedżer komponentów ESP-IDF **pobiera ją
automatycznie przy pierwszej kompilacji** (do katalogu `managed_components/`,
ignorowanego przez git) — nie trzeba nic robić ręcznie.

Gdyby z jakiegoś powodu zależność nie pobrała się sama, można dodać ją ręcznie:

```bash
idf.py add-dependency "espressif/cjson^1.7.19"
```

## 3. Konfiguracja

Dane sieci WiFi oraz pozostałe parametry sprzętu znajdują się w
**`main/config.h`**. Przed wgraniem ustaw swoją sieć:

```c
#define WIFI_SSID   "Twoja_siec"
#define WIFI_PASS   "Twoje_haslo"
```

W tym samym pliku można w razie potrzeby zmienić piny GPIO, prędkości silników,
progi nawigacji itp. Adresy czujników I2C są zaszyte na stałe (zweryfikowane na
sprzęcie) — patrz sekcja niżej.

## 4. Kompilacja, wgranie i monitor

```bash
idf.py set-target esp32      # tylko przy pierwszej kompilacji
idf.py build
idf.py -p /dev/ttyUSB0 flash # podmień port na właściwy dla Twojego systemu
```

> **Uwaga o konsoli szeregowej.** LIDAR korzysta z UART2 na pinach GPIO1/GPIO3,
> które domyślnie obsługują konsolę USB-Serial (UART0). Po starcie firmware
> **konsola `idf.py monitor` przestaje wyświetlać logi.** Dlatego logi są
> dostępne przez przeglądarkę:
>
> - dashboard sterujący: `http://<IP_ESP32>/`
> - surowe logi (jak monitor szeregowy): `http://<IP_ESP32>/api/logs`
>
> Adres IP pojazd zgłasza po połączeniu z WiFi (widoczny w logach jako
> `Dashboard: http://<IP_ESP32>/`). Najprościej odczytać go z routera lub —
> jednorazowo na czas debugowania — ustawić `LIDAR_ENABLED` na `0` w
> `main/config.h`, co zwalnia UART0 i przywraca `idf.py monitor`.

## 5. Narzędzie desktopowe — mapa z LIDAR (opcjonalne)

W katalogu `tools/` znajduje się aplikacja w Pythonie rysująca mapę przeszkód
na podstawie danych z endpointu `GET /api/lidar/scan`:

```bash
cd tools
pip install -r requirements.txt
python3 lidar_map.py
```

Szczegóły działania i parametry opisuje `tools/README.md`.

## Struktura projektu

```
autonomous_vehicle/
├── CMakeLists.txt          # projekt ESP-IDF
├── main/
│   ├── main.c              # punkt wejścia (app_main), inicjalizacja
│   ├── config.h            # konfiguracja: piny, adresy I2C, WiFi, parametry
│   ├── autonomy.c/.h       # logika autonomii (maszyna stanów)
│   ├── motor_driver.c/.h   # sterownik silników (TB6612FNG)
│   ├── led_control.c/.h    # diody sygnalizacyjne
│   ├── buzzer.c/.h         # buzzer
│   ├── wifi_manager.c/.h   # łączność WiFi
│   ├── http_server.c/.h    # serwer HTTP: dashboard + API JSON
│   ├── web_monitor.c/.h    # przechwytywanie logów do podglądu przez WWW
│   └── sensors/            # sterowniki czujników
│       ├── imu.c/.h            # ICM-20948
│       ├── ina219.c/.h         # monitor prądu/napięcia
│       ├── sht40.c/.h          # temperatura/wilgotność
│       ├── pyrometer.c/.h      # MLX90614
│       ├── line_sensor.c/.h    # czujniki linii CNY70
│       ├── odometry.c/.h       # enkodery Halla
│       └── lidar.c/.h          # LIDAR LD06/LD14P
└── tools/
    ├── lidar_map.py        # desktopowa mapa przeszkód
    ├── requirements.txt
    └── README.md
```

## Adresy I2C (zaszyte na stałe)

Skanowanie magistrali zostało usunięte — adresy są ustalone w `main/config.h`:

| Czujnik              | Adres |
|----------------------|-------|
| MLX90614 (pirometr)  | 0x5A  |
| ICM-20948 (IMU)      | 0x69  |
| INA219 (prąd)        | 0x40  |
| SHT40 (temp./wilg.)  | 0x44  |

Magistrala I2C pracuje z częstotliwością 100 kHz (wymagane przez MLX90614 w
trybie SMBus).

## Uwaga na przyszłość

Sterowniki czujników korzystają ze starszego API I2C ESP-IDF
(`driver/i2c.h`, `i2c_cmd_link_*`). Działa ono poprawnie, ale w nowszych
wersjach ESP-IDF jest oznaczone jako przestarzałe na rzecz `driver/i2c_master.h`.
Migracja jest opcjonalna i warto ją wykonać dopiero z dostępem do sprzętu, aby
zweryfikować komunikację z każdym czujnikiem.
