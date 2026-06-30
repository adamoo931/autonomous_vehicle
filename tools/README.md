# Mapa przeszkód z lidaru

Moduł dodaje do projektu podgląd mapy przeszkół skanowanych lidarem (LD06/LD14P)
na wykresie punktowym, w prostej aplikacji desktopowej w Pythonie.

## Jak to działa

```
┌────────────┐   UART   ┌──────────────┐   HTTP /api/lidar/scan   ┌──────────────┐
│  LIDAR LD06│ ───────► │   ESP32      │ ◄──────────────────────► │  lidar_map.py│
│            │  punkty  │ (firmware)   │   JSON {seq,rpm,n,pts}   │  (desktop)   │
└────────────┘          │ bufor kołowy │                          │ siatka 5×5cm │
                        └──────────────┘                          └──────────────┘
```

1. **Firmware (ESP32)** – w `main/sensors/lidar.c` doszedł bufor kołowy, który akumuluje
   punkty (kąt + odległość) z wielu pakietów lidaru, każdy z numerem sekwencyjnym.
   Nowy endpoint **`GET /api/lidar/scan?since=N`** (w `main/http_server.c`) zwraca
   tylko punkty nowsze niż `N`, w kompaktowym JSON:

   ```json
   { "seq": 12345, "rpm": 10, "n": 480, "pts": [ang0, dist0, ang1, dist1, ...] }
   ```
   - `ang` – kąt w setnych stopnia (0..35999)
   - `dist` – odległość w mm (0 = brak echa)
   - `seq` – bieżący licznik; przekaż go jako `?since=` przy kolejnym zapytaniu,
     dzięki czemu pobierasz wyłącznie NOWE punkty (bez duplikatów i luk).

   Reszta firmware oraz dashboard WWW działają bez zmian.

2. **Aplikacja desktopowa (`lidar_map.py`)** – odpytuje endpoint kilka razy na
   sekundę i nanosi punkty na **siatkę zajętości o boku 5 cm**. Każda zajęta
   komórka 5×5 cm = **jedna czarna kropka**. Stąd:
   - sześcian 5×5 cm → 1 kropka,
   - przedmiot 5×30 cm → 6 kropek,
   - ściana → ciąg kropek (linia).

   Liczy się tylko rzut na podłogę (szerokość × długość) – wysokość przeszkody
   nie ma znaczenia, bo lidar skanuje w jednej płaszczyźnie.

## Uruchomienie aplikacji

```bash
cd tools
pip install -r requirements.txt          # matplotlib + numpy
python3 lidar_map.py
```

1. Wpisz **adres IP robota** – ten sam, pod którym otwierasz dashboard WWW
   (firmware loguje go na monitorze webowym: `Dashboard: http://<IP_ESP32>/`).
2. Kliknij **START**. Mapa zacznie się akumulować.
3. **RESET mapy** czyści wszystkie zeskanowane przeszkody.

### Wskazówki

- **Najlepiej skanuj nieruchomym robotem** (albo obracającym się w miejscu) –
  mapa budowana jest w układzie współrzędnych lidaru (robot w środku, oś X =
  przód). Pozwala to zweryfikować, czy ściany i przeszkody rysują się poprawnie.
- **Offset kąta** i **Odwróć kierunek** służą do obrócenia mapy tak, by „przód”
  na wykresie zgadzał się z fizycznym przodem robota (zależy od montażu lidaru).
- **Komórka [mm]** – zmień rozmiar komórki (domyślnie 50 mm = 5 cm).
- **Zasięg [m]** – odcina dalekie/szumowe punkty.
- **Eksport CSV** zapisuje środki zajętych komórek (w metrach) do dalszej obróbki.
- Pasek narzędzi pod wykresem (matplotlib) umożliwia zoom, przesuwanie i zapis PNG.

## Parametry firmware (opcjonalnie do zmiany)

- `LIDAR_SCAN_BUFFER` w `main/sensors/lidar.h` – pojemność bufora kołowego (domyślnie
  1800 punktów ≈ kilka obrotów LD06). Większy bufor = większa tolerancja na
  wolniejsze odpytywanie z aplikacji.
- Okno odczytu na żądanie (max 512 punktów) ustawione w `handle_lidar_scan`
  w `main/http_server.c`.
