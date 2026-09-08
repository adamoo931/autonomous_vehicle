#include "http_server.h"
#include "motor_driver.h"
#include "led_control.h"
#include "odometry.h"
#include "pyrometer.h"
#include "imu.h"
#include "ina219.h"
#include "sht40.h"
#include "ads1115.h"
#include "hall_finish.h"
#include "buzzer.h"
#include "lidar.h"
#include "autonomy.h"
#include "line_test.h"
#include "line_sensor.h"
#include "web_monitor.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "HTTP";
static httpd_handle_t s_server = NULL;

/* Wbudowany dashboard HTML (strona sterująca serwowana pod GET /). */
static const char DASHBOARD_HTML[] =
    "<!DOCTYPE html>\n"
    "<html lang=\"pl\">\n"
    "<head>\n"
    "<meta charset=\"UTF-8\">\n"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
    "<title>Robot Dashboard</title>\n"
    "<style>\n"
    "*{box-sizing:border-box;margin:0;padding:0}\n"
    "body{font-family:monospace;background:#0d1117;color:#c9d1d9;padding:8px}\n"
    "h1{text-align:center;color:#58a6ff;padding:10px 0;font-size:1.3em}\n"
    ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:8px;margin-bottom:8px}\n"
    ".card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px}\n"
    ".card h2{color:#f78166;font-size:0.85em;margin-bottom:8px;text-transform:uppercase;letter-spacing:.05em}\n"
    ".val{color:#79c0ff;font-weight:bold}\n"
    ".ok{color:#3fb950}.warn{color:#d29922}.err{color:#f85149}\n"
    "button{background:#21262d;color:#c9d1d9;border:1px solid #30363d;padding:8px 14px;\n"
    "       border-radius:6px;cursor:pointer;font-family:monospace;margin:2px}\n"
    "button:hover{background:#30363d}\n"
    ".stop-btn{background:#3d1212;border-color:#f85149;color:#f85149}\n"
    ".stop-btn:hover{background:#5a1a1a}\n"
    ".dpad{display:grid;grid-template-columns:repeat(3,60px);gap:4px;margin:8px auto;width:188px}\n"
    ".dpad button{width:60px;height:40px;font-size:1.1em;text-align:center}\n"
    ".dpad .mid{display:flex;gap:4px}\n"
    ".speed-row{display:flex;align-items:center;gap:8px;margin:8px 0}\n"
    ".speed-row input{flex:1}\n"
    ".led-row button{width:44px;height:44px;border-radius:50%;font-weight:bold;font-size:0.9em}\n"
    ".tag{display:inline-block;padding:2px 6px;border-radius:4px;font-size:0.8em}\n"
    ".tag-ok{background:#1b3a2d;color:#3fb950}\n"
    ".tag-warn{background:#3a2a0d;color:#d29922}\n"
    "#status-bar{text-align:center;padding:6px;background:#161b22;border-radius:6px;margin-bottom:8px;font-size:0.85em}\n"
    ".lE{color:#f85149}.lW{color:#d29922}.lI{color:#3fb950}.lD{color:#8b949e}.lo{color:#c9d1d9}\n"
    "</style>\n"
    "</head>\n"
    "<body>\n"
    "<h1>&#129302; Robot Dashboard</h1>\n"
    "<div id=\"status-bar\">&#128997; Laczenie...</div>\n"
    "\n"
    "<div class=\"grid\">\n"
    "  <div class=\"card\">\n"
    "    <h2>&#128207; LIDAR (LD06)</h2>\n"
    "    <div>Min odl.: <span class=\"val\" id=\"lid-min\">-</span> mm</div>\n"
    "    <div>K&#261;t: <span class=\"val\" id=\"lid-angle\">-</span>&#176;</div>\n"
    "    <div>Pr&#281;dk.: <span class=\"val\" id=\"lid-rpm\">-</span> RPM</div>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#127777;&#65039; Pirometr MLX90614</h2>\n"
    "    <div>Obiekt: <span class=\"val\" id=\"py-obj\">-</span> &#176;C</div>\n"
    "    <div>Otoczenie: <span class=\"val\" id=\"py-amb\">-</span> &#176;C</div>\n"
    "    <div>Szukanie obiektu: <span class=\"val\" id=\"py-search\">-</span></div>\n"
    "    <div>Obiekt cieplny: <span class=\"val\" id=\"py-hot\">-</span></div>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#128208; IMU ICM-20948</h2>\n"
    "    <div>Ax:<span class=\"val\" id=\"ax\">-</span> Ay:<span class=\"val\" id=\"ay\">-</span> Az:<span class=\"val\" id=\"az\">-</span> g</div>\n"
    "    <div>Gx:<span class=\"val\" id=\"gx\">-</span> Gy:<span class=\"val\" id=\"gy\">-</span> Gz:<span class=\"val\" id=\"gz\">-</span> &#176;/s</div>\n"
    "    <div>Mx:<span class=\"val\" id=\"mx\">-</span> My:<span class=\"val\" id=\"my\">-</span> Mz:<span class=\"val\" id=\"mz\">-</span> &#181;T</div>\n"
    "    <div>Azymut: <span class=\"val\" id=\"hdg\">-</span>&#176;</div>\n"
    "    <div>T: <span class=\"val\" id=\"imu-t\">-</span> &#176;C</div>\n"
    "    <div style=\"margin-top:6px;display:flex;gap:6px;flex-wrap:wrap\">\n"
    "      <button onclick=\"magCalStart()\">&#129517; Kalibruj (obr&#243;t 360&#176;)</button>\n"
    "      <button onclick=\"magSetNorth()\">&#129517; Ustaw N (obecny kierunek)</button>\n"
    "    </div>\n"
    "    <div style=\"font-size:0.8em;color:#8b949e;margin-top:4px\" id=\"mag-cal-status\"></div>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#9889; INA219 Zasilanie</h2>\n"
    "    <div>Napi&#281;cie: <span class=\"val\" id=\"in-v\">-</span> V</div>\n"
    "    <div>Pr&#261;d: <span class=\"val\" id=\"in-i\">-</span> mA</div>\n"
    "    <div>Moc: <span class=\"val\" id=\"in-p\">-</span> mW</div>\n"
    "    <div>Adres I2C: <span class=\"val\" id=\"in-addr\">-</span></div>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#127777;&#65039; SHT40 Klimat</h2>\n"
    "    <div>Temp.: <span class=\"val\" id=\"sh-t\">-</span> &#176;C</div>\n"
    "    <div>Wilgotno&#347;&#263;: <span class=\"val\" id=\"sh-h\">-</span> %</div>\n"
    "    <div>Adres I2C: <span class=\"val\" id=\"sh-addr\">-</span></div>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#129442; Halla mety (49E, cyfrowy DO)</h2>\n"
    "    <div>DO surowy: <span class=\"val\" id=\"hl-do\" style=\"font-size:1.3em\">-</span></div>\n"
    "    <div>Meta: <span class=\"val\" id=\"hl-det\">-</span></div>\n"
    "    <label style=\"display:block;margin-top:6px;font-size:0.9em\"><input type=\"checkbox\" id=\"hl-alow\" onchange=\"setHallActiveLow()\"> meta = DO w stanie LOW</label>\n"
    "    <div style=\"font-size:0.8em;color:#8b949e\">Pr&#243;g ustaw potencjometrem na module, patrz&#261;c na \"DO surowy\" &#8211; ma prze&#322;&#261;cza&#263; si&#281; przy magnesie. Potem ustaw polaryzacj&#281; powy&#380;ej, tak by \"Meta\" = WYKRYTO tylko z magnesem.</div>\n"
    "    <label style=\"display:block;margin-top:6px;font-size:0.9em\"><input type=\"checkbox\" id=\"hl-manual\" onchange=\"setHallManual()\"> reaguj na Hall w je&#378;dzie r&#281;cznej</label>\n"
    "    <div style=\"font-size:0.8em;color:#8b949e\">(w autonomii meta jest sygnalizowana decyzj&#261; autonomii, niezale&#380;nie od tego)</div>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#128207; Odometria</h2>\n"
    "    <div>Lewe: <span class=\"val\" id=\"od-l\">-</span> mm</div>\n"
    "    <div>Prawe: <span class=\"val\" id=\"od-r\">-</span> mm</div>\n"
    "    <div>Suma: <span class=\"val\" id=\"od-t\">-</span> mm</div>\n"
    "    <div>Impulsy L/P: <span class=\"val\" id=\"od-pl\">-</span> / <span class=\"val\" id=\"od-pr\">-</span></div>\n"
    "    <div>Czas przejazdu: <span class=\"val\" id=\"od-time\">-</span> s</div>\n"
    "    <div>Zu&#380;yta energia: <span class=\"val\" id=\"od-energy\">-</span> mWh</div>\n"
    "    <br><button onclick=\"fetch('/api/odometry/reset',{method:'POST'})\">&#128260; Reset</button>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#9633; Czujniki linii CNY70</h2>\n"
    "    <div>Prz&#243;d P (A0): <span class=\"val\" id=\"ls-fr-v\">-</span> V &nbsp; <span id=\"ls-fr\">-</span></div>\n"
    "    <div>Prz&#243;d L (A1): <span class=\"val\" id=\"ls-fl-v\">-</span> V &nbsp; <span id=\"ls-fl\">-</span></div>\n"
    "    <div>Ty&#322; L (A2): <span class=\"val\" id=\"ls-bl-v\">-</span> V &nbsp; <span id=\"ls-bl\">-</span></div>\n"
    "    <div>Ty&#322; P (A3): <span class=\"val\" id=\"ls-br-v\">-</span> V &nbsp; <span id=\"ls-br\">-</span></div>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#9881;&#65039; Silniki</h2>\n"
    "    <div>Lewy: <span class=\"val\" id=\"m-l\">-</span> %</div>\n"
    "    <div>Prawy: <span class=\"val\" id=\"m-r\">-</span> %</div>\n"
    "  </div>\n"
    "</div>\n"
    "\n"
    "<div class=\"grid\">\n"
    "  <div class=\"card\">\n"
    "    <h2>&#127918; Sterowanie silnikami</h2>\n"
    "    <div class=\"speed-row\">\n"
    "      <label>V:</label>\n"
    "      <input type=\"range\" id=\"spd\" min=\"10\" max=\"100\" value=\"50\">\n"
    "      <span id=\"spd-v\">50</span>%\n"
    "    </div>\n"
    "    <div class=\"dpad\">\n"
    "      <div></div>\n"
    "      <button onclick=\"mv(spd(),spd())\">&#8679;</button>\n"
    "      <div></div>\n"
    "      <button onclick=\"mv(-spd(),spd())\">&#8678;</button>\n"
    "      <button class=\"stop-btn\" onclick=\"doStop()\">&#9632;</button>\n"
    "      <button onclick=\"mv(spd(),-spd())\">&#8680;</button>\n"
    "      <div></div>\n"
    "      <button onclick=\"mv(-spd(),-spd())\">&#8681;</button>\n"
    "      <div></div>\n"
    "    </div>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#128161; LEDs</h2>\n"
    "    <div class=\"led-row\">\n"
    "      <button style=\"background:#c62828;color:#fff\" onclick=\"tgl('red')\" id=\"b-red\">R</button>\n"
    "      <button style=\"background:#f9a825;color:#000\" onclick=\"tgl('yellow')\" id=\"b-yellow\">Y</button>\n"
    "      <button style=\"background:#2e7d32;color:#fff\" onclick=\"tgl('green')\" id=\"b-green\">G</button>\n"
    "      <button onclick=\"allOff()\">Wy&#322;.</button>\n"
    "    </div>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#128266; Buzzer</h2>\n"
    "    <button onclick=\"beep()\">&#9658; Test (2 kHz)</button>\n"
    "    <button onclick=\"buzz(1000,300)\">Niski ton</button>\n"
    "    <button onclick=\"buzz(4000,300)\">Wysoki ton</button>\n"
    "    <button onclick=\"song()\">&#127926; Muzyka</button>\n"
    "    <button class=\"stop-btn\" onclick=\"buzzerStop()\">&#9632; Stop</button>\n"
    "  </div>\n"
    "</div>\n"
    "\n"
    "<div class=\"card\" style=\"margin-bottom:8px\">\n"
    "  <h2>&#129302; Autonomia</h2>\n"
    "  <div style=\"display:flex;gap:12px;align-items:center;flex-wrap:wrap\">\n"
    "    <button id=\"auto-btn\" onclick=\"autoToggle()\" style=\"font-size:1.05em;padding:12px 20px\">&#9654; Symuluj autonomi&#281;</button>\n"
    "    <span>Stan: <span class=\"val\" id=\"auto-state\">-</span></span>\n"
    "    <a id=\"auto-log-link\" href=\"/api/autonomy/log.csv\" style=\"padding:10px 16px;border:1px solid #30363d;border-radius:6px;text-decoration:none;color:#c9d1d9;background:#21262d\">&#128190; Pobierz log przejazdu (CSV)</a>\n"
    "    <span style=\"font-size:0.85em;color:#8b949e\">wierszy w logu: <span class=\"val\" id=\"auto-logcount\">0</span></span>\n"
    "  </div>\n"
    "  <div style=\"display:flex;gap:8px;align-items:center;margin-top:8px;flex-wrap:wrap\">\n"
    "    <label>Offset kursu (cel, + = w lewo):</label>\n"
    "    <input type=\"number\" id=\"auto-heading-in\" min=\"-90\" max=\"90\" step=\"1\" value=\"18\" style=\"width:70px\">\n"
    "    <span>&#176;</span>\n"
    "    <button onclick=\"setHeading()\">Zapisz</button>\n"
    "    <span style=\"font-size:0.85em;color:#8b949e\">ustawiony: <span class=\"val\" id=\"auto-heading-cur\">-</span>&#176; &#8212; do jazdy prosto ustaw 0</span>\n"
    "  </div>\n"
    "  <div style=\"display:flex;gap:8px;align-items:center;margin-top:8px;flex-wrap:wrap\">\n"
    "    <label>Pr&#281;dko&#347;&#263; jazdy (na wprost/do ty&#322;u):</label>\n"
    "    <input type=\"number\" id=\"auto-speed-in\" min=\"0\" max=\"100\" step=\"1\" value=\"35\" style=\"width:70px\">\n"
    "    <span>%</span>\n"
    "    <button onclick=\"setSpeed()\">Zapisz</button>\n"
    "    <span style=\"font-size:0.85em;color:#8b949e\">ustawiona: <span class=\"val\" id=\"auto-speed-cur\">-</span>%</span>\n"
    "  </div>\n"
    "  <div style=\"display:flex;gap:8px;align-items:center;margin-top:8px;flex-wrap:wrap\">\n"
    "    <label>LIDAR &#8211; offset przodu:</label>\n"
    "    <input type=\"number\" id=\"auto-lidfront-in\" min=\"-180\" max=\"359\" step=\"1\" value=\"0\" style=\"width:70px\"><span>&#176;</span>\n"
    "    <label style=\"margin-left:8px\">STOP przed przeszkod&#261;:</label>\n"
    "    <input type=\"number\" id=\"auto-stopmm-in\" min=\"150\" max=\"1500\" step=\"10\" value=\"300\" style=\"width:80px\"><span>mm</span>\n"
    "    <button onclick=\"setLidar()\">Zapisz</button>\n"
    "  </div>\n"
    "  <div style=\"display:flex;gap:8px;align-items:center;margin-top:8px;flex-wrap:wrap\">\n"
    "    <label>Omijanie &#8211; zakres skanu:</label>\n"
    "    <input type=\"number\" id=\"auto-scandeg-in\" min=\"30\" max=\"120\" step=\"5\" value=\"80\" style=\"width:70px\"><span>&#177;&#176;</span>\n"
    "    <label style=\"margin-left:8px\">czas przez szczelin&#281;:</label>\n"
    "    <input type=\"number\" id=\"auto-passms-in\" min=\"500\" max=\"6000\" step=\"100\" value=\"2500\" style=\"width:80px\"><span>ms</span>\n"
    "    <button onclick=\"setLidar()\">Zapisz</button>\n"
    "  </div>\n"
    "  <div style=\"display:flex;gap:8px;align-items:center;margin-top:8px;flex-wrap:wrap\">\n"
    "    <label>G&#243;rna kraw&#281;d&#378; &#8211; pr&#243;g czasu jazdy naprz&#243;d:</label>\n"
    "    <input type=\"number\" id=\"auto-travms-in\" min=\"5000\" max=\"180000\" step=\"1000\" value=\"25000\" style=\"width:90px\"><span>ms</span>\n"
    "    <button onclick=\"setLidar()\">Zapisz</button>\n"
    "    <span style=\"font-size:0.85em;color:#8b949e\">jazda naprz&#243;d: <span class=\"val\" id=\"auto-fwdms\">-</span> ms</span>\n"
    "  </div>\n"
    "  <div style=\"font-size:0.82em;color:#8b949e;margin-top:6px;font-family:monospace\">\n"
    "    kurs: <span class=\"val\" id=\"auto-heading\">-</span>&#176; / cel <span class=\"val\" id=\"auto-htgt\">-</span>&#176; &nbsp; "
    "gyro_z raw/filt: <span class=\"val\" id=\"auto-gyroz\">-</span>&#176;/s &nbsp; "
    "bias: <span class=\"val\" id=\"auto-gbias\">-</span>&#176;/s &nbsp; "
    "korytarz: <span class=\"val\" id=\"auto-corr\">-</span>/<span class=\"val\" id=\"auto-stopmm-cur\">-</span> mm &nbsp; "
    "LIDAR P/PL/L/TL/T/TP/R/PP: <span class=\"val\" id=\"auto-sectors\">-</span> mm\n"
    "  </div>\n"
    "  <div style=\"font-size:0.8em;color:#8b949e;margin-top:6px\">Etap 1: pojazd jedzie na wprost ustawion&#261; pr&#281;dko&#347;ci&#261;, trzymaj&#261;c kurs. Przeszkoda w korytarzu (LIDAR) bli&#380;ej ni&#380; pr&#243;g &#8594; skan szczelin, obr&#243;t ku najlepszej szczelinie w stron&#281; celu i przejazd obok przeszkody (Krok 7); brak szczeliny &#8594; STOP i ponawianie skanu. Po wykryciu linii toru staje na ~1,5 s i czeka na Hall &#8211; je&#347;li meta, ko&#324;czy przejazd; je&#347;li nie, cofa si&#281; i jedzie dalej. Dowolny ruch r&#281;czny lub STOP przerywa autonomi&#281;. Log pobierz zaraz po je&#378;dzie &#8211; nast&#281;pny przejazd go nadpisuje.</div>\n"
    "</div>\n"
    "\n"
    "<div class=\"card\" style=\"margin-bottom:8px\">\n"
    "  <h2>&#9633; Test wykrywania linii</h2>\n"
    "  <div style=\"display:flex;gap:12px;align-items:center;flex-wrap:wrap\">\n"
    "    <button id=\"lt-btn\" onclick=\"lineTestToggle()\" style=\"font-size:1.05em;padding:12px 20px\">&#9654; Uruchom test</button>\n"
    "    <label><input type=\"radio\" name=\"lt-dir\" value=\"forward\" checked> do przodu</label>\n"
    "    <label><input type=\"radio\" name=\"lt-dir\" value=\"backward\"> do ty&#322;u</label>\n"
    "    <span>Stan: <span class=\"val\" id=\"lt-state\">-</span></span>\n"
    "  </div>\n"
    "  <div style=\"font-size:0.8em;color:#8b949e;margin-top:6px\">Pojazd porusza si&#281; skokowo: 0,3 s jazdy z pr&#281;dko&#347;ci&#261; 25 %, potem 0,3 s postoju &#8211; w p&#281;tli. Gdy <b>kt&#243;rykolwiek</b> czujnik CNY70 wykryje lini&#281;/kraw&#281;d&#378;, pojazd zatrzymuje si&#281; ca&#322;kowicie, a w monitorze pojawia si&#281; log z nazw&#261; czujnika. Test i autonomia wykluczaj&#261; si&#281;; dowolny ruch r&#281;czny lub STOP przerywa test. Kierunek zmieniaj przy zatrzymanym te&#347;cie.</div>\n"
    "</div>\n"
    "\n"
    "<div class=\"card\" style=\"margin-bottom:8px\">\n"
    "  <h2>&#128421;&#65039; Monitor (logi UART / ESP_LOG)</h2>\n"
    "  <div style=\"display:flex;gap:10px;align-items:center;margin-bottom:6px;flex-wrap:wrap\">\n"
    "    <button onclick=\"clearLogs()\">&#128465;&#65039; Wyczy&#347;&#263;</button>\n"
    "    <label style=\"font-size:0.8em\"><input type=\"checkbox\" id=\"mon-on\" checked> od&#347;wie&#380;anie</label>\n"
    "    <label style=\"font-size:0.8em\"><input type=\"checkbox\" id=\"mon-scroll\" checked> auto-scroll</label>\n"
    "    <span id=\"mon-status\" style=\"font-size:0.8em;color:#8b949e\"></span>\n"
    "  </div>\n"
    "  <pre id=\"logbox\" style=\"background:#010409;border:1px solid #30363d;border-radius:6px;padding:8px;height:340px;overflow-y:auto;font-size:0.78em;line-height:1.4;white-space:pre-wrap;word-break:break-word;margin:0\"></pre>\n"
    "</div>\n"
    "\n"
    "<script>\n"
    "var leds={red:0,yellow:0,green:0};\n"
    "function spd(){return parseInt(document.getElementById('spd').value);}\n"
    "document.getElementById('spd').oninput=function(){document.getElementById('spd-v').textContent=this.value;};\n"
    "\n"
    "function mv(l,r){\n"
    "  fetch('/api/motor',{method:'POST',headers:{'Content-Type':'application/json'},\n"
    "    body:JSON.stringify({left:l,right:r})});\n"
    "}\n"
    "function doStop(){\n"
    "  fetch('/api/motor/stop',{method:'POST'});\n"
    "}\n"
    "function tgl(c){\n"
    "  leds[c]=leds[c]?0:1;\n"
    "  fetch('/api/led',{method:'POST',headers:{'Content-Type':'application/json'},\n"
    "    body:JSON.stringify(leds)});\n"
    "  updateLedButtons();\n"
    "}\n"
    "function allOff(){\n"
    "  leds={red:0,yellow:0,green:0};\n"
    "  fetch('/api/led',{method:'POST',headers:{'Content-Type':'application/json'},\n"
    "    body:JSON.stringify(leds)});\n"
    "  updateLedButtons();\n"
    "}\n"
    "function updateLedButtons(){\n"
    "  document.getElementById('b-red').style.opacity=leds.red?'1':'0.35';\n"
    "  document.getElementById('b-yellow').style.opacity=leds.yellow?'1':'0.35';\n"
    "  document.getElementById('b-green').style.opacity=leds.green?'1':'0.35';\n"
    "}\n"
    "function tag(v,okLabel,warnLabel){\n"
    "  return v?('<span class=\"tag tag-warn\">'+warnLabel+'</span>'):('<span class=\"tag tag-ok\">'+okLabel+'</span>');\n"
    "}\n"
    "function poll(){\n"
    "  fetch('/api/sensors').then(function(r){return r.json();}).then(function(d){\n"
    "    document.getElementById('status-bar').innerHTML='<span class=\"ok\">&#128994; Polaczono</span> &nbsp; '+new Date().toLocaleTimeString();\n"
    "\n"
    "    if(d.lidar){\n"
    "      document.getElementById('lid-min').textContent=d.lidar.min_distance_mm;\n"
    "      document.getElementById('lid-angle').textContent=(d.lidar.angle_hundredths/100).toFixed(1);\n"
    "      document.getElementById('lid-rpm').textContent=d.lidar.speed_rpm;\n"
    "    }\n"
    "    if(d.pyrometer){\n"
    "      document.getElementById('py-obj').textContent=d.pyrometer.object_temp.toFixed(1);\n"
    "      document.getElementById('py-amb').textContent=d.pyrometer.ambient_temp.toFixed(1);\n"
    "      document.getElementById('py-search').innerHTML=d.pyrometer.searching?'<span class=\"tag tag-warn\">AKTYWNE</span>':'Nie';\n"
    "      document.getElementById('py-hot').innerHTML=d.pyrometer.hot_detected?'<span class=\"ok\">WYKRYTO</span>':'Nie';\n"
    "    }\n"
    "    if(d.imu){\n"
    "      document.getElementById('ax').textContent=d.imu.accel_x.toFixed(2);\n"
    "      document.getElementById('ay').textContent=d.imu.accel_y.toFixed(2);\n"
    "      document.getElementById('az').textContent=d.imu.accel_z.toFixed(2);\n"
    "      document.getElementById('gx').textContent=d.imu.gyro_x.toFixed(1);\n"
    "      document.getElementById('gy').textContent=d.imu.gyro_y.toFixed(1);\n"
    "      document.getElementById('gz').textContent=d.imu.gyro_z.toFixed(1);\n"
    "      document.getElementById('mx').textContent=d.imu.mag_x.toFixed(1);\n"
    "      document.getElementById('my').textContent=d.imu.mag_y.toFixed(1);\n"
    "      document.getElementById('mz').textContent=d.imu.mag_z.toFixed(1);\n"
    "      document.getElementById('hdg').innerHTML=d.imu.mag_initialized?d.imu.azimuth_deg.toFixed(1):'<span class=\"err\">BRAK</span>';\n"
    "      document.getElementById('imu-t').textContent=d.imu.temp.toFixed(1);\n"
    "      if(d.imu.mag_cal){\n"
    "        var mc=d.imu.mag_cal;\n"
    "        document.getElementById('mag-cal-status').textContent=mc.active\n"
    "          ? ('Kalibracja: obracaj pojazdem... zostalo '+Math.ceil(mc.remaining_ms/1000)+' s')\n"
    "          : ('offset=('+mc.offset_x.toFixed(1)+','+mc.offset_y.toFixed(1)+') faza='+mc.phase_deg.toFixed(1)+'\\u00B0');\n"
    "      }\n"
    "    }\n"
    "    if(d.ina219){\n"
    "      document.getElementById('in-v').textContent=d.ina219.bus_voltage_v.toFixed(2);\n"
    "      document.getElementById('in-i').textContent=d.ina219.current_ma.toFixed(0);\n"
    "      document.getElementById('in-p').textContent=d.ina219.power_mw.toFixed(0);\n"
    "      document.getElementById('in-addr').innerHTML=d.ina219.initialized?('0x'+d.ina219.address.toString(16)):'<span class=\"err\">BRAK</span>';\n"
    "    }\n"
    "    if(d.sht40){\n"
    "      document.getElementById('sh-t').textContent=d.sht40.temperature_c.toFixed(1);\n"
    "      document.getElementById('sh-h').textContent=d.sht40.humidity_pct.toFixed(1);\n"
    "      document.getElementById('sh-addr').innerHTML=d.sht40.initialized?('0x'+d.sht40.address.toString(16)):'<span class=\"err\">BRAK</span>';\n"
    "    }\n"
    "    if(d.odometry){\n"
    "      document.getElementById('od-l').textContent=d.odometry.dist_left_mm.toFixed(0);\n"
    "      document.getElementById('od-r').textContent=d.odometry.dist_right_mm.toFixed(0);\n"
    "      document.getElementById('od-t').textContent=d.odometry.dist_total_mm.toFixed(0);\n"
    "      document.getElementById('od-pl').textContent=d.odometry.pulses_left;\n"
    "      document.getElementById('od-pr').textContent=d.odometry.pulses_right;\n"
    "    }\n"
    "    if(d.hall){\n"
    "      if(typeof d.hall.do_raw!=='undefined') document.getElementById('hl-do').textContent=d.hall.do_raw;\n"
    "      document.getElementById('hl-det').innerHTML=d.hall.detected?'<span class=\"ok\">WYKRYTO</span>':'Nie';\n"
    "      if(typeof d.hall.active_low!=='undefined'){\n"
    "        var ha=document.getElementById('hl-alow');\n"
    "        if(document.activeElement!==ha) ha.checked=d.hall.active_low;\n"
    "      }\n"
    "      if(typeof d.hall.manual_enabled!=='undefined'){\n"
    "        var hm=document.getElementById('hl-manual');\n"
    "        if(document.activeElement!==hm) hm.checked=d.hall.manual_enabled;\n"
    "      }\n"
    "    }\n"
    "    if(d.line_sensors){\n"
    "      var ls=d.line_sensors;\n"
    "      var ld=function(v){return v?'<span class=\"ok\">WYKRYTO</span>':'Nie';};\n"
    "      if(typeof ls.front_left_v!=='undefined'){\n"
    "        document.getElementById('ls-fr-v').textContent=ls.front_right_v.toFixed(3);\n"
    "        document.getElementById('ls-fl-v').textContent=ls.front_left_v.toFixed(3);\n"
    "        document.getElementById('ls-bl-v').textContent=ls.back_left_v.toFixed(3);\n"
    "        document.getElementById('ls-br-v').textContent=ls.back_right_v.toFixed(3);\n"
    "      }\n"
    "      document.getElementById('ls-fr').innerHTML=ld(ls.front_right);\n"
    "      document.getElementById('ls-fl').innerHTML=ld(ls.front_left);\n"
    "      document.getElementById('ls-bl').innerHTML=ld(ls.back_left);\n"
    "      document.getElementById('ls-br').innerHTML=ld(ls.back_right);\n"
    "    }\n"
    "    if(d.motors){\n"
    "      document.getElementById('m-l').textContent=d.motors.left;\n"
    "      document.getElementById('m-r').textContent=d.motors.right;\n"
    "    }\n"
    "    if(d.autonomy){\n"
    "      updateAuto(d.autonomy.enabled,d.autonomy.state,d.autonomy.log_count);\n"
    "      if(typeof d.autonomy.heading_target_deg!=='undefined'){\n"
    "        document.getElementById('auto-heading-cur').textContent=d.autonomy.heading_target_deg.toFixed(0);\n"
    "        var hi=document.getElementById('auto-heading-in');\n"
    "        if(document.activeElement!==hi && !hi.dataset.touched){hi.value=d.autonomy.heading_target_deg.toFixed(0);}\n"
    "      }\n"
    "      if(typeof d.autonomy.speed_pct!=='undefined'){\n"
    "        document.getElementById('auto-speed-cur').textContent=d.autonomy.speed_pct;\n"
    "        var spi=document.getElementById('auto-speed-in');\n"
    "        if(document.activeElement!==spi && !spi.dataset.touched){spi.value=d.autonomy.speed_pct;}\n"
    "      }\n"
    "      document.getElementById('od-time').textContent=d.autonomy.run_time_s.toFixed(1);\n"
    "      document.getElementById('od-energy').textContent=d.autonomy.run_energy_mwh.toFixed(1);\n"
    "      if(typeof d.autonomy.heading_deg!=='undefined'){\n"
    "        document.getElementById('auto-heading').textContent=d.autonomy.heading_deg.toFixed(1);\n"
    "        if(typeof d.autonomy.heading_target_deg!=='undefined')document.getElementById('auto-htgt').textContent=d.autonomy.heading_target_deg.toFixed(1);\n"
    "        document.getElementById('auto-gyroz').textContent=d.autonomy.gyro_z_dps.toFixed(1)+'/'+d.autonomy.gyro_z_filt_dps.toFixed(1);\n"
    "        document.getElementById('auto-gbias').textContent=d.autonomy.gyro_bias_dps.toFixed(2);\n"
    "        var sc=d.autonomy.lidar_sectors_mm||[];\n"
    "        document.getElementById('auto-sectors').textContent=sc.join('/');\n"
    "      }\n"
    "      if(typeof d.autonomy.corridor_mm!=='undefined'){\n"
    "        document.getElementById('auto-corr').textContent=d.autonomy.corridor_mm;\n"
    "        document.getElementById('auto-stopmm-cur').textContent=d.autonomy.front_stop_mm;\n"
    "        var lfi=document.getElementById('auto-lidfront-in');\n"
    "        if(document.activeElement!==lfi && !lfi.dataset.touched){lfi.value=d.autonomy.lid_front_deg;}\n"
    "        var smi=document.getElementById('auto-stopmm-in');\n"
    "        if(document.activeElement!==smi && !smi.dataset.touched){smi.value=d.autonomy.front_stop_mm;}\n"
    "        if(typeof d.autonomy.scan_max_deg!=='undefined'){\n"
    "          var sdi=document.getElementById('auto-scandeg-in');\n"
    "          if(document.activeElement!==sdi && !sdi.dataset.touched){sdi.value=d.autonomy.scan_max_deg;}\n"
    "          var pmi=document.getElementById('auto-passms-in');\n"
    "          if(document.activeElement!==pmi && !pmi.dataset.touched){pmi.value=d.autonomy.avoid_pass_ms;}\n"
    "        }\n"
    "        if(typeof d.autonomy.forward_ms!=='undefined'){\n"
    "          document.getElementById('auto-fwdms').textContent=d.autonomy.forward_ms;\n"
    "          var tmi=document.getElementById('auto-travms-in');\n"
    "          if(document.activeElement!==tmi && !tmi.dataset.touched){tmi.value=d.autonomy.traverse_ms;}\n"
    "        }\n"
    "      }\n"
    "    }\n"
    "    if(d.line_test){\n"
    "      var lts=d.line_test.state;\n"
    "      if(!d.line_test.running && d.line_test.hit) lts+=': '+d.line_test.hit;\n"
    "      updateLineTest(d.line_test.running,lts,d.line_test.forward);\n"
    "    }\n"
    "  }).catch(function(){\n"
    "    document.getElementById('status-bar').innerHTML='<span class=\"err\">&#128997; Brak polaczenia</span>';\n"
    "  });\n"
    "}\n"
    "\n"
    "function beep(){fetch('/api/buzzer',{method:'POST'});}\n"
    "function buzz(f,d){fetch('/api/buzzer',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({freq:f,duration_ms:d})});}\n"
    "function song(){fetch('/api/buzzer/song',{method:'POST'});}\n"
    "function buzzerStop(){fetch('/api/buzzer/stop',{method:'POST'});}\n"
    "function magCalStart(){fetch('/api/imu/mag_cal/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({duration_ms:15000})});}\n"
    "function magSetNorth(){fetch('/api/imu/mag_cal/set_north',{method:'POST'});}\n"
    "document.getElementById('auto-heading-in').addEventListener('input',function(){this.dataset.touched='1';});\n"
    "function setHeading(){\n"
    "  var v=parseFloat(document.getElementById('auto-heading-in').value);\n"
    "  if(isNaN(v)){alert('Podaj offset kursu w stopniach');return;}\n"
    "  fetch('/api/autonomy/heading',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({heading_deg:v})})\n"
    "    .then(function(r){return r.json();})\n"
    "    .then(function(d){if(d&&typeof d.heading_target_deg!=='undefined'){\n"
    "      var hi=document.getElementById('auto-heading-in');delete hi.dataset.touched;\n"
    "      hi.value=d.heading_target_deg;\n"
    "      document.getElementById('auto-heading-cur').textContent=d.heading_target_deg.toFixed(0);\n"
    "    }}).catch(function(){});\n"
    "}\n"
    "document.getElementById('auto-speed-in').addEventListener('input',function(){this.dataset.touched='1';});\n"
    "function setSpeed(){\n"
    "  var v=parseInt(document.getElementById('auto-speed-in').value);\n"
    "  if(isNaN(v)){alert('Podaj predkosc w procentach (0-100)');return;}\n"
    "  fetch('/api/autonomy/speed',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({speed_pct:v})})\n"
    "    .then(function(r){return r.json();})\n"
    "    .then(function(d){if(d&&typeof d.speed_pct!=='undefined'){\n"
    "      var spi=document.getElementById('auto-speed-in');delete spi.dataset.touched;\n"
    "      spi.value=d.speed_pct;\n"
    "      document.getElementById('auto-speed-cur').textContent=d.speed_pct;\n"
    "    }}).catch(function(){});\n"
    "}\n"
    "document.getElementById('auto-lidfront-in').addEventListener('input',function(){this.dataset.touched='1';});\n"
    "document.getElementById('auto-stopmm-in').addEventListener('input',function(){this.dataset.touched='1';});\n"
    "document.getElementById('auto-scandeg-in').addEventListener('input',function(){this.dataset.touched='1';});\n"
    "document.getElementById('auto-passms-in').addEventListener('input',function(){this.dataset.touched='1';});\n"
    "document.getElementById('auto-travms-in').addEventListener('input',function(){this.dataset.touched='1';});\n"
    "function setLidar(){\n"
    "  var fd=parseInt(document.getElementById('auto-lidfront-in').value);\n"
    "  var sm=parseInt(document.getElementById('auto-stopmm-in').value);\n"
    "  var sd=parseInt(document.getElementById('auto-scandeg-in').value);\n"
    "  var pm=parseInt(document.getElementById('auto-passms-in').value);\n"
    "  var tm=parseInt(document.getElementById('auto-travms-in').value);\n"
    "  var body={};\n"
    "  if(!isNaN(fd))body.front_deg=fd;\n"
    "  if(!isNaN(sm))body.stop_mm=sm;\n"
    "  if(!isNaN(sd))body.scan_deg=sd;\n"
    "  if(!isNaN(pm))body.pass_ms=pm;\n"
    "  if(!isNaN(tm))body.traverse_ms=tm;\n"
    "  fetch('/api/autonomy/lidar',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})\n"
    "    .then(function(r){return r.json();})\n"
    "    .then(function(d){\n"
    "      var lfi=document.getElementById('auto-lidfront-in');delete lfi.dataset.touched;\n"
    "      var smi=document.getElementById('auto-stopmm-in');delete smi.dataset.touched;\n"
    "      var sdi=document.getElementById('auto-scandeg-in');delete sdi.dataset.touched;\n"
    "      var pmi=document.getElementById('auto-passms-in');delete pmi.dataset.touched;\n"
    "      var tmi=document.getElementById('auto-travms-in');delete tmi.dataset.touched;\n"
    "      if(d&&typeof d.lid_front_deg!=='undefined'){lfi.value=d.lid_front_deg;smi.value=d.front_stop_mm;}\n"
    "      if(d&&typeof d.scan_deg!=='undefined'){sdi.value=d.scan_deg;pmi.value=d.pass_ms;}\n"
    "      if(d&&typeof d.traverse_ms!=='undefined'){tmi.value=d.traverse_ms;}\n"
    "    }).catch(function(){});\n"
    "}\n"
    "function setHallActiveLow(){\n"
    "  var al=document.getElementById('hl-alow').checked;\n"
    "  fetch('/api/hall/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({active_low:al})})\n"
    "    .then(function(r){return r.json();})\n"
    "    .then(function(d){if(d&&typeof d.active_low!=='undefined'){document.getElementById('hl-alow').checked=d.active_low;}})\n"
    "    .catch(function(){});\n"
    "}\n"
    "function setHallManual(){\n"
    "  var en=document.getElementById('hl-manual').checked;\n"
    "  fetch('/api/hall/manual',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:en})})\n"
    "    .then(function(r){return r.json();})\n"
    "    .then(function(d){if(d&&typeof d.manual_enabled!=='undefined'){document.getElementById('hl-manual').checked=d.manual_enabled;}})\n"
    "    .catch(function(){});\n"
    "}\n"
    "\n"
    "var ltOn=false;\n"
    "function ltDir(){\n"
    "  var r=document.querySelector('input[name=\"lt-dir\"]:checked');\n"
    "  return r?r.value:'forward';\n"
    "}\n"
    "function updateLineTest(running,st,forward){\n"
    "  ltOn=running;\n"
    "  var b=document.getElementById('lt-btn');\n"
    "  b.innerHTML=running?'&#9209; Zatrzymaj test':'&#9654; Uruchom test';\n"
    "  b.style.background=running?'#3d1212':'#21262d';\n"
    "  b.style.borderColor=running?'#f85149':'#30363d';\n"
    "  b.style.color=running?'#f85149':'#c9d1d9';\n"
    "  document.getElementById('lt-state').textContent=st;\n"
    "  document.querySelectorAll('input[name=\"lt-dir\"]').forEach(function(el){el.disabled=running;});\n"
    "  if(!running && typeof forward!=='undefined'){\n"
    "    var want=forward?'forward':'backward';\n"
    "    var el=document.querySelector('input[name=\"lt-dir\"][value=\"'+want+'\"]');\n"
    "    var ae=document.activeElement;\n"
    "    if(el && !(ae && ae.name==='lt-dir')) el.checked=true;\n"
    "  }\n"
    "}\n"
    "function lineTestToggle(){\n"
    "  fetch('/api/line_test',{method:'POST',headers:{'Content-Type':'application/json'},\n"
    "    body:JSON.stringify({enable:!ltOn,direction:ltDir()})})\n"
    "    .then(function(r){return r.json();})\n"
    "    .then(function(d){updateLineTest(d.running,d.state,d.forward);})\n"
    "    .catch(function(){});\n"
    "}\n"
    "\n"
    "var autoOn=false;\n"
    "function updateAuto(en,st,logCount){\n"
    "  autoOn=en;\n"
    "  var b=document.getElementById('auto-btn');\n"
    "  b.innerHTML=en?'&#9209; Zatrzymaj autonomi\\u0119':'&#9654; Symuluj autonomi\\u0119';\n"
    "  b.style.background=en?'#3d1212':'#21262d';\n"
    "  b.style.borderColor=en?'#f85149':'#30363d';\n"
    "  b.style.color=en?'#f85149':'#c9d1d9';\n"
    "  document.getElementById('auto-state').textContent=st;\n"
    "  if(typeof logCount!=='undefined'){\n"
    "    document.getElementById('auto-logcount').textContent=logCount;\n"
    "  }\n"
    "}\n"
    "function autoToggle(){\n"
    "  fetch('/api/autonomy',{method:'POST',headers:{'Content-Type':'application/json'},\n"
    "    body:JSON.stringify({enable:!autoOn})})\n"
    "    .then(function(r){return r.json();})\n"
    "    .then(function(d){updateAuto(d.enabled,d.state);})\n"
    "    .catch(function(){});\n"
    "}\n"
    "\n"
    "function esc(s){return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');}\n"
    "function colorize(t){\n"
    "  return t.split('\\n').map(function(ln){\n"
    "    var m=ln.match(/^([EWID]) /);var c=m?('l'+m[1]):'lo';\n"
    "    return '<span class=\"'+c+'\">'+esc(ln)+'</span>';\n"
    "  }).join('\\n');\n"
    "}\n"
    "function pollLogs(){\n"
    "  if(!document.getElementById('mon-on').checked)return;\n"
    "  fetch('/api/logs').then(function(r){return r.text();}).then(function(t){\n"
    "    var box=document.getElementById('logbox');\n"
    "    var atBottom=box.scrollTop+box.clientHeight>=box.scrollHeight-20;\n"
    "    box.innerHTML=colorize(t);\n"
    "    document.getElementById('mon-status').textContent=t.length+' B';\n"
    "    if(document.getElementById('mon-scroll').checked&&atBottom){box.scrollTop=box.scrollHeight;}\n"
    "  }).catch(function(){});\n"
    "}\n"
    "function clearLogs(){fetch('/api/logs/clear',{method:'POST'}).then(function(){document.getElementById('logbox').innerHTML='';});}\n"
    "\n"
    "poll();\n"
    "setInterval(poll,100);\n"
    "pollLogs();\n"
    "setInterval(pollLogs,1000);\n"
    "</script>\n"
    "</body>\n"
    "</html>\n";

/* Pomocnicza: odczyt ciała żądania POST do bufora. */
static int read_body(httpd_req_t *req, char *buf, size_t maxlen) {
    int total = req->content_len;
    if (total <= 0 || total >= (int)maxlen) return -1;
    int received = 0, ret;
    while (received < total) {
        ret = httpd_req_recv(req, buf + received, total - received);
        if (ret <= 0) return -1;
        received += ret;
    }
    buf[received] = '\0';
    return received;
}

/* GET / - strona dashboardu. */
static esp_err_t handle_root(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, DASHBOARD_HTML, sizeof(DASHBOARD_HTML) - 1);
    return ESP_OK;
}

/* GET /api/sensors - zbiorczy odczyt wszystkich czujników jako JSON. */
static esp_err_t handle_sensors(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();

    /* LIDAR */
    lidar_packet_t lpkt = lidar_get_last_packet();
    cJSON *lidar = cJSON_CreateObject();
    cJSON_AddNumberToObject(lidar, "min_distance_mm", lidar_get_min_distance_mm());
    cJSON_AddNumberToObject(lidar, "angle_hundredths",
        lpkt.valid && lpkt.count > 0 ? lpkt.points[0].angle_hundredths : 0);
    cJSON_AddNumberToObject(lidar, "speed_rpm", lpkt.speed_rpm);
    cJSON_AddBoolToObject(lidar, "valid", lpkt.valid);
    cJSON_AddItemToObject(root, "lidar", lidar);

    /* Pirometr */
    pyrometer_data_t pd = pyrometer_get_last();
    cJSON *pyro = cJSON_CreateObject();
    cJSON_AddNumberToObject(pyro, "object_temp",  (double)pd.object_temp);
    cJSON_AddNumberToObject(pyro, "ambient_temp", (double)pd.ambient_temp);
    cJSON_AddBoolToObject(pyro, "initialized",     pd.initialized);
    cJSON_AddBoolToObject(pyro, "searching",       pd.searching);
    cJSON_AddBoolToObject(pyro, "hot_detected",    pd.hot_detected);
    cJSON_AddItemToObject(root, "pyrometer", pyro);

    /* IMU */
    imu_data_t id = imu_get_last();
    cJSON *imu = cJSON_CreateObject();
    cJSON_AddNumberToObject(imu, "accel_x", (double)id.accel_x);
    cJSON_AddNumberToObject(imu, "accel_y", (double)id.accel_y);
    cJSON_AddNumberToObject(imu, "accel_z", (double)id.accel_z);
    cJSON_AddNumberToObject(imu, "gyro_x",  (double)id.gyro_x);
    cJSON_AddNumberToObject(imu, "gyro_y",  (double)id.gyro_y);
    cJSON_AddNumberToObject(imu, "gyro_z",  (double)id.gyro_z);
    cJSON_AddNumberToObject(imu, "mag_x",   (double)id.mag_x);
    cJSON_AddNumberToObject(imu, "mag_y",   (double)id.mag_y);
    cJSON_AddNumberToObject(imu, "mag_z",   (double)id.mag_z);
    cJSON_AddNumberToObject(imu, "azimuth_deg", (double)id.azimuth_deg);
    cJSON_AddNumberToObject(imu, "temp",    (double)id.temp);
    cJSON_AddBoolToObject(imu, "initialized", id.initialized);
    cJSON_AddBoolToObject(imu, "mag_initialized", id.mag_initialized);

    imu_mag_cal_status_t mcal = imu_mag_get_cal_status();
    cJSON *mag_cal = cJSON_CreateObject();
    cJSON_AddBoolToObject(mag_cal,   "active",       mcal.active);
    cJSON_AddNumberToObject(mag_cal, "remaining_ms", mcal.remaining_ms);
    cJSON_AddNumberToObject(mag_cal, "offset_x",     (double)mcal.offset_x);
    cJSON_AddNumberToObject(mag_cal, "offset_y",     (double)mcal.offset_y);
    cJSON_AddNumberToObject(mag_cal, "phase_deg",    (double)mcal.phase_deg);
    cJSON_AddItemToObject(imu, "mag_cal", mag_cal);

    cJSON_AddItemToObject(root, "imu", imu);

    /* INA219 (prąd/napięcie) */
    ina219_data_t in = ina219_get_last();
    cJSON *ina = cJSON_CreateObject();
    cJSON_AddNumberToObject(ina, "bus_voltage_v",    (double)in.bus_voltage_v);
    cJSON_AddNumberToObject(ina, "shunt_voltage_mv", (double)in.shunt_voltage_mv);
    cJSON_AddNumberToObject(ina, "current_ma",       (double)in.current_ma);
    cJSON_AddNumberToObject(ina, "power_mw",         (double)in.power_mw);
    cJSON_AddNumberToObject(ina, "address",          in.address);
    cJSON_AddBoolToObject(ina, "initialized",        in.initialized);
    cJSON_AddItemToObject(root, "ina219", ina);

    /* SHT40 (temperatura/wilgotność) */
    sht40_data_t sh = sht40_get_last();
    cJSON *sht = cJSON_CreateObject();
    cJSON_AddNumberToObject(sht, "temperature_c", (double)sh.temperature_c);
    cJSON_AddNumberToObject(sht, "humidity_pct",  (double)sh.humidity_pct);
    cJSON_AddNumberToObject(sht, "address",       sh.address);
    cJSON_AddBoolToObject(sht, "initialized",     sh.initialized);
    cJSON_AddItemToObject(root, "sht40", sht);

    /* Odometria */
    odometry_data_t od = odometry_get();
    cJSON *odo = cJSON_CreateObject();
    cJSON_AddNumberToObject(odo, "pulses_left",    od.pulses_left);
    cJSON_AddNumberToObject(odo, "pulses_right",   od.pulses_right);
    cJSON_AddNumberToObject(odo, "dist_left_mm",   (double)od.dist_left_mm);
    cJSON_AddNumberToObject(odo, "dist_right_mm",  (double)od.dist_right_mm);
    cJSON_AddNumberToObject(odo, "dist_total_mm",  (double)od.dist_total_mm);
    cJSON_AddItemToObject(root, "odometry", odo);

    /* Czujnik Halla mety - cyfrowy (49E + LM393 + potencjometr, DO na GPIO) */
    cJSON *hallj = cJSON_CreateObject();
    cJSON_AddNumberToObject(hallj, "do_raw",         hall_finish_do_raw());
    cJSON_AddBoolToObject(hallj,   "detected",       hall_finish_detected());
    cJSON_AddBoolToObject(hallj,   "active_low",     hall_finish_get_active_low());
    cJSON_AddBoolToObject(hallj,   "manual_enabled", hall_finish_get_manual_enabled());
    cJSON_AddItemToObject(root, "hall", hallj);

    /* Czujniki linii - 4x analogowo przez ADS1115 (A0 przód-P, A1 przód-L,
     * A2 tył-L, A3 tył-P) */
    line_sensor_data_t ls = line_sensor_read();
    cJSON *line = cJSON_CreateObject();
    cJSON_AddBoolToObject(line,   "front_left",    ls.front_left);
    cJSON_AddBoolToObject(line,   "front_right",   ls.front_right);
    cJSON_AddBoolToObject(line,   "back_left",     ls.back_left);
    cJSON_AddBoolToObject(line,   "back_right",    ls.back_right);
    cJSON_AddNumberToObject(line, "front_left_v",  (double)ls.front_left_v);
    cJSON_AddNumberToObject(line, "front_right_v", (double)ls.front_right_v);
    cJSON_AddNumberToObject(line, "back_left_v",   (double)ls.back_left_v);
    cJSON_AddNumberToObject(line, "back_right_v",  (double)ls.back_right_v);
    cJSON_AddItemToObject(root, "line_sensors", line);

    /* Silniki */
    cJSON *motors = cJSON_CreateObject();
    cJSON_AddNumberToObject(motors, "left",  motor_get_left_speed());
    cJSON_AddNumberToObject(motors, "right", motor_get_right_speed());
    cJSON_AddItemToObject(root, "motors", motors);

    /* Autonomia */
    cJSON *autoj = cJSON_CreateObject();
    cJSON_AddBoolToObject(autoj, "enabled", autonomy_is_enabled());
    cJSON_AddStringToObject(autoj, "state", autonomy_state_str());
    cJSON_AddNumberToObject(autoj, "log_count", autonomy_log_count());
    cJSON_AddNumberToObject(autoj, "speed_pct", autonomy_get_speed_pct());
    cJSON_AddNumberToObject(autoj, "run_time_s", (double)autonomy_get_run_time_s());
    cJSON_AddNumberToObject(autoj, "run_energy_mwh", (double)autonomy_get_run_energy_mwh());
    /* Krok 1/2: podglad estymatora kursu i 8 sektorow LIDAR [mm]. */
    cJSON_AddNumberToObject(autoj, "heading_deg",   (double)autonomy_get_heading_deg());
    cJSON_AddNumberToObject(autoj, "gyro_z_dps",    (double)autonomy_get_gyro_z_dps());
    cJSON_AddNumberToObject(autoj, "gyro_z_filt_dps", (double)autonomy_get_gyro_z_filt_dps());
    cJSON_AddNumberToObject(autoj, "gyro_bias_dps", (double)autonomy_get_gyro_bias_dps());
    cJSON_AddNumberToObject(autoj, "heading_target_deg", (double)autonomy_get_heading_target_deg());
    /* Krok 6: kontrola korytarza (LIDAR). */
    cJSON_AddNumberToObject(autoj, "lid_front_deg", autonomy_get_lid_front_deg());
    cJSON_AddNumberToObject(autoj, "front_stop_mm", autonomy_get_front_stop_mm());
    cJSON_AddNumberToObject(autoj, "corridor_mm",   autonomy_get_corridor_mm());
    /* Krok 7: parametry omijania (follow-the-gap). */
    cJSON_AddNumberToObject(autoj, "scan_max_deg",  autonomy_get_scan_max_deg());
    cJSON_AddNumberToObject(autoj, "avoid_pass_ms", autonomy_get_avoid_pass_ms());
    /* Krok 9: przejscie faza 1->2 (gorna krawedz toru). */
    cJSON_AddNumberToObject(autoj, "forward_ms",    autonomy_get_forward_ms());
    cJSON_AddNumberToObject(autoj, "traverse_ms",   autonomy_get_traverse_ms());
    {
        int16_t secs[8];
        autonomy_get_lidar_sectors_mm(secs);
        cJSON *sarr = cJSON_CreateArray();
        for (int i = 0; i < 8; i++)
            cJSON_AddItemToArray(sarr, cJSON_CreateNumber(secs[i]));
        cJSON_AddItemToObject(autoj, "lidar_sectors_mm", sarr);
    }
    cJSON_AddItemToObject(root, "autonomy", autoj);

    /* Test wykrywania linii */
    cJSON *ltj = cJSON_CreateObject();
    cJSON_AddBoolToObject(ltj,   "running", line_test_is_running());
    cJSON_AddStringToObject(ltj, "state",   line_test_state_str());
    cJSON_AddBoolToObject(ltj,   "forward", line_test_get_forward());
    cJSON_AddStringToObject(ltj, "hit",     line_test_hit_str());
    cJSON_AddItemToObject(root, "line_test", ltj);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, json_str);
    free(json_str);
    return ESP_OK;
}

/* POST /api/motor {"left":-100..100,"right":-100..100} - sterowanie silnikami. */
static esp_err_t handle_motor(httpd_req_t *req) {
    autonomy_set_enabled(false);   /* ręczne sterowanie wyłącza autonomię (kill-switch) */
    line_test_stop();              /* ...oraz test wykrywania linii */
    char buf[128];
    if (read_body(req, buf, sizeof(buf)) < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }
    cJSON *j = cJSON_Parse(buf);
    if (j) {
        cJSON *l = cJSON_GetObjectItem(j, "left");
        cJSON *r = cJSON_GetObjectItem(j, "right");
        if (cJSON_IsNumber(l)) motor_set_left((int)l->valuedouble);
        if (cJSON_IsNumber(r)) motor_set_right((int)r->valuedouble);
        cJSON_Delete(j);
    }
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* POST /api/motor/stop - awaryjne zatrzymanie. */
static esp_err_t handle_motor_stop(httpd_req_t *req) {
    autonomy_set_enabled(false);   /* STOP wyłącza także autonomię */
    line_test_stop();              /* ...oraz test wykrywania linii */
    motor_stop();
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* POST /api/led {"red":0/1,"yellow":0/1,"green":0/1} - sterowanie diodami. */
static esp_err_t handle_led(httpd_req_t *req) {
    char buf[128];
    if (read_body(req, buf, sizeof(buf)) < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }
    cJSON *j = cJSON_Parse(buf);
    if (j) {
        cJSON *r = cJSON_GetObjectItem(j, "red");
        cJSON *y = cJSON_GetObjectItem(j, "yellow");
        cJSON *g = cJSON_GetObjectItem(j, "green");
        if (r) led_set_red(cJSON_IsTrue(r) || (cJSON_IsNumber(r) && r->valuedouble != 0));
        if (y) led_set_yellow(cJSON_IsTrue(y) || (cJSON_IsNumber(y) && y->valuedouble != 0));
        if (g) led_set_green(cJSON_IsTrue(g) || (cJSON_IsNumber(g) && g->valuedouble != 0));
        cJSON_Delete(j);
    }
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* POST /api/odometry/reset - zerowanie liczników odometrii, flagi mety
 * i trybu szukania obiektu cieplnego pirometrem - czyli reset całego
 * "przebiegu" przed kolejną próbą, nie tylko samej odometrii. */
static esp_err_t handle_odo_reset(httpd_req_t *req) {
    odometry_reset();
    pyrometer_reset_search();
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* POST /api/imu/mag_cal/start {opcjonalnie "duration_ms"} - start zbierania
 * min/max magnetometru (krok 1 kalibracji) - obróć pojazd o pełny obrót. */
static esp_err_t handle_mag_cal_start(httpd_req_t *req) {
    int dur = 15000;
    if (req->content_len > 0) {
        char buf[64];
        if (read_body(req, buf, sizeof(buf)) > 0) {
            cJSON *j = cJSON_Parse(buf);
            if (j) {
                cJSON *d = cJSON_GetObjectItem(j, "duration_ms");
                if (cJSON_IsNumber(d)) dur = (int)d->valuedouble;
                cJSON_Delete(j);
            }
        }
    }
    imu_mag_calibration_start((uint32_t)dur);
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* POST /api/imu/mag_cal/set_north - krok 2 kalibracji: bieżący kierunek = 0°. */
static esp_err_t handle_mag_cal_set_north(httpd_req_t *req) {
    imu_mag_set_north();
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* GET /api/logs - podgląd logów (monitor szeregowy przez WWW). */
static esp_err_t handle_logs(httpd_req_t *req) {
    size_t cap = 8200;
    char *buf = malloc(cap);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
        return ESP_FAIL;
    }
    size_t n = web_monitor_dump(buf, cap);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, buf, n);
    free(buf);
    return ESP_OK;
}

/* POST /api/logs/clear - czyszczenie bufora logów. */
static esp_err_t handle_logs_clear(httpd_req_t *req) {
    web_monitor_clear();
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* POST /api/buzzer {opcjonalnie "freq","duration_ms"} - sygnał dźwiękowy. */
static esp_err_t handle_buzzer(httpd_req_t *req) {
    int freq = 2000, dur = 200;
    if (req->content_len > 0) {
        char buf[128];
        if (read_body(req, buf, sizeof(buf)) > 0) {
            cJSON *j = cJSON_Parse(buf);
            if (j) {
                cJSON *f = cJSON_GetObjectItem(j, "freq");
                cJSON *d = cJSON_GetObjectItem(j, "duration_ms");
                if (cJSON_IsNumber(f)) freq = (int)f->valuedouble;
                if (cJSON_IsNumber(d)) dur  = (int)d->valuedouble;
                cJSON_Delete(j);
            }
        }
    }
    buzzer_tone((uint32_t)freq, (uint32_t)dur);
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* POST /api/buzzer/song - odtwarza w pętli wbudowany jingle "furgonetki z lodami". */
static esp_err_t handle_buzzer_song(httpd_req_t *req) {
    buzzer_play_ice_cream_song();
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* POST /api/buzzer/stop - przerywa granie (w tym pętlę melodii). */
static esp_err_t handle_buzzer_stop(httpd_req_t *req) {
    buzzer_off();
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* GET /api/lidar/scan?since=N - strumień punktów LIDAR do budowy mapy przeszkód.
 * Zwraca kompaktowy JSON: {"seq":S,"rpm":R,"n":K,"pts":[a0,d0,a1,d1,...]}.
 *   seq - bieżący licznik sekwencyjny; podaj jako ?since= przy kolejnym żądaniu,
 *         aby pobierać tylko nowe punkty (bez duplikatów).
 *   pts - spłaszczona tablica par (kąt [setne stopnia], odległość [mm]). */
static esp_err_t handle_lidar_scan(httpd_req_t *req) {
    uint32_t since = 0;
    char q[48];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char v[24];
        if (httpd_query_key_value(q, "since", v, sizeof(v)) == ESP_OK)
            since = (uint32_t)strtoul(v, NULL, 10);
    }

    /* Okno odczytu: najnowsze do 512 punktów na żądanie (~jeden obrót LD06). */
    static lidar_scan_point_t pts[512];
    uint32_t seq = 0;
    uint16_t n = lidar_copy_scan(since, pts, 512, &seq);

    /* Ręczne budowanie kompaktowego JSON - cJSON dla setek liczb jest kosztowny
       pamięciowo i czasowo. */
    size_t cap = 64 + (size_t)n * 14;   /* ok. 12 znaków na punkt: "65535,65535," */
    char *buf = malloc(cap);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
        return ESP_FAIL;
    }

    int len = snprintf(buf, cap, "{\"seq\":%lu,\"rpm\":%u,\"n\":%u,\"pts\":[",
                       (unsigned long)seq, lidar_get_speed_rpm(), n);
    for (uint16_t i = 0; i < n && len < (int)cap; i++) {
        len += snprintf(buf + len, cap - len, "%s%u,%u",
                        i ? "," : "", pts[i].angle_hundredths, pts[i].distance_mm);
    }
    if (len < (int)cap) len += snprintf(buf + len, cap - len, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, buf, len);
    free(buf);
    return ESP_OK;
}

/* GET /api/autonomy/log.csv - log ostatniego/bieżącego przejazdu (bufor RAM,
 * zerowany przy starcie każdego przejazdu; pobierz zaraz po jeździe, zanim
 * ruszy następna). Wysyłane w kawałkach (chunked), aby nie wymagać dużego
 * bufora nawet przy ~2000 wierszach. */
static esp_err_t handle_autonomy_log_csv(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Disposition",
                        "attachment; filename=\"przejazd_log.csv\"");

    static const char *header =
        "czas_ms,stan,silnik_L_proc,silnik_R_proc,"
        "gyro_x_dps,gyro_y_dps,gyro_z_dps,gyro_z_filt_dps,gyro_bias_dps,kurs_deg,"
        "lidar_przod_mm,lidar_przodL_mm,lidar_lewo_mm,lidar_tylL_mm,"
        "lidar_tyl_mm,lidar_tylP_mm,lidar_prawo_mm,lidar_przodP_mm,korytarz_mm,"
        "linia_PP_mV,linia_PL_mV,linia_TL_mV,linia_TP_mV,hall_do,hall_wykryto,"
        "temp_obiekt_C,temp_otoczenie_C,delta_C\r\n";
    httpd_resp_send_chunk(req, header, strlen(header));

    char buf[384];
    uint32_t n = autonomy_log_count();
    for (uint32_t i = 0; i < n; i++) {
        autonomy_log_rec_t r;
        if (!autonomy_log_get(i, &r)) break;
        float obj = r.obj_temp_x10 / 10.0f;
        float amb = r.amb_temp_x10 / 10.0f;
        int len = snprintf(buf, sizeof(buf),
            "%lu,%s,%d,%d,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f\r\n",
            (unsigned long)r.t_ms, autonomy_log_state_name(r.state),
            r.motor_l, r.motor_r,
            r.gyro_x_x10 / 10.0f, r.gyro_y_x10 / 10.0f, r.gyro_z_x10 / 10.0f,
            r.gyro_zf_x10 / 10.0f, r.gyro_bias_x10 / 10.0f, r.heading_x10 / 10.0f,
            r.lidar_mm[0], r.lidar_mm[1], r.lidar_mm[2], r.lidar_mm[3],
            r.lidar_mm[4], r.lidar_mm[5], r.lidar_mm[6], r.lidar_mm[7], r.corridor_mm,
            r.line_fr_mv, r.line_fl_mv, r.line_bl_mv, r.line_br_mv, r.hall_do, r.hall_hit,
            obj, amb, obj - amb);
        httpd_resp_send_chunk(req, buf, len);
    }
    httpd_resp_send_chunk(req, NULL, 0);   /* zakończ odpowiedź chunked */
    return ESP_OK;
}

/* POST /api/autonomy {"enable":true/false} (akceptuje też "run"). Wywoływane
 * przyciskiem autonomii w dashboardzie. Brak ciała żądania = przełącz stan. */
static esp_err_t handle_autonomy(httpd_req_t *req) {
    bool target = !autonomy_is_enabled();   /* domyślnie: przełącz */
    if (req->content_len > 0) {
        char buf[128];
        if (read_body(req, buf, sizeof(buf)) > 0) {
            cJSON *j = cJSON_Parse(buf);
            if (j) {
                cJSON *e = cJSON_GetObjectItem(j, "enable");
                if (!e) e = cJSON_GetObjectItem(j, "run");
                if (e) target = cJSON_IsTrue(e) ||
                                (cJSON_IsNumber(e) && e->valuedouble != 0);
                cJSON_Delete(j);
            }
        }
    }
    if (target) line_test_stop();   /* autonomia i test wykrywania linii wykluczają się */
    autonomy_set_enabled(target);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "enabled", autonomy_is_enabled());
    cJSON_AddStringToObject(root, "state", autonomy_state_str());
    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, s);
    free(s);
    return ESP_OK;
}

/* POST /api/autonomy/heading {"heading_deg":N} - Krok 4: ustawia zadany kurs
 * (cel regulatora utrzymania kursu w ST_CRUISE) względem kierunku startowego.
 * + = w lewo. Moduł autonomii przycina do ±90°. Odpowiada aktualną wartością
 * (po przycięciu). Zastępuje dawne /api/autonomy/azimuth. */
static esp_err_t handle_autonomy_heading(httpd_req_t *req) {
    if (req->content_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing body");
        return ESP_FAIL;
    }
    char buf[64];
    if (read_body(req, buf, sizeof(buf)) <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }
    cJSON *j = cJSON_Parse(buf);
    if (!j) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
        return ESP_FAIL;
    }
    cJSON *a = cJSON_GetObjectItem(j, "heading_deg");
    if (!cJSON_IsNumber(a)) {
        cJSON_Delete(j);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing heading_deg");
        return ESP_FAIL;
    }
    autonomy_set_heading_target_deg((float)a->valuedouble);
    cJSON_Delete(j);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "heading_target_deg", (double)autonomy_get_heading_target_deg());
    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    free(out);
    return ESP_OK;
}

/* POST /api/autonomy/speed {"speed_pct":N} - ustawia moc silników jazdy
 * autonomicznej na wprost/do tyłu (ST_CRUISE/ST_LINE_BACKUP w autonomy.c).
 * Wartość w procentach mocy (0..100); moduł autonomii przycina ją do tego
 * zakresu. Odpowiada aktualnie obowiązującą wartością (po przycięciu). */
static esp_err_t handle_autonomy_speed(httpd_req_t *req) {
    if (req->content_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing body");
        return ESP_FAIL;
    }
    char buf[64];
    if (read_body(req, buf, sizeof(buf)) <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }
    cJSON *j = cJSON_Parse(buf);
    if (!j) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
        return ESP_FAIL;
    }
    cJSON *s = cJSON_GetObjectItem(j, "speed_pct");
    if (!cJSON_IsNumber(s)) {
        cJSON_Delete(j);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing speed_pct");
        return ESP_FAIL;
    }
    autonomy_set_speed_pct((int)s->valuedouble);
    cJSON_Delete(j);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "speed_pct", autonomy_get_speed_pct());
    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    free(out);
    return ESP_OK;
}

/* POST /api/autonomy/lidar {"front_deg":N,"stop_mm":N,"scan_deg":N,"pass_ms":N}
 * Krok 6/7: kalibracja kontroli korytarza i parametry omijania. front_deg =
 * offset przodu głowicy LIDAR [°] (dobierany z tools/lidar_map.py); stop_mm =
 * próg zatrzymania przed przeszkodą [mm] (przycinany do [150,1500]); scan_deg =
 * połowa zakresu skanu szczelin [°] (Krok 7, przycinany do [30,120]); pass_ms =
 * czas jazdy przez szczelinę [ms] (Krok 7, przycinany do [500,6000]). Wszystkie
 * pola opcjonalne. Odpowiada aktualnymi wartościami. */
static esp_err_t handle_autonomy_lidar(httpd_req_t *req) {
    if (req->content_len > 0) {
        char buf[160];
        if (read_body(req, buf, sizeof(buf)) > 0) {
            cJSON *j = cJSON_Parse(buf);
            if (j) {
                cJSON *fd = cJSON_GetObjectItem(j, "front_deg");
                cJSON *sm = cJSON_GetObjectItem(j, "stop_mm");
                cJSON *sd = cJSON_GetObjectItem(j, "scan_deg");
                cJSON *pm = cJSON_GetObjectItem(j, "pass_ms");
                cJSON *tm = cJSON_GetObjectItem(j, "traverse_ms");
                if (cJSON_IsNumber(fd)) autonomy_set_lid_front_deg((int)fd->valuedouble);
                if (cJSON_IsNumber(sm)) autonomy_set_front_stop_mm((int)sm->valuedouble);
                if (cJSON_IsNumber(sd)) autonomy_set_scan_max_deg((int)sd->valuedouble);
                if (cJSON_IsNumber(pm)) autonomy_set_avoid_pass_ms((int)pm->valuedouble);
                if (cJSON_IsNumber(tm)) autonomy_set_traverse_ms((int)tm->valuedouble);
                cJSON_Delete(j);
            }
        }
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "lid_front_deg", autonomy_get_lid_front_deg());
    cJSON_AddNumberToObject(root, "front_stop_mm", autonomy_get_front_stop_mm());
    cJSON_AddNumberToObject(root, "scan_deg",      autonomy_get_scan_max_deg());
    cJSON_AddNumberToObject(root, "pass_ms",       autonomy_get_avoid_pass_ms());
    cJSON_AddNumberToObject(root, "traverse_ms",   autonomy_get_traverse_ms());
    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    free(out);
    return ESP_OK;
}

/* POST /api/hall/config {"active_low":true/false} - polaryzacja cyfrowego
 * wyjścia DO czujnika Halla mety: true = "meta" gdy DO w stanie LOW (typowe
 * moduły LM393), false = gdy HIGH. Próg jest sprzętowy (potencjometr na
 * module) - tu tylko interpretacja stanu. Brak/niepełne ciało pozostawia
 * bieżącą wartość. Odpowiada aktualnym stanem. */
static esp_err_t handle_hall_config(httpd_req_t *req) {
    if (req->content_len > 0) {
        char buf[64];
        if (read_body(req, buf, sizeof(buf)) > 0) {
            cJSON *j = cJSON_Parse(buf);
            if (j) {
                cJSON *e = cJSON_GetObjectItem(j, "active_low");
                if (e) hall_finish_set_active_low(cJSON_IsTrue(e) ||
                                (cJSON_IsNumber(e) && e->valuedouble != 0));
                cJSON_Delete(j);
            }
        }
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "active_low", hall_finish_get_active_low());
    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, s);
    free(s);
    return ESP_OK;
}

/* POST /api/hall/manual {"enabled":true/false} - włącza/wyłącza reakcję na
 * Hall mety podczas jazdy RĘCZNEJ (ton finiszu + zielona dioda + tryb
 * szukania ciepła w main.c). Nie dotyczy autonomii - tam meta jest
 * sygnalizowana decyzją autonomii. Brak ciała = przełącz stan. */
static esp_err_t handle_hall_manual(httpd_req_t *req) {
    bool target = !hall_finish_get_manual_enabled();   /* domyślnie: przełącz */
    if (req->content_len > 0) {
        char buf[64];
        if (read_body(req, buf, sizeof(buf)) > 0) {
            cJSON *j = cJSON_Parse(buf);
            if (j) {
                cJSON *e = cJSON_GetObjectItem(j, "enabled");
                if (e) target = cJSON_IsTrue(e) ||
                                (cJSON_IsNumber(e) && e->valuedouble != 0);
                cJSON_Delete(j);
            }
        }
    }
    hall_finish_set_manual_enabled(target);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "manual_enabled", hall_finish_get_manual_enabled());
    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, s);
    free(s);
    return ESP_OK;
}

/* POST /api/line_test - test wykrywania linii czujnikami odbiciowymi. Ciało:
 *   {"enable":true/false}          - włącz/wyłącz (akceptuje też "run");
 *                                    brak ciała = przełącz stan
 *   {"direction":"forward"/"backward"} lub {"forward":true/false}
 *                                  - kierunek jazdy (uwzględniany przy starcie)
 * Włączenie testu wyłącza autonomię (i odwrotnie - patrz handle_autonomy). */
static esp_err_t handle_line_test(httpd_req_t *req) {
    bool target  = !line_test_is_running();   /* domyślnie: przełącz */
    bool forward = line_test_get_forward();
    if (req->content_len > 0) {
        char buf[128];
        if (read_body(req, buf, sizeof(buf)) > 0) {
            cJSON *j = cJSON_Parse(buf);
            if (j) {
                cJSON *e = cJSON_GetObjectItem(j, "enable");
                if (!e) e = cJSON_GetObjectItem(j, "run");
                if (e) target = cJSON_IsTrue(e) ||
                                (cJSON_IsNumber(e) && e->valuedouble != 0);

                cJSON *f = cJSON_GetObjectItem(j, "forward");
                if (f) forward = cJSON_IsTrue(f) ||
                                 (cJSON_IsNumber(f) && f->valuedouble != 0);

                cJSON *dir = cJSON_GetObjectItem(j, "direction");
                if (cJSON_IsString(dir) && dir->valuestring) {
                    if (strcmp(dir->valuestring, "backward") == 0 ||
                        strcmp(dir->valuestring, "back") == 0 ||
                        strcmp(dir->valuestring, "tyl") == 0)
                        forward = false;
                    else if (strcmp(dir->valuestring, "forward") == 0 ||
                             strcmp(dir->valuestring, "przod") == 0)
                        forward = true;
                }
                cJSON_Delete(j);
            }
        }
    }

    if (target) {
        autonomy_set_enabled(false);   /* test i autonomia wykluczają się */
        line_test_start(forward);
    } else {
        line_test_stop();
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root,   "running", line_test_is_running());
    cJSON_AddStringToObject(root, "state",   line_test_state_str());
    cJSON_AddBoolToObject(root,   "forward", line_test_get_forward());
    cJSON_AddStringToObject(root, "hit",     line_test_hit_str());
    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, s);
    free(s);
    return ESP_OK;
}

esp_err_t http_server_start(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port        = 80;
    cfg.max_uri_handlers   = 22;
    cfg.stack_size         = 8192;

    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Nie mozna uruchomic serwera HTTP");
        return ESP_FAIL;
    }

    httpd_uri_t routes[] = {
        { .uri="/",                   .method=HTTP_GET,  .handler=handle_root       },
        { .uri="/api/sensors",        .method=HTTP_GET,  .handler=handle_sensors    },
        { .uri="/api/lidar/scan",     .method=HTTP_GET,  .handler=handle_lidar_scan },
        { .uri="/api/motor",          .method=HTTP_POST, .handler=handle_motor      },
        { .uri="/api/motor/stop",     .method=HTTP_POST, .handler=handle_motor_stop },
        { .uri="/api/led",            .method=HTTP_POST, .handler=handle_led        },
        { .uri="/api/odometry/reset", .method=HTTP_POST, .handler=handle_odo_reset  },
        { .uri="/api/imu/mag_cal/start",     .method=HTTP_POST, .handler=handle_mag_cal_start     },
        { .uri="/api/imu/mag_cal/set_north", .method=HTTP_POST, .handler=handle_mag_cal_set_north },
        { .uri="/api/logs",           .method=HTTP_GET,  .handler=handle_logs       },
        { .uri="/api/logs/clear",     .method=HTTP_POST, .handler=handle_logs_clear },
        { .uri="/api/buzzer",         .method=HTTP_POST, .handler=handle_buzzer     },
        { .uri="/api/buzzer/song",    .method=HTTP_POST, .handler=handle_buzzer_song },
        { .uri="/api/buzzer/stop",    .method=HTTP_POST, .handler=handle_buzzer_stop },
        { .uri="/api/autonomy",       .method=HTTP_POST, .handler=handle_autonomy   },
        { .uri="/api/autonomy/heading", .method=HTTP_POST, .handler=handle_autonomy_heading },
        { .uri="/api/autonomy/speed",   .method=HTTP_POST, .handler=handle_autonomy_speed },
        { .uri="/api/autonomy/lidar",   .method=HTTP_POST, .handler=handle_autonomy_lidar },
        { .uri="/api/autonomy/log.csv", .method=HTTP_GET, .handler=handle_autonomy_log_csv },
        { .uri="/api/hall/config",      .method=HTTP_POST, .handler=handle_hall_config },
        { .uri="/api/hall/manual",      .method=HTTP_POST, .handler=handle_hall_manual },
        { .uri="/api/line_test",        .method=HTTP_POST, .handler=handle_line_test },
    };

    int n_routes = sizeof(routes) / sizeof(routes[0]);
    for (int i = 0; i < n_routes; i++)
        httpd_register_uri_handler(s_server, &routes[i]);

    ESP_LOGI(TAG, "HTTP server uruchomiony na porcie 80");
    return ESP_OK;
}

void http_server_stop(void) {
    if (s_server) { httpd_stop(s_server); s_server = NULL; }
}
