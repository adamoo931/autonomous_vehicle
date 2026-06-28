#include "http_server.h"
#include "motor_driver.h"
#include "led_control.h"
#include "odometry.h"
#include "pyrometer.h"
#include "imu.h"
#include "ina219.h"
#include "sht40.h"
#include "buzzer.h"
#include "lidar.h"
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

// ── Embedded HTML dashboard ──────────────────────────────────
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
    "    <div>Meta: <span class=\"val\" id=\"py-fin\">-</span></div>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#128208; IMU ICM-20948</h2>\n"
    "    <div>Ax:<span class=\"val\" id=\"ax\">-</span> Ay:<span class=\"val\" id=\"ay\">-</span> Az:<span class=\"val\" id=\"az\">-</span> g</div>\n"
    "    <div>Gx:<span class=\"val\" id=\"gx\">-</span> Gy:<span class=\"val\" id=\"gy\">-</span> Gz:<span class=\"val\" id=\"gz\">-</span> &#176;/s</div>\n"
    "    <div>T: <span class=\"val\" id=\"imu-t\">-</span> &#176;C</div>\n"
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
    "    <h2>&#128207; Odometria</h2>\n"
    "    <div>Lewe: <span class=\"val\" id=\"od-l\">-</span> mm</div>\n"
    "    <div>Prawe: <span class=\"val\" id=\"od-r\">-</span> mm</div>\n"
    "    <div>Suma: <span class=\"val\" id=\"od-t\">-</span> mm</div>\n"
    "    <div>Impulsy L/P: <span class=\"val\" id=\"od-pl\">-</span> / <span class=\"val\" id=\"od-pr\">-</span></div>\n"
    "    <div>Hall meta: <span class=\"val\" id=\"od-fin\">-</span></div>\n"
    "    <br><button onclick=\"fetch('/api/odometry/reset',{method:'POST'})\">&#128260; Reset</button>\n"
    "  </div>\n"
    "  <div class=\"card\">\n"
    "    <h2>&#9633; Czujniki linii CNY70</h2>\n"
    "    <div>Prz&#243;d L: <span class=\"val\" id=\"ls-fl\">-</span></div>\n"
    "    <div>Prz&#243;d P: <span class=\"val\" id=\"ls-fr\">-</span></div>\n"
    "    <div>Ty&#322; L: <span class=\"val\" id=\"ls-bl\">-</span></div>\n"
    "    <div>Ty&#322; P: <span class=\"val\" id=\"ls-br\">-</span></div>\n"
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
    "      <button onclick=\"mv(0,spd())\">&#8679;</button>\n"
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
    "  </div>\n"
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
    "      document.getElementById('py-fin').innerHTML=d.pyrometer.finish_detected?'<span class=\"ok\">META!</span>':'Nie';\n"
    "    }\n"
    "    if(d.imu){\n"
    "      document.getElementById('ax').textContent=d.imu.accel_x.toFixed(2);\n"
    "      document.getElementById('ay').textContent=d.imu.accel_y.toFixed(2);\n"
    "      document.getElementById('az').textContent=d.imu.accel_z.toFixed(2);\n"
    "      document.getElementById('gx').textContent=d.imu.gyro_x.toFixed(1);\n"
    "      document.getElementById('gy').textContent=d.imu.gyro_y.toFixed(1);\n"
    "      document.getElementById('gz').textContent=d.imu.gyro_z.toFixed(1);\n"
    "      document.getElementById('imu-t').textContent=d.imu.temp.toFixed(1);\n"
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
    "      document.getElementById('od-fin').innerHTML=d.odometry.finish_detected?'<span class=\"ok\">WYKRYTA</span>':'Nie';\n"
    "    }\n"
    "    if(d.line_sensors){\n"
    "      var ls=d.line_sensors;\n"
    "      document.getElementById('ls-fl').innerHTML=tag(ls.front_left,'OK','KRAWEDZ');\n"
    "      document.getElementById('ls-fr').innerHTML=tag(ls.front_right,'OK','KRAWEDZ');\n"
    "      document.getElementById('ls-bl').innerHTML=tag(ls.back_left,'OK','KRAWEDZ');\n"
    "      document.getElementById('ls-br').innerHTML=tag(ls.back_right,'OK','KRAWEDZ');\n"
    "    }\n"
    "    if(d.motors){\n"
    "      document.getElementById('m-l').textContent=d.motors.left;\n"
    "      document.getElementById('m-r').textContent=d.motors.right;\n"
    "    }\n"
    "  }).catch(function(){\n"
    "    document.getElementById('status-bar').innerHTML='<span class=\"err\">&#128997; Brak polaczenia</span>';\n"
    "  });\n"
    "}\n"
    "\n"
    "function beep(){fetch('/api/buzzer',{method:'POST'});}\n"
    "function buzz(f,d){fetch('/api/buzzer',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({freq:f,duration_ms:d})});}\n"
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
    "setInterval(poll,1000);\n"
    "pollLogs();\n"
    "setInterval(pollLogs,1000);\n"
    "</script>\n"
    "</body>\n"
    "</html>\n";

// ── Pomocnicza: odczytaj body żądania POST ───────────────────
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

// ── GET / ────────────────────────────────────────────────────
static esp_err_t handle_root(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, DASHBOARD_HTML, sizeof(DASHBOARD_HTML) - 1);
    return ESP_OK;
}

// ── GET /api/sensors ─────────────────────────────────────────
static esp_err_t handle_sensors(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();

    // LIDAR
    lidar_packet_t lpkt = lidar_get_last_packet();
    cJSON *lidar = cJSON_CreateObject();
    cJSON_AddNumberToObject(lidar, "min_distance_mm", lidar_get_min_distance_mm());
    cJSON_AddNumberToObject(lidar, "angle_hundredths",
        lpkt.valid && lpkt.count > 0 ? lpkt.points[0].angle_hundredths : 0);
    cJSON_AddNumberToObject(lidar, "speed_rpm", lpkt.speed_rpm);
    cJSON_AddBoolToObject(lidar, "valid", lpkt.valid);
    cJSON_AddItemToObject(root, "lidar", lidar);

    // Pirometr
    pyrometer_data_t pd = pyrometer_get_last();
    cJSON *pyro = cJSON_CreateObject();
    cJSON_AddNumberToObject(pyro, "object_temp",  (double)pd.object_temp);
    cJSON_AddNumberToObject(pyro, "ambient_temp", (double)pd.ambient_temp);
    cJSON_AddBoolToObject(pyro, "finish_detected", pd.finish_detected);
    cJSON_AddBoolToObject(pyro, "initialized",     pd.initialized);
    cJSON_AddItemToObject(root, "pyrometer", pyro);

    // IMU
    imu_data_t id = imu_get_last();
    cJSON *imu = cJSON_CreateObject();
    cJSON_AddNumberToObject(imu, "accel_x", (double)id.accel_x);
    cJSON_AddNumberToObject(imu, "accel_y", (double)id.accel_y);
    cJSON_AddNumberToObject(imu, "accel_z", (double)id.accel_z);
    cJSON_AddNumberToObject(imu, "gyro_x",  (double)id.gyro_x);
    cJSON_AddNumberToObject(imu, "gyro_y",  (double)id.gyro_y);
    cJSON_AddNumberToObject(imu, "gyro_z",  (double)id.gyro_z);
    cJSON_AddNumberToObject(imu, "temp",    (double)id.temp);
    cJSON_AddBoolToObject(imu, "initialized", id.initialized);
    cJSON_AddItemToObject(root, "imu", imu);

    // INA219 (prad/napiecie)
    ina219_data_t in = ina219_get_last();
    cJSON *ina = cJSON_CreateObject();
    cJSON_AddNumberToObject(ina, "bus_voltage_v",    (double)in.bus_voltage_v);
    cJSON_AddNumberToObject(ina, "shunt_voltage_mv", (double)in.shunt_voltage_mv);
    cJSON_AddNumberToObject(ina, "current_ma",       (double)in.current_ma);
    cJSON_AddNumberToObject(ina, "power_mw",         (double)in.power_mw);
    cJSON_AddNumberToObject(ina, "address",          in.address);
    cJSON_AddBoolToObject(ina, "initialized",        in.initialized);
    cJSON_AddItemToObject(root, "ina219", ina);

    // SHT40 (temperatura/wilgotnosc)
    sht40_data_t sh = sht40_get_last();
    cJSON *sht = cJSON_CreateObject();
    cJSON_AddNumberToObject(sht, "temperature_c", (double)sh.temperature_c);
    cJSON_AddNumberToObject(sht, "humidity_pct",  (double)sh.humidity_pct);
    cJSON_AddNumberToObject(sht, "address",       sh.address);
    cJSON_AddBoolToObject(sht, "initialized",     sh.initialized);
    cJSON_AddItemToObject(root, "sht40", sht);

    // Odometria
    odometry_data_t od = odometry_get();
    cJSON *odo = cJSON_CreateObject();
    cJSON_AddNumberToObject(odo, "pulses_left",    od.pulses_left);
    cJSON_AddNumberToObject(odo, "pulses_right",   od.pulses_right);
    cJSON_AddNumberToObject(odo, "dist_left_mm",   (double)od.dist_left_mm);
    cJSON_AddNumberToObject(odo, "dist_right_mm",  (double)od.dist_right_mm);
    cJSON_AddNumberToObject(odo, "dist_total_mm",  (double)od.dist_total_mm);
    cJSON_AddBoolToObject(odo, "finish_detected",  od.finish_detected);
    cJSON_AddItemToObject(root, "odometry", odo);

    // Czujniki linii
    line_sensor_data_t ls = line_sensor_read();
    cJSON *line = cJSON_CreateObject();
    cJSON_AddBoolToObject(line, "front_left",  ls.front_left);
    cJSON_AddBoolToObject(line, "front_right", ls.front_right);
    cJSON_AddBoolToObject(line, "back_left",   ls.back_left);
    cJSON_AddBoolToObject(line, "back_right",  ls.back_right);
    cJSON_AddItemToObject(root, "line_sensors", line);

    // Silniki
    cJSON *motors = cJSON_CreateObject();
    cJSON_AddNumberToObject(motors, "left",  motor_get_left_speed());
    cJSON_AddNumberToObject(motors, "right", motor_get_right_speed());
    cJSON_AddItemToObject(root, "motors", motors);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, json_str);
    free(json_str);
    return ESP_OK;
}

// ── POST /api/motor  {"left": -100..100, "right": -100..100} ─
static esp_err_t handle_motor(httpd_req_t *req) {
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

// ── POST /api/motor/stop ─────────────────────────────────────
static esp_err_t handle_motor_stop(httpd_req_t *req) {
    motor_stop();
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// ── POST /api/led  {"red":0/1,"yellow":0/1,"green":0/1} ───
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

// ── POST /api/odometry/reset ─────────────────────────────────
static esp_err_t handle_odo_reset(httpd_req_t *req) {
    odometry_reset();
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// ── GET /api/logs  (monitor szeregowy przez WWW) ─────────────
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

// ── POST /api/logs/clear ─────────────────────────────────────
static esp_err_t handle_logs_clear(httpd_req_t *req) {
    web_monitor_clear();
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// ── POST /api/buzzer  {opcjonalnie "freq","duration_ms"} ─────
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

// ── GET /api/lidar/scan?since=N ──────────────────────────────
// Strumień punktów lidaru do budowy mapy przeszkód.
// Zwraca kompaktowy JSON: {"seq":S,"rpm":R,"n":K,"pts":[a0,d0,a1,d1,...]}
//   seq – bieżący licznik sekwencyjny (podaj jako ?since= przy kolejnym żądaniu,
//         dzięki czemu pobierasz tylko NOWE punkty, bez duplikatów)
//   pts – spłaszczona tablica par (kąt[setne stopnia], odległość[mm])
static esp_err_t handle_lidar_scan(httpd_req_t *req) {
    uint32_t since = 0;
    char q[48];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char v[24];
        if (httpd_query_key_value(q, "since", v, sizeof(v)) == ESP_OK)
            since = (uint32_t)strtoul(v, NULL, 10);
    }

    // Okno odczytu: najnowsze do 512 punktów na żądanie (~jeden obrót LD06).
    static lidar_scan_point_t pts[512];
    uint32_t seq = 0;
    uint16_t n = lidar_copy_scan(since, pts, 512, &seq);

    // Ręczny, kompaktowy JSON (cJSON dla setek liczb jest pamięcio/czasożerny).
    size_t cap = 64 + (size_t)n * 14;   // "65535,65535," ≈ 12 znaków/punkt
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

esp_err_t http_server_start(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port        = 80;
    cfg.max_uri_handlers   = 16;
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
        { .uri="/api/logs",           .method=HTTP_GET,  .handler=handle_logs       },
        { .uri="/api/logs/clear",     .method=HTTP_POST, .handler=handle_logs_clear },
        { .uri="/api/buzzer",         .method=HTTP_POST, .handler=handle_buzzer     },
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
