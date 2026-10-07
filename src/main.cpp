#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "config.h"

// CYD タッチコントローラ(XPT2046)のIRQピン。タッチ中はLOW
#define T_IRQ 36

TFT_eSPI tft;

struct Plane { char call[9]; float lat, lon, trk, dist; int alt, gs; };
const int MAXP = 40;
Plane planes[MAXP];
int nPlanes = 0;

const int RANGES[] = {10, 25, 50, 100};
int rangeIdx = 1;

const int CX = 120, CY = 120, R = 112;  // レーダー円 (landscape 320x240)

float distKm(float lat, float lon, float &dx, float &dy) {
  dy = (lat - HOME_LAT) * 111.12f;
  dx = (lon - HOME_LON) * 111.12f * cosf(HOME_LAT * DEG_TO_RAD);
  return sqrtf(dx * dx + dy * dy);
}

// 同一JSON形式(ac配列)の無料API。adsb.lol を優先し、制限(429/403)時は60秒休ませて adsb.fi を使う
static String buildUrl(int host, int radius) {
  String lat = String(HOME_LAT, 4), lon = String(HOME_LON, 4);
  if (host == 0) return "https://api.adsb.lol/v2/point/" + lat + "/" + lon + "/" + radius;
  return "https://opendata.adsb.fi/api/v2/lat/" + lat + "/lon/" + lon + "/dist/" + radius;
}
const int NHOSTS = 2;
const uint32_t COOLDOWN_MS = 60000;
uint32_t blockedUntil[NHOSTS] = {0, 0};

bool fetchPlanes() {
  int radius = (int)ceilf(RANGES[rangeIdx] / 1.852f);  // APIはnm指定なのでkm→nm変換
  WiFiClientSecure client;
  client.setInsecure();  // 簡易化のため証明書検証なし
  HTTPClient http;
  http.useHTTP10(true);  // chunked回避（ストリームパース用）
  http.setTimeout(8000);
  http.setUserAgent("Mozilla/5.0 (ESP32; CYD-Flightradar)");
  http.addHeader("Accept", "application/json");

  bool ok = false;
  for (int host = 0; host < NHOSTS && !ok; host++) {
    if ((int32_t)(blockedUntil[host] - millis()) > 0) continue;  // 休止中
    String url = buildUrl(host, radius);
    if (!http.begin(client, url)) continue;
    int code = http.GET();
    Serial.printf("[%d] %s -> HTTP %d\n", host, url.c_str(), code);
    if (code == 200) { ok = true; break; }
    if (code == 429 || code == 403) blockedUntil[host] = millis() + COOLDOWN_MS;
    http.end();
  }
  if (!ok) return false;

  JsonDocument filter;
  JsonObject f = filter["ac"][0].to<JsonObject>();
  f["flight"] = true; f["lat"] = true; f["lon"] = true;
  f["alt_baro"] = true; f["gs"] = true; f["track"] = true;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
  http.end();
  if (err) { Serial.println(err.c_str()); return false; }

  nPlanes = 0;
  for (JsonObject ac : doc["ac"].as<JsonArray>()) {
    if (nPlanes >= MAXP) break;
    if (ac["lat"].isNull() || ac["lon"].isNull()) continue;
    Plane &p = planes[nPlanes];
    p.lat = ac["lat"]; p.lon = ac["lon"];
    p.trk = ac["track"] | 0.0f;
    p.gs = (int)((ac["gs"] | 0.0f) + 0.5f);  // gsは小数で来るのでfloatで受ける
    p.alt = ac["alt_baro"].is<float>() ? (int)ac["alt_baro"].as<float>() : 0;  // "ground" は 0
    const char *fl = ac["flight"] | "";
    strlcpy(p.call, fl, sizeof(p.call));
    for (int i = strlen(p.call) - 1; i >= 0 && p.call[i] == ' '; i--) p.call[i] = 0;
    if (!p.call[0]) strcpy(p.call, "------");
    float dx, dy; p.dist = distKm(p.lat, p.lon, dx, dy);
    nPlanes++;
  }
  // 近い順
  for (int i = 1; i < nPlanes; i++) {
    Plane t = planes[i]; int j = i - 1;
    while (j >= 0 && planes[j].dist > t.dist) { planes[j + 1] = planes[j]; j--; }
    planes[j + 1] = t;
  }
  return true;
}

void draw(const char *status) {
  int range = RANGES[rangeIdx];
  tft.fillScreen(TFT_BLACK);
  // レーダー
  tft.drawCircle(CX, CY, R, TFT_DARKGREEN);
  tft.drawCircle(CX, CY, R * 2 / 3, TFT_DARKGREEN);
  tft.drawCircle(CX, CY, R / 3, TFT_DARKGREEN);
  tft.drawFastHLine(CX - R, CY, 2 * R, TFT_DARKGREEN);
  tft.drawFastVLine(CX, CY - R, 2 * R, TFT_DARKGREEN);
  tft.fillCircle(CX, CY, 3, TFT_CYAN);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("N", CX + 3, CY - R, 1);
  tft.drawString(String(range) + "km", 2, 2, 1);

  for (int i = 0; i < nPlanes; i++) {
    float dx, dy; distKm(planes[i].lat, planes[i].lon, dx, dy);
    int px = CX + (int)(dx / range * R), py = CY - (int)(dy / range * R);
    if ((px - CX) * (px - CX) + (py - CY) * (py - CY) > R * R) continue;
    float a = planes[i].trk * DEG_TO_RAD;
    uint16_t col = planes[i].alt < 10000 ? TFT_ORANGE : TFT_YELLOW;
    tft.drawLine(px, py, px + (int)(9 * sinf(a)), py - (int)(9 * cosf(a)), col);
    tft.fillCircle(px, py, 2, col);
    if (i < 3) { tft.setTextColor(col, TFT_BLACK); tft.drawString(planes[i].call, px + 4, py + 3, 1); }
  }

  // 右パネル: 近い順リスト
  tft.drawFastVLine(238, 0, 240, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(nPlanes) + " ac", 244, 2, 2);
  for (int i = 0; i < nPlanes && i < 6; i++) {
    int y = 22 + i * 34;
    char buf[24];
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString(planes[i].call, 244, y, 1);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%d m", (int)(planes[i].alt * 0.3048f + 0.5f));      // ft -> m
    tft.drawString(buf, 244, y + 10, 1);
    snprintf(buf, sizeof(buf), "%d km/h", (int)(planes[i].gs * 1.852f + 0.5f));      // kt -> km/h
    tft.drawString(buf, 244, y + 20, 1);
  }
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString(status, 244, 230, 1);
}

void setup() {
  Serial.begin(115200);
  pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
  tft.init(); tft.setRotation(1); tft.fillScreen(TFT_BLACK);
  pinMode(T_IRQ, INPUT);

  tft.drawString("WiFi connecting...", 10, 10, 2);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(300);
}

void loop() {
  static uint32_t last = 0;
  static bool touched = false;
  bool refresh = false;

  if (digitalRead(T_IRQ) == LOW) {
    if (!touched) { rangeIdx = (rangeIdx + 1) % 4; refresh = true; }
    touched = true;
  } else touched = false;

  if (refresh || millis() - last > FETCH_MS || last == 0) {
    last = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
    bool ok = fetchPlanes();
    draw(ok ? "OK" : "ERR");
  }
  delay(50);
}
