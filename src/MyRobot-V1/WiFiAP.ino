/**
 * @file WiFiAP.ino
 * @brief SoftAP mode + simple HTTP control for ESP32/ESP8266.
 *
 * Creates a Wi-Fi access point and serves a tiny web UI plus
 * command endpoint that maps to the existing command interface.
 */

#include <Arduino.h>

// Forward declaration from CommandInterface.ino
void commandInterfaceHandleCommand(const String& cmd);

// ---------- Wi-Fi configuration ----------
static const char* kApSsid = "Robot-AP";
static const char* kApPassword = "robot1234"; // 8+ chars required

#if defined(ESP32)
  #include <WiFi.h>
  #include <WebServer.h>
  static WebServer g_server(80);
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  static ESP8266WebServer g_server(80);
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNO_R4_WIFI)
  #include <WiFiS3.h>
  #include <WiFiClient.h>
  #include <WiFiServer.h>
  static WiFiServer g_server(80);
#else
  // Non-WiFi boards (e.g., Uno/Nano) compile stubs.
#endif

#if defined(ESP32) || defined(ESP8266)
static void handleRoot() {
  const char* page =
    "<!doctype html>"
    "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'/>"
    "<title>Robot Control</title>"
    "<style>body{font-family:Arial;margin:24px}button{font-size:18px;margin:6px;padding:10px 14px}</style>"
    "</head><body>"
    "<h2>Robot Control</h2>"
    "<p>Tap a button to send a command.</p>"
    "<div>"
    "<button onclick=\"send('FWD 500 200')\">Forward</button>"
    "<button onclick=\"send('REV 500 200')\">Reverse</button>"
    "<button onclick=\"send('LEFT 400 150 50')\">Left</button>"
    "<button onclick=\"send('RIGHT 400 150 50')\">Right</button>"
    "<button onclick=\"send('SPINL 400 180')\">Spin Left</button>"
    "<button onclick=\"send('SPINR 400 180')\">Spin Right</button>"
    "<button onclick=\"send('STOP')\">Stop</button>"
    "</div>"
    "<script>function send(c){fetch('/cmd?c='+encodeURIComponent(c)).then(r=>r.text()).then(t=>console.log(t));}</script>"
    "</body></html>";
  g_server.send(200, "text/html", page);
}

static void handleCmd() {
  String cmd = g_server.arg("c");
  cmd.trim();
  if (cmd.length() == 0) {
    g_server.send(400, "text/plain", "Missing c parameter");
    return;
  }
  commandInterfaceHandleCommand(cmd);
  g_server.send(200, "text/plain", "OK");
}

static void handleNotFound() {
  g_server.send(404, "text/plain", "Not found");
}
#endif

#if defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNO_R4_WIFI)
static const char* kHttpPage =
  "<!doctype html>"
  "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'/>"
  "<title>Robot Control</title>"
  "<style>body{font-family:Arial;margin:24px}button{font-size:18px;margin:6px;padding:10px 14px}</style>"
  "</head><body>"
  "<h2>Robot Control</h2>"
  "<p>Tap a button to send a command.</p>"
  "<div>"
  "<button onclick=\"send('FWD 500 200')\">Forward</button>"
  "<button onclick=\"send('REV 500 200')\">Reverse</button>"
  "<button onclick=\"send('LEFT 400 150 50')\">Left</button>"
  "<button onclick=\"send('RIGHT 400 150 50')\">Right</button>"
  "<button onclick=\"send('SPINL 400 180')\">Spin Left</button>"
  "<button onclick=\"send('SPINR 400 180')\">Spin Right</button>"
  "<button onclick=\"send('STOP')\">Stop</button>"
  "</div>"
  "<script>function send(c){fetch('/cmd?c='+encodeURIComponent(c)).then(r=>r.text()).then(t=>console.log(t));}</script>"
  "</body></html>";

// Forward declarations (avoid Arduino auto-prototype issues)
static String urlDecode(const String& s);
static void sendHttp(WiFiClient& client, const char* status, const char* type, const String& body);
static void handleHttpClient(WiFiClient& client);

static String urlDecode(const String& s) {
  String out;
  out.reserve(s.length());
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    if (c == '+') { out += ' '; }
    else if (c == '%' && i + 2 < (int)s.length()) {
      char h1 = s[i + 1];
      char h2 = s[i + 2];
      int v1 = isDigit(h1) ? h1 - '0' : (toupper(h1) - 'A' + 10);
      int v2 = isDigit(h2) ? h2 - '0' : (toupper(h2) - 'A' + 10);
      out += (char)((v1 << 4) | v2);
      i += 2;
    } else {
      out += c;
    }
  }
  return out;
}

static void sendHttp(WiFiClient& client, const char* status, const char* type, const String& body) {
  client.print("HTTP/1.1 "); client.println(status);
  client.println("Connection: close");
  client.print("Content-Type: "); client.println(type);
  client.print("Content-Length: "); client.println(body.length());
  client.println();
  client.print(body);
}

static void handleHttpClient(WiFiClient& client) {
  String requestLine = client.readStringUntil('\n');
  requestLine.trim();
  // Consume headers
  while (client.available()) {
    String h = client.readStringUntil('\n');
    if (h == "\r" || h.length() == 0) break;
  }

  // Expect: GET /path?query HTTP/1.1
  if (!requestLine.startsWith("GET ")) {
    sendHttp(client, "405 Method Not Allowed", "text/plain", "Only GET supported");
    return;
  }

  int pathStart = 4;
  int pathEnd = requestLine.indexOf(' ', pathStart);
  if (pathEnd < 0) { sendHttp(client, "400 Bad Request", "text/plain", "Bad request"); return; }

  String fullPath = requestLine.substring(pathStart, pathEnd);
  String path = fullPath;
  String query;
  int qpos = fullPath.indexOf('?');
  if (qpos >= 0) {
    path = fullPath.substring(0, qpos);
    query = fullPath.substring(qpos + 1);
  }

  if (path == "/") {
    sendHttp(client, "200 OK", "text/html", String(kHttpPage));
    return;
  }

  if (path == "/cmd") {
    // parse c=...
    String cmd;
    if (query.startsWith("c=")) {
      cmd = urlDecode(query.substring(2));
    }
    cmd.trim();
    if (cmd.length() == 0) {
      sendHttp(client, "400 Bad Request", "text/plain", "Missing c parameter");
      return;
    }
    commandInterfaceHandleCommand(cmd);
    sendHttp(client, "200 OK", "text/plain", "OK");
    return;
  }

  sendHttp(client, "404 Not Found", "text/plain", "Not found");
}
#endif

void initializeWifiAp() {
#if defined(ESP32) || defined(ESP8266)
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(kApSsid, kApPassword);
  if (ok) {
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP started. SSID: ");
    Serial.print(kApSsid);
    Serial.print(" IP: ");
    Serial.println(ip);
  } else {
    Serial.println("AP start failed");
  }

  g_server.on("/", handleRoot);
  g_server.on("/cmd", handleCmd);
  g_server.onNotFound(handleNotFound);
  g_server.begin();
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNO_R4_WIFI)
  int status = WiFi.beginAP(kApSsid, kApPassword);
  if (status == WL_AP_LISTENING || status == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    Serial.print("AP started. SSID: ");
    Serial.print(kApSsid);
    Serial.print(" IP: ");
    Serial.println(ip);
  } else {
    Serial.println("AP start failed");
  }
  g_server.begin();
#else
  Serial.println("WiFi AP not available on this board");
#endif
}

void wifiTick() {
#if defined(ESP32) || defined(ESP8266)
  g_server.handleClient();
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNO_R4_WIFI)
  WiFiClient client = g_server.available();
  if (client) {
    handleHttpClient(client);
    delay(1);
    client.stop();
  }
#endif
}
