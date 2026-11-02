/*
  ============================================================
  ESP32-WROOM-32 SYSTEM MONITOR  -  v4
  ============================================================
  Requires Arduino-ESP32 core 3.x

  FEATURES
  ------------------------------------------------------------
  SHT40
  DHT11
  W5500 Ethernet
  Battery monitor
  Power source detection
  4 status LEDs
  System health
  AJAX live webpage (auto updates every 2 seconds)

  NEW IN v4
  ------------------------------------------------------------
  - Ethernet IP settings on the Network page:
      DHCP      -> address from the router (fallback 192.168.50.2
                   after 15 s when no DHCP server answers)
      Static IP -> fixed address, subnet, optional gateway / DNS
    Applied within a second, no restart needed.
  - Status page: everything about WiFi, Ethernet, setup AP,
    uptime, memory, restart reason, live every 3 seconds.
  - Serial Log page: shows everything the sketch prints to the
    Serial Monitor (last 8 KB, with time since boot), live.
    Protected: you must enter your own password again
    (unlocked for 10 minutes without activity).

  NEW IN v3: WIFI + ETHERNET + SETUP AP COORDINATION
  ------------------------------------------------------------
  - Setting "WiFi while Ethernet is connected" (WiFi page):
      Keep WiFi on  -> WiFi and Ethernet work at the same time
      Turn WiFi off -> once Ethernet has been connected for
                       5 seconds, WiFi (and the setup AP) are
                       switched off. If Ethernet disconnects or
                       is disabled, WiFi turns on again and
                       reconnects automatically.
  - Ethernet reconnects automatically:
      * if the W5500 fails to start          -> retry every 30 s
      * if enabled but not connected for 60 s -> W5500 restarted
      * cable plugged in later                -> connects by itself
  - Ethernet DHCP fallback for a direct laptop cable:
      if no DHCP server answers within 15 s, the device uses
      192.168.50.2 / 255.255.255.0. Give the laptop 192.168.50.1.
      When the cable is removed, DHCP is used again.
  - Setup AP (ESP32-SETUP) runs whenever the device has NO
    network connection (neither WiFi nor Ethernet) for 5 s,
    while WiFi and Ethernet keep trying to connect.
    As soon as WiFi or Ethernet is connected, the setup AP is
    stopped (immediately if nobody uses it, otherwise after
    60 s at the latest). It starts again if all networks are
    lost.

  WIFI MANAGEMENT
  ------------------------------------------------------------
  - Up to 5 saved WiFi networks (add / edit / delete)
  - Eye icon to show / hide the password when adding or editing
  - Priority for every network (1 = highest ... 5 = lowest)
  - Default network
  - Connection modes:
      Default WiFi only
      Priority mode
  - "Connect now" button for any saved network
  - DHCP / Static IP (applied without restart)
  - Restart button

  PRIORITY MODE RULES
  ------------------------------------------------------------
  1. Connection order: the DEFAULT network first, then the other
     networks from highest to lowest priority.
  2. If a network is unavailable, the next one is tried.
  3. While connected to a lower network, the device scans every
     30 seconds. When a higher network is in range:
       - it is the DEFAULT network -> switch automatically,
                                      no notification
       - any other network         -> notification on the web
                                      page: "Switch now" / "Not now"
  4. If the current network is lost, the device tries the
     network it was using before (previous network) first,
     then the normal priority order.
  5. If a switch fails, the device returns to the network it
     was on and does not retry that switch for 10 minutes.
  6. "Not now" hides the notification for that network for
     10 minutes.

  SETUP AP
  ------------------------------------------------------------
  AP SSID      : ESP32-SETUP
  AP PASSWORD  : ESP32setup
  AP IP        : 192.168.4.1

  LOGIN / USERS
  ------------------------------------------------------------
  - Every page requires a login (up to 8 user accounts)
  - Passwords stored as salted SHA-256 hashes
  - Sessions expire after 30 minutes without activity
  - 5 wrong passwords -> login blocked for 60 seconds
  - First boot: create the first user with the master password
    (allowed from any connection, including the setup AP)
  - Adding / editing / deleting users requires:
      1. ESP32 connected to the DEFAULT or a PRIORITY 1 network
      2. Browser reaching the ESP32 through that WiFi connection
         (not through the setup AP or Ethernet)
      3. The permanent master password: TH-sensor-web
         (unlocks user management for 10 minutes)

  ============================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <WiFi.h>
#include <WebServer.h>
#include <ETH.h>
#include <Preferences.h>

#include <Adafruit_SHT4x.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#include <functional>
#include <esp_random.h>
#include "mbedtls/sha256.h"
#include <esp_system.h>
#include <esp_mac.h>


// ============================================================
// SERIAL LOG (everything printed also goes to the web page)
// ============================================================
// Use SerialLog.print / println instead of Serial.print.
// Output goes to the Serial Monitor unchanged, and a copy with
// a time stamp is kept in an 8 KB ring buffer for the web page.

#define LOG_BUFFER_SIZE 8192

char logBuffer[LOG_BUFFER_SIZE];
uint32_t logTotal = 0;          // bytes ever stored (position counter)
bool logAtLineStart = true;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

class SerialLogger : public Print
{
public:
  using Print::write;

  size_t write(uint8_t c) override
  {
    return write(&c, 1);
  }

  size_t write(const uint8_t *buffer, size_t size) override
  {
    Serial.write(buffer, size);

    // Time since boot, e.g. "[01:02:03.456] "
    unsigned long now = millis();
    char stamp[24];
    snprintf(stamp, sizeof(stamp), "[%02lu:%02lu:%02lu.%03lu] ",
             now / 3600000UL, (now / 60000UL) % 60UL, (now / 1000UL) % 60UL, now % 1000UL);

    // Can be called from the network event task too
    portENTER_CRITICAL(&logMux);

    for (size_t i = 0; i < size; i++)
    {
      char c = (char)buffer[i];

      if (c == '\r' || c == 0) continue;

      if (logAtLineStart)
      {
        for (const char *p = stamp; *p; p++)
        {
          logBuffer[logTotal % LOG_BUFFER_SIZE] = *p;
          logTotal++;
        }
        logAtLineStart = false;
      }

      logBuffer[logTotal % LOG_BUFFER_SIZE] = c;
      logTotal++;

      if (c == '\n') logAtLineStart = true;
    }

    portEXIT_CRITICAL(&logMux);

    return size;
  }
};

SerialLogger SerialLog;



// ============================================================
// HARDWARE
// ============================================================

// SHT40
#define SDA_PIN 21
#define SCL_PIN 22

// W5500
#define W5500_SCK  18
#define W5500_MISO 19
#define W5500_MOSI 23
#define W5500_CS   5
#define W5500_INT  -1
#define W5500_RST  -1

// DHT11
#define DHT_PIN 27
#define DHT_TYPE DHT11

// Battery
#define BATTERY_VOLTAGE_PIN   34
#define BATTERY_DIVIDER_RATIO 5.00f
#define BATTERY_EMPTY_VOLTAGE 3.00f
#define BATTERY_FULL_VOLTAGE  4.20f

// Power source
#define POWER_SOURCE_PIN 32

// Ethernet enable
#define ETHERNET_STATUS_PIN 33

// LEDs
#define WIFI_LED_PIN          13
#define ETHERNET_LED_PIN      14
#define SYSTEM_HEALTH_LED_PIN 25
#define BATTERY_LED_PIN       26


// ============================================================
// SETUP ACCESS POINT
// ============================================================

const char* SETUP_AP_SSID = "ESP32-SETUP";
const char* SETUP_AP_PASSWORD = "ESP32setup";

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

#define SETUP_AP_START_DELAY   5000UL   // no network for 5 s -> start AP
#define SETUP_AP_CLOSE_GRACE  60000UL   // max time AP stays on after connecting
#define SETUP_AP_RETRY_TIME   10000UL   // if starting the AP failed

unsigned long noNetworkSince = 0;
unsigned long networkUpSince = 0;
bool hadNetwork = false;


// ============================================================
// ETHERNET FALLBACK ADDRESS (direct cable to a laptop)
// ============================================================

IPAddress ETH_FALLBACK_IP(192, 168, 50, 2);
IPAddress ETH_FALLBACK_GATEWAY(192, 168, 50, 1);
IPAddress ETH_FALLBACK_SUBNET(255, 255, 255, 0);

#define ETHERNET_DHCP_TIMEOUT 15000UL   // no DHCP answer -> fallback IP
#define ETHERNET_REINIT_TIME  60000UL   // not connected -> restart W5500


// ============================================================
// ETHERNET IP SETTINGS (saved in namespace "eth")
// ============================================================

bool ethStaticEnabled = false;

String ethStaticIP      = "";
String ethStaticGateway = "";   // optional
String ethStaticSubnet  = "";
String ethStaticDNS     = "";   // optional

// Applied from the loop shortly after the web reply was sent
bool ethConfigPending = false;
unsigned long ethConfigPendingAt = 0;


// ============================================================
// STATISTICS (Status page)
// ============================================================

unsigned long wifiConnectedAt = 0;
uint32_t wifiConnectCount = 0;
uint32_t wifiLostCount = 0;
uint32_t ethConnectCount = 0;
uint32_t ethRestartCount = 0;
esp_reset_reason_t bootReason = ESP_RST_UNKNOWN;


// ============================================================
// WIFI STORAGE
// ============================================================

#define MAX_WIFI_NETWORKS 5

struct WiFiNetwork
{
  String name;
  String password;
  int priority;          // 1 = highest, MAX_WIFI_NETWORKS = lowest
};

WiFiNetwork savedWiFi[MAX_WIFI_NETWORKS];

int savedWiFiCount = 0;
int defaultWiFiIndex = -1;


// ============================================================
// WIFI CONNECTION MODE
// ============================================================

// true  = only the default WiFi
// false = priority mode (default first, then by priority)
bool defaultOnlyMode = false;

// true = turn WiFi off while Ethernet is connected
bool wifiOffWhenEthernet = false;

#define ETHERNET_STABLE_TIME 5000UL   // Ethernet must be up this long
#define WIFI_OFF_HOLD_TIME   3000UL   // let the web reply finish first

unsigned long wifiOffHoldStart = 0;


// ============================================================
// STATIC IP SETTINGS
// ============================================================

bool staticIPEnabled = false;

String staticIPAddress = "";
String staticGateway   = "";
String staticSubnet    = "";
String staticDNS       = "";


// ============================================================
// PREFERENCES / OBJECTS
// ============================================================

Preferences preferences;

Adafruit_SHT4x sht4;

DHT dht(DHT_PIN, DHT_TYPE);

WebServer server(80);


// ============================================================
// SENSOR VALUES
// ============================================================

float sht40Temperature = 0.0f;
float sht40Humidity = 0.0f;

float dhtTemperature = 0.0f;
float dhtHumidity = 0.0f;

float batteryVoltage = 0.0f;
float batteryPercent = 0.0f;

bool sht40OK = false;
bool dht11OK = false;


// ============================================================
// NETWORK STATUS
// ============================================================

bool wifiConnected = false;
bool setupAPActive = false;
unsigned long lastSetupAPAttempt = 0;

bool ethernetEnabled = false;
bool ethernetStarted = false;
bool ethernetFirstAttempt = true;
bool ethernetStartFailed = false;

// Link state, written from the network event task
volatile bool ethernetLinkUp = false;

// Cable connected AND an IP address (DHCP or fallback)
bool ethernetConnected = false;
unsigned long ethernetConnectedSince = 0;

bool ethFallbackActive = false;
bool ethLinkWasUp = false;
unsigned long ethLinkUpSince = 0;
unsigned long ethNotConnectedSince = 0;

unsigned long lastEthernetStartAttempt = 0;

#define ETHERNET_RETRY_INTERVAL 30000UL

// Ethernet enable pin debounce
bool ethRawState = false;
unsigned long ethRawChangedAt = 0;

#define ETHERNET_DEBOUNCE_TIME 100UL


// ============================================================
// POWER
// ============================================================

bool mainPowerDetected = false;
bool batteryPower = false;


// ============================================================
// WIFI STATE MACHINE (non-blocking)
// ============================================================

enum WiFiState
{
  WIFI_STATE_IDLE,
  WIFI_STATE_CONNECTING,
  WIFI_STATE_CONNECTED,
  WIFI_STATE_WAIT_RETRY,
  WIFI_STATE_SUSPENDED      // WiFi off because Ethernet is connected
};

WiFiState wifiState = WIFI_STATE_IDLE;

#define WIFI_ATTEMPT_TIMEOUT  15000UL   // per network
#define WIFI_RETRY_INTERVAL   30000UL   // after all networks failed
#define WIFI_SCAN_INTERVAL    30000UL   // look for higher priority networks
#define WIFI_SCAN_TIMEOUT     15000UL
#define SWITCH_DISMISS_TIME  600000UL   // 10 minutes
#define MIN_SWITCH_RSSI         -80     // ignore weaker networks for switching

// Current connection cycle
int attemptOrder[MAX_WIFI_NETWORKS];
int attemptCount = 0;
int attemptPos = 0;
int attemptIndex = -1;
bool attemptInvalid = false;
unsigned long attemptStart = 0;

// Set by the WiFi event handler
volatile uint8_t attemptDisconnectReason = 0;

unsigned long wifiRetryStart = 0;

// Network history
int currentWiFiIndex = -1;
int lastConnectedIndex = -1;
int previousWiFiIndex = -1;

String wifiFailureReason = "";

// Scanning
bool scanRunning = false;
unsigned long scanStart = 0;
unsigned long lastScanTime = 0;

// Switch notification
int pendingSwitchIndex = -1;      // higher network waiting for user decision
int dismissedSwitchIndex = -1;    // "Not now" or failed switch
unsigned long dismissedAt = 0;
int switchTargetIndex = -1;       // network we are switching to

// Requests from the web pages (handled in wifiTask)
int requestedSwitchIndex = -1;
bool reconnectRequested = false;


// ============================================================
// RESTART
// ============================================================

bool restartPending = false;
unsigned long restartRequestTime = 0;


// ============================================================
// LOGIN / USER ACCOUNTS
// ============================================================

// Permanent master password for adding / editing users.
// It can only be changed by editing this line and re-uploading.
const char* MASTER_PASSWORD = "TH-sensor-web";

#define MAX_USERS               8
#define MAX_SESSIONS            6
#define SESSION_TIMEOUT         1800000UL   // 30 min without activity
#define USER_ADMIN_UNLOCK_TIME   600000UL   // 10 min after master password
#define LOGIN_MAX_FAILURES      5
#define LOGIN_LOCKOUT_TIME        60000UL
#define PASSWORD_HASH_ROUNDS    1000
#define LOG_UNLOCK_TIME          600000UL   // 10 min after password

struct UserAccount
{
  String username;
  String salt;
  String hash;
};

UserAccount users[MAX_USERS];
int userCount = 0;

struct Session
{
  bool active = false;
  String token;
  String username;
  unsigned long lastActive = 0;
  bool adminUnlocked = false;
  unsigned long adminUnlockedAt = 0;
  bool logUnlocked = false;
  unsigned long logUnlockedAt = 0;
};

Session sessions[MAX_SESSIONS];

// Session of the request being handled (-1 = none)
int currentSession = -1;

int loginFailures = 0;
unsigned long loginLockStart = 0;

typedef void (*PageHandler)();


// ============================================================
// SENSOR TIMING
// ============================================================

#define SENSOR_READ_INTERVAL 2000UL

unsigned long lastSensorRead = 0;


// ============================================================
// LED TIMING
// ============================================================

#define WIFI_BLINK_INTERVAL      500UL
#define ETHERNET_BLINK_INTERVAL  500UL
#define HEALTH_BLINK_INTERVAL    500UL

unsigned long lastWiFiBlink = 0;
unsigned long lastEthernetBlink = 0;
unsigned long lastHealthBlink = 0;


// ============================================================
// BATTERY LED
// ============================================================

#define BATTERY_LEVEL_1_PERIOD  5000UL
#define BATTERY_LEVEL_2_PERIOD 10000UL
#define BATTERY_LEVEL_3_PERIOD 15000UL

#define BATTERY_LEVEL_1_ON_TIME 1000UL
#define BATTERY_LEVEL_2_ON_TIME 2000UL
#define BATTERY_LEVEL_3_ON_TIME 3000UL

unsigned long batteryBlinkStart = 0;


// ============================================================
// LED STATES
// ============================================================

bool wifiLedState = false;
bool ethernetLedState = false;
bool healthLedState = false;
bool batteryLedState = false;


// ============================================================
// ERROR TRACKING / HEALTH
// ============================================================

int dhtErrorCount = 0;
int sht40ErrorCount = 0;

bool systemHealthy = true;

String systemHealthMessage = "System operating normally";

bool sht40InitialStatusPrinted = false;
bool dht11InitialStatusPrinted = false;


// ============================================================
// FORWARD DECLARATIONS
// ============================================================

void beginAttempt(int index);
void startConnectionCycle(int first, int second);


// ============================================================
// HELPERS
// ============================================================

String htmlEscape(String value)
{
  value.replace("&", "&amp;");
  value.replace("<", "&lt;");
  value.replace(">", "&gt;");
  value.replace("\"", "&quot;");
  value.replace("'", "&#39;");
  return value;
}

String jsonEscape(const String &value)
{
  String out;
  out.reserve(value.length() + 8);

  for (unsigned int i = 0; i < value.length(); i++)
  {
    char c = value[i];

    if (c == '"' || c == '\\')
    {
      out += '\\';
      out += c;
    }
    else if ((uint8_t)c < 0x20)
    {
      char buf[8];
      snprintf(buf, sizeof(buf), "\\u%04x", (uint8_t)c);
      out += buf;
    }
    else
    {
      out += c;
    }
  }

  return out;
}

bool isValidIndex(int index)
{
  return index >= 0 && index < savedWiFiCount;
}

int clampPriority(int p)
{
  if (p < 1) return 1;
  if (p > MAX_WIFI_NETWORKS) return MAX_WIFI_NETWORKS;
  return p;
}

void redirectTo(const String &location)
{
  server.sendHeader("Location", location, true);
  server.send(302, "text/plain", "");
}

// Only allow known pages as redirect targets
String safeBack()
{
  return server.arg("back") == "/wifi" ? String("/wifi") : String("/");
}

// Returns -1 when the "i" argument is missing
int argIndex()
{
  if (!server.hasArg("i")) return -1;
  return server.arg("i").toInt();
}

bool ipIsSet(const IPAddress &ip)
{
  return (uint32_t)ip != 0;
}

bool wifiSuspended()
{
  return wifiState == WIFI_STATE_SUSPENDED;
}


// ============================================================
// LOAD WIFI SETTINGS
// ============================================================

void loadWiFiSettings()
{
  preferences.begin("wifi", true);

  savedWiFiCount = preferences.getInt("count", 0);

  if (savedWiFiCount < 0 || savedWiFiCount > MAX_WIFI_NETWORKS)
  {
    savedWiFiCount = 0;
  }

  defaultWiFiIndex    = preferences.getInt("default", -1);
  defaultOnlyMode     = preferences.getBool("defaultOnly", false);
  wifiOffWhenEthernet = preferences.getBool("offOnEth", false);

  staticIPEnabled = preferences.getBool("staticIP", false);
  staticIPAddress = preferences.getString("ip", "");
  staticGateway   = preferences.getString("gateway", "");
  staticSubnet    = preferences.getString("subnet", "");
  staticDNS       = preferences.getString("dns", "");

  for (int i = 0; i < savedWiFiCount; i++)
  {
    String keySSID = "ssid" + String(i);
    String keyPASS = "pass" + String(i);
    String keyPRIO = "prio" + String(i);

    savedWiFi[i].name     = preferences.getString(keySSID.c_str(), "");
    savedWiFi[i].password = preferences.getString(keyPASS.c_str(), "");
    savedWiFi[i].priority = clampPriority(preferences.getInt(keyPRIO.c_str(), i + 1));
  }

  preferences.end();

  if (!isValidIndex(defaultWiFiIndex))
  {
    defaultWiFiIndex = savedWiFiCount > 0 ? 0 : -1;
  }
}


// ============================================================
// SAVE WIFI SETTINGS
// ============================================================

void saveWiFiSettings()
{
  preferences.begin("wifi", false);

  preferences.clear();

  preferences.putInt("count", savedWiFiCount);
  preferences.putInt("default", defaultWiFiIndex);
  preferences.putBool("defaultOnly", defaultOnlyMode);
  preferences.putBool("offOnEth", wifiOffWhenEthernet);

  preferences.putBool("staticIP", staticIPEnabled);
  preferences.putString("ip", staticIPAddress);
  preferences.putString("gateway", staticGateway);
  preferences.putString("subnet", staticSubnet);
  preferences.putString("dns", staticDNS);

  for (int i = 0; i < savedWiFiCount; i++)
  {
    String keySSID = "ssid" + String(i);
    String keyPASS = "pass" + String(i);
    String keyPRIO = "prio" + String(i);

    preferences.putString(keySSID.c_str(), savedWiFi[i].name);
    preferences.putString(keyPASS.c_str(), savedWiFi[i].password);
    preferences.putInt(keyPRIO.c_str(), savedWiFi[i].priority);
  }

  preferences.end();
}


// ============================================================
// IP HELPERS
// ============================================================

bool parseIP(String text, IPAddress &result)
{
  text.trim();

  if (text.length() == 0)
  {
    return false;
  }

  return result.fromString(text);
}

bool applyStaticIP()
{
  if (!staticIPEnabled)
  {
    return true;
  }

  IPAddress ip, gateway, subnet, dns;

  if (!parseIP(staticIPAddress, ip))      return false;
  if (!parseIP(staticGateway, gateway))   return false;
  if (!parseIP(staticSubnet, subnet))     return false;
  if (!parseIP(staticDNS, dns))           return false;

  return WiFi.config(ip, gateway, subnet, dns);
}

// Empty text = 0.0.0.0 (not used)
bool parseOptionalIP(String text, IPAddress &result)
{
  text.trim();

  if (text.length() == 0)
  {
    result = IPAddress((uint32_t)0);
    return true;
  }

  return result.fromString(text);
}


// ============================================================
// ETHERNET IP SETTINGS: LOAD / SAVE / APPLY
// ============================================================

void loadEthernetSettings()
{
  preferences.begin("eth", true);

  ethStaticEnabled = preferences.getBool("static", false);
  ethStaticIP      = preferences.getString("ip", "");
  ethStaticGateway = preferences.getString("gateway", "");
  ethStaticSubnet  = preferences.getString("subnet", "");
  ethStaticDNS     = preferences.getString("dns", "");

  preferences.end();
}

void saveEthernetSettings()
{
  preferences.begin("eth", false);

  preferences.putBool("static", ethStaticEnabled);
  preferences.putString("ip", ethStaticIP);
  preferences.putString("gateway", ethStaticGateway);
  preferences.putString("subnet", ethStaticSubnet);
  preferences.putString("dns", ethStaticDNS);

  preferences.end();
}

// Applies DHCP or the static address to the running W5500
bool applyEthernetIPConfig()
{
  if (!ethernetStarted) return true;

  ethFallbackActive = false;
  ethLinkUpSince = millis();   // DHCP timeout starts again

  if (!ethStaticEnabled)
  {
    return ETH.config();       // all zero = DHCP
  }

  IPAddress ip, gateway, subnet, dns;

  if (!parseIP(ethStaticIP, ip))                return false;
  if (!parseIP(ethStaticSubnet, subnet))        return false;
  if (!parseOptionalIP(ethStaticGateway, gateway)) return false;
  if (!parseOptionalIP(ethStaticDNS, dns))      return false;

  return ETH.config(ip, gateway, subnet, dns);
}

String ethernetModeText()
{
  if (ethStaticEnabled)  return "Static IP";
  if (ethFallbackActive) return "Fallback address (no DHCP server)";
  return "DHCP";
}


// ============================================================
// READ SHT40
// ============================================================

void readSHT40()
{
  sensors_event_t humidityEvent;
  sensors_event_t temperatureEvent;

  if (sht4.getEvent(&humidityEvent, &temperatureEvent))
  {
    sht40Temperature = temperatureEvent.temperature;
    sht40Humidity = humidityEvent.relative_humidity;
    sht40OK = true;
    sht40ErrorCount = 0;
  }
  else
  {
    sht40OK = false;
    sht40ErrorCount++;

    if (!sht40InitialStatusPrinted)
    {
      SerialLog.println("ERROR: SHT40 reading failed");
      sht40InitialStatusPrinted = true;
    }
  }
}


// ============================================================
// READ DHT11
// ============================================================

void readDHT11()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))
  {
    dht11OK = false;
    dhtErrorCount++;

    if (!dht11InitialStatusPrinted)
    {
      SerialLog.println("ERROR: DHT11 reading failed");
      dht11InitialStatusPrinted = true;
    }

    return;
  }

  dhtHumidity = h;
  dhtTemperature = t;
  dht11OK = true;
  dhtErrorCount = 0;
}


// ============================================================
// BATTERY (uses factory ADC calibration)
// ============================================================

void readBatteryVoltage()
{
  const int samples = 20;
  uint32_t totalMilliVolts = 0;

  for (int i = 0; i < samples; i++)
  {
    totalMilliVolts += analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
    delayMicroseconds(500);
  }

  float pinVoltage = ((float)totalMilliVolts / (float)samples) / 1000.0f;

  batteryVoltage = pinVoltage * BATTERY_DIVIDER_RATIO;

  batteryPercent =
    ((batteryVoltage - BATTERY_EMPTY_VOLTAGE) /
     (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE)) * 100.0f;

  if (batteryPercent < 0)   batteryPercent = 0;
  if (batteryPercent > 100) batteryPercent = 100;
}


// ============================================================
// POWER SOURCE
// ============================================================

void readPowerSource()
{
  mainPowerDetected = digitalRead(POWER_SOURCE_PIN) == HIGH;
  batteryPower = !mainPowerDetected;
}


// ============================================================
// ETHERNET ENABLE PIN (debounced)
// ============================================================

void readEthernetEnable()
{
  bool raw = digitalRead(ETHERNET_STATUS_PIN) == HIGH;

  if (raw != ethRawState)
  {
    ethRawState = raw;
    ethRawChangedAt = millis();
  }

  if (millis() - ethRawChangedAt >= ETHERNET_DEBOUNCE_TIME)
  {
    ethernetEnabled = ethRawState;
  }
}


// ============================================================
// WIFI DISCONNECT REASON
// ============================================================

String disconnectReasonText(uint8_t reason)
{
  switch (reason)
  {
    case WIFI_REASON_NO_AP_FOUND:
      return "WiFi network not found";

    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
      return "Wrong password or authentication failed";

    case WIFI_REASON_AUTH_EXPIRE:
      return "Authentication timed out (weak signal?)";

    case WIFI_REASON_ASSOC_FAIL:
      return "Router refused the connection";

    case WIFI_REASON_BEACON_TIMEOUT:
      return "WiFi signal lost";

    default:
      return "Disconnected (reason " + String(reason) + ")";
  }
}

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
  {
    uint8_t reason = info.wifi_sta_disconnected.reason;

    // Ignore the disconnect we cause ourselves before each attempt
    if (reason != WIFI_REASON_ASSOC_LEAVE)
    {
      attemptDisconnectReason = reason;
    }
  }
}


// ============================================================
// PRIORITY ORDER
// ============================================================
// order[0] = default network
// then the other networks sorted by priority (1 first).
// Equal priorities keep their saved order.

int buildPriorityOrder(int order[])
{
  int n = 0;
  int start = 0;

  if (isValidIndex(defaultWiFiIndex))
  {
    order[n++] = defaultWiFiIndex;
    start = 1;
  }

  for (int i = 0; i < savedWiFiCount; i++)
  {
    if (i == defaultWiFiIndex) continue;

    int j = n;

    while (j > start && savedWiFi[order[j - 1]].priority > savedWiFi[i].priority)
    {
      order[j] = order[j - 1];
      j--;
    }

    order[j] = i;
    n++;
  }

  return n;
}

// 0 = highest (default). Unknown network = lowest.
int networkRank(int index)
{
  int order[MAX_WIFI_NETWORKS];
  int n = buildPriorityOrder(order);

  for (int k = 0; k < n; k++)
  {
    if (order[k] == index) return k;
  }

  return MAX_WIFI_NETWORKS;
}


// ============================================================
// SETUP AP
// ============================================================

void startSetupAP()
{
  lastSetupAPAttempt = millis();

  SerialLog.println();
  SerialLog.println("No network connected - starting ESP32 setup AP...");

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);

  if (WiFi.softAP(SETUP_AP_SSID, SETUP_AP_PASSWORD))
  {
    setupAPActive = true;

    SerialLog.println("================================");
    SerialLog.println("SETUP AP ACTIVE");
    SerialLog.print("SSID: ");     SerialLog.println(SETUP_AP_SSID);
    SerialLog.print("Password: "); SerialLog.println(SETUP_AP_PASSWORD);
    SerialLog.print("IP: ");       SerialLog.println(WiFi.softAPIP());
    SerialLog.println("================================");

    // The mode change can interrupt a running WiFi attempt: restart it
    if (wifiState == WIFI_STATE_CONNECTING && isValidIndex(attemptIndex))
    {
      beginAttempt(attemptIndex);
    }
  }
  else
  {
    SerialLog.println("ERROR: Setup AP failed - retrying in 10 s");
  }
}

void stopSetupAP(const char *why)
{
  if (!setupAPActive) return;

  WiFi.softAPdisconnect(true);
  setupAPActive = false;

  SerialLog.print("Setup AP stopped (");
  SerialLog.print(why);
  SerialLog.println(")");

  if (wifiState == WIFI_STATE_CONNECTING && isValidIndex(attemptIndex))
  {
    beginAttempt(attemptIndex);
  }
}

// Called every loop.
// No network at all for 5 s  -> start the setup AP.
// WiFi or Ethernet connected -> stop it (at once if unused,
//                               otherwise after 60 s at the latest).
void updateSetupAP()
{
  bool online = wifiConnected || ethernetConnected;

  if (online)
  {
    if (!hadNetwork)
    {
      hadNetwork = true;
      networkUpSince = millis();
    }

    if (setupAPActive)
    {
      if (WiFi.softAPgetStationNum() == 0)
      {
        stopSetupAP("network connected");
      }
      else if (millis() - networkUpSince >= SETUP_AP_CLOSE_GRACE)
      {
        stopSetupAP("network connected, 60 s grace time over");
      }
    }

    return;
  }

  if (hadNetwork)
  {
    hadNetwork = false;
    noNetworkSince = millis();
  }

  if (setupAPActive || wifiSuspended()) return;

  if (millis() - noNetworkSince < SETUP_AP_START_DELAY) return;

  if (lastSetupAPAttempt != 0 && millis() - lastSetupAPAttempt < SETUP_AP_RETRY_TIME) return;

  startSetupAP();
}


// ============================================================
// CONNECTION ATTEMPTS
// ============================================================

void beginAttempt(int index)
{
  attemptIndex = index;
  attemptStart = millis();
  attemptInvalid = false;

  SerialLog.println();
  SerialLog.print("Trying WiFi #");
  SerialLog.print(networkRank(index) + 1);
  SerialLog.print(" in order: ");
  SerialLog.println(savedWiFi[index].name);

  WiFi.disconnect(false);
  WiFi.mode(setupAPActive ? WIFI_AP_STA : WIFI_STA);
  delay(100);

  attemptDisconnectReason = 0;

  if (staticIPEnabled)
  {
    if (!applyStaticIP())
    {
      attemptInvalid = true;
      wifiFailureReason = "Invalid static IP configuration";
      wifiState = WIFI_STATE_CONNECTING;
      return;
    }
  }
  else
  {
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  }

  WiFi.begin(
    savedWiFi[index].name.c_str(),
    savedWiFi[index].password.c_str()
  );

  wifiState = WIFI_STATE_CONNECTING;
}

void addAttempt(int index)
{
  if (!isValidIndex(index)) return;

  for (int k = 0; k < attemptCount; k++)
  {
    if (attemptOrder[k] == index) return;
  }

  if (attemptCount < MAX_WIFI_NETWORKS)
  {
    attemptOrder[attemptCount++] = index;
  }
}

void onAllAttemptsFailed()
{
  WiFi.disconnect(false);

  wifiConnected = false;
  currentWiFiIndex = -1;
  attemptIndex = -1;
  switchTargetIndex = -1;

  if (savedWiFiCount == 0)
  {
    wifiFailureReason = "No saved WiFi networks";
    SerialLog.println("No saved WiFi networks.");
  }
  else
  {
    SerialLog.println();
    SerialLog.println("No saved WiFi network connected. Retrying in 30 s.");
  }

  // The setup AP is started by updateSetupAP() when no network
  // (WiFi or Ethernet) is connected.

  wifiState = WIFI_STATE_WAIT_RETRY;
  wifiRetryStart = millis();
}

void onAttemptFailed()
{
  if (!attemptInvalid)
  {
    uint8_t reason = attemptDisconnectReason;
    wifiFailureReason = reason ? disconnectReasonText(reason) : String("Connection timed out");
  }

  if (isValidIndex(attemptIndex))
  {
    SerialLog.print("WiFi failed: ");
    SerialLog.print(savedWiFi[attemptIndex].name);
    SerialLog.print(" - ");
    SerialLog.println(wifiFailureReason);
  }

  attemptPos++;

  if (attemptPos < attemptCount)
  {
    beginAttempt(attemptOrder[attemptPos]);
  }
  else
  {
    onAllAttemptsFailed();
  }
}

// first / second are tried before the normal priority order (-1 = none)
void startConnectionCycle(int first, int second)
{
  wifiConnected = false;
  currentWiFiIndex = -1;
  pendingSwitchIndex = -1;
  switchTargetIndex = -1;
  attemptCount = 0;
  attemptPos = 0;

  if (defaultOnlyMode)
  {
    addAttempt(defaultWiFiIndex);
  }
  else
  {
    addAttempt(first);
    addAttempt(second);

    int order[MAX_WIFI_NETWORKS];
    int n = buildPriorityOrder(order);

    for (int k = 0; k < n; k++)
    {
      addAttempt(order[k]);
    }
  }

  if (attemptCount == 0)
  {
    onAllAttemptsFailed();
    return;
  }

  beginAttempt(attemptOrder[0]);
}

void onConnected()
{
  int index = attemptIndex;

  // A requested switch ended on a different network = switch failed
  if (switchTargetIndex >= 0 && switchTargetIndex != index)
  {
    SerialLog.print("Switch to ");
    SerialLog.print(savedWiFi[switchTargetIndex].name);
    SerialLog.println(" failed - not retrying it for 10 minutes");

    dismissedSwitchIndex = switchTargetIndex;
    dismissedAt = millis();
  }

  switchTargetIndex = -1;

  if (lastConnectedIndex >= 0 && lastConnectedIndex != index)
  {
    previousWiFiIndex = lastConnectedIndex;
  }

  lastConnectedIndex = index;
  currentWiFiIndex = index;

  wifiConnected = true;
  wifiConnectedAt = millis();
  wifiConnectCount++;
  wifiFailureReason = "";
  attemptDisconnectReason = 0;
  pendingSwitchIndex = -1;
  lastScanTime = millis();

  wifiState = WIFI_STATE_CONNECTED;

  SerialLog.println();
  SerialLog.println("==============================");
  SerialLog.println("WiFi CONNECTED");
  SerialLog.print("Network : "); SerialLog.println(savedWiFi[index].name);
  SerialLog.print("Priority: "); SerialLog.println(savedWiFi[index].priority);
  SerialLog.print("IP      : "); SerialLog.println(WiFi.localIP());

  if (isValidIndex(previousWiFiIndex))
  {
    SerialLog.print("Previous: "); SerialLog.println(savedWiFi[previousWiFiIndex].name);
  }

  SerialLog.println("==============================");
}


// ============================================================
// WIFI OFF / ON WHILE ETHERNET IS CONNECTED
// ============================================================

bool wifiShouldBeOff()
{
  return wifiOffWhenEthernet &&
         ethernetConnected &&
         millis() - ethernetConnectedSince >= ETHERNET_STABLE_TIME &&
         millis() - wifiOffHoldStart >= WIFI_OFF_HOLD_TIME;
}

void suspendWiFi()
{
  SerialLog.println();
  SerialLog.println("Ethernet connected - turning WiFi OFF");

  if (scanRunning)
  {
    WiFi.scanDelete();
    scanRunning = false;
  }

  if (setupAPActive)
  {
    WiFi.softAPdisconnect(true);
    setupAPActive = false;
    SerialLog.println("Setup AP stopped (WiFi turned off)");
  }

  WiFi.disconnect(false);
  WiFi.mode(WIFI_OFF);

  wifiConnected = false;
  currentWiFiIndex = -1;
  attemptIndex = -1;
  pendingSwitchIndex = -1;
  switchTargetIndex = -1;
  requestedSwitchIndex = -1;
  reconnectRequested = false;

  wifiFailureReason = "WiFi turned off while Ethernet is connected";
  wifiState = WIFI_STATE_SUSPENDED;
}

void resumeWiFi()
{
  SerialLog.println();

  if (!ethernetConnected)
  {
    SerialLog.println("Ethernet not connected - turning WiFi ON");
  }
  else
  {
    SerialLog.println("Setting changed - turning WiFi ON");
  }

  WiFi.mode(WIFI_STA);
  wifiFailureReason = "";

  startConnectionCycle(-1, -1);
}


// ============================================================
// HIGHER PRIORITY NETWORK DETECTION
// ============================================================

void evaluateScanResults(int found)
{
  int currentRank = networkRank(currentWiFiIndex);
  int best = -1;
  int bestRank = currentRank;

  for (int s = 0; s < found; s++)
  {
    if (WiFi.RSSI(s) < MIN_SWITCH_RSSI) continue;

    String ssid = WiFi.SSID(s);

    for (int i = 0; i < savedWiFiCount; i++)
    {
      if (savedWiFi[i].name != ssid) continue;

      int r = networkRank(i);

      if (r < bestRank)
      {
        best = i;
        bestRank = r;
      }
    }
  }

  if (best < 0)
  {
    if (pendingSwitchIndex >= 0)
    {
      SerialLog.println("Higher priority network no longer in range");
    }

    pendingSwitchIndex = -1;
    return;
  }

  // Dismissed by the user or a recent failed switch
  if (best == dismissedSwitchIndex && millis() - dismissedAt < SWITCH_DISMISS_TIME)
  {
    pendingSwitchIndex = -1;
    return;
  }

  // Default network: switch automatically, no notification
  if (best == defaultWiFiIndex)
  {
    SerialLog.print("Default network in range: ");
    SerialLog.print(savedWiFi[best].name);
    SerialLog.println(" - switching automatically");

    requestedSwitchIndex = best;
    return;
  }

  // Other higher priority network: ask the user
  if (pendingSwitchIndex != best)
  {
    SerialLog.print("Higher priority network in range: ");
    SerialLog.print(savedWiFi[best].name);
    SerialLog.println(" - waiting for user (web page)");
  }

  pendingSwitchIndex = best;
}


// ============================================================
// CONNECTED STATE
// ============================================================

void handleConnectedState()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    int lost = currentWiFiIndex;
    uint8_t reason = attemptDisconnectReason;

    wifiFailureReason = reason ? disconnectReasonText(reason) : String("WiFi connection lost");
    wifiLostCount++;

    SerialLog.println();
    SerialLog.print("WiFi lost: ");
    if (isValidIndex(lost)) SerialLog.print(savedWiFi[lost].name);
    SerialLog.print(" - ");
    SerialLog.println(wifiFailureReason);

    // Go back to the previous network first
    int first = (previousWiFiIndex != lost) ? previousWiFiIndex : -1;

    startConnectionCycle(first, -1);
    return;
  }

  if (defaultOnlyMode || networkRank(currentWiFiIndex) == 0)
  {
    pendingSwitchIndex = -1;
    return;
  }

  // On a lower network: scan for higher ones
  if (millis() - lastScanTime >= WIFI_SCAN_INTERVAL)
  {
    int result = WiFi.scanNetworks(true);

    if (result == WIFI_SCAN_RUNNING)
    {
      scanRunning = true;
      scanStart = millis();
    }
    else
    {
      lastScanTime = millis();
    }
  }
}


// ============================================================
// WIFI TASK (called every loop, never blocks for long)
// ============================================================

void wifiTask()
{
  // ----------------------------------------------------------
  // WiFi off while Ethernet is connected (optional setting)
  // ----------------------------------------------------------

  if (wifiSuspended())
  {
    // Nothing to do on WiFi while it is off
    requestedSwitchIndex = -1;
    reconnectRequested = false;

    if (!wifiOffWhenEthernet || !ethernetConnected)
    {
      resumeWiFi();
    }

    return;
  }

  if (wifiShouldBeOff())
  {
    suspendWiFi();
    return;
  }

  // ----------------------------------------------------------
  // Finish a running scan first
  // ----------------------------------------------------------

  if (scanRunning)
  {
    int found = WiFi.scanComplete();

    if (found == WIFI_SCAN_RUNNING && millis() - scanStart < WIFI_SCAN_TIMEOUT)
    {
      return;
    }

    scanRunning = false;
    lastScanTime = millis();

    if (found > 0 && wifiState == WIFI_STATE_CONNECTED && WiFi.status() == WL_CONNECTED)
    {
      evaluateScanResults(found);
    }

    WiFi.scanDelete();
  }

  // ----------------------------------------------------------
  // Switch request (user or automatic default switch)
  // ----------------------------------------------------------

  if (requestedSwitchIndex >= 0)
  {
    int target = requestedSwitchIndex;
    int fallback = currentWiFiIndex;

    requestedSwitchIndex = -1;
    pendingSwitchIndex = -1;

    if (isValidIndex(target) && !(wifiConnected && target == currentWiFiIndex))
    {
      SerialLog.println();
      SerialLog.print("Switching WiFi to: ");
      SerialLog.println(savedWiFi[target].name);

      // Target first, then back to the current network if it fails
      startConnectionCycle(target, fallback);
      switchTargetIndex = target;
    }

    return;
  }

  // ----------------------------------------------------------
  // Settings changed
  // ----------------------------------------------------------

  if (reconnectRequested)
  {
    reconnectRequested = false;

    SerialLog.println();
    SerialLog.println("Reconnecting with updated WiFi settings...");

    startConnectionCycle(-1, -1);
    return;
  }

  // ----------------------------------------------------------
  // State machine
  // ----------------------------------------------------------

  switch (wifiState)
  {
    case WIFI_STATE_IDLE:
      startConnectionCycle(-1, -1);
      break;

    case WIFI_STATE_CONNECTING:
      if (attemptInvalid)
      {
        onAttemptFailed();
      }
      else if (WiFi.status() == WL_CONNECTED)
      {
        onConnected();
      }
      else if (attemptDisconnectReason != 0 && millis() - attemptStart > 1000UL)
      {
        // Router rejected us or network not found: fail fast
        onAttemptFailed();
      }
      else if (millis() - attemptStart >= WIFI_ATTEMPT_TIMEOUT)
      {
        onAttemptFailed();
      }
      break;

    case WIFI_STATE_CONNECTED:
      handleConnectedState();
      break;

    case WIFI_STATE_WAIT_RETRY:
      if (savedWiFiCount == 0) break;

      if (millis() - wifiRetryStart < WIFI_RETRY_INTERVAL) break;

      // Don't disturb someone using the setup AP
      if (setupAPActive && WiFi.softAPgetStationNum() > 0)
      {
        wifiRetryStart = millis();
        break;
      }

      startConnectionCycle(-1, -1);
      break;

    case WIFI_STATE_SUSPENDED:
      break;
  }
}

// Call after any WiFi setting change
void settingsChanged(bool currentNetworkChanged)
{
  pendingSwitchIndex = -1;

  // While WiFi is off the new settings are used when it turns on
  if (wifiSuspended()) return;

  bool needReconnect =
    !wifiConnected ||
    currentNetworkChanged ||
    (defaultOnlyMode && currentWiFiIndex != defaultWiFiIndex);

  if (needReconnect)
  {
    reconnectRequested = true;
  }
  else
  {
    // Check for higher priority networks right away
    lastScanTime = millis() - WIFI_SCAN_INTERVAL;
  }
}


// ============================================================
// ETHERNET
// ============================================================

// Runs in the network event task: only set flags and print
void onNetworkEvent(arduino_event_id_t event)
{
  switch (event)
  {
    case ARDUINO_EVENT_ETH_CONNECTED:
      ethernetLinkUp = true;
      SerialLog.println("Ethernet : CABLE CONNECTED (link up)");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      SerialLog.print("Ethernet : DHCP address ");
      SerialLog.println(ETH.localIP());
      break;

    case ARDUINO_EVENT_ETH_LOST_IP:
      SerialLog.println("Ethernet : IP LOST");
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      ethernetLinkUp = false;
      SerialLog.println("Ethernet : CABLE DISCONNECTED (link down)");
      break;

    case ARDUINO_EVENT_ETH_STOP:
      ethernetLinkUp = false;
      SerialLog.println("Ethernet : STOPPED");
      break;

    default:
      break;
  }
}

void resetEthernetState()
{
  ethernetLinkUp = false;
  ethernetConnected = false;
  ethFallbackActive = false;
  ethLinkWasUp = false;
  ethNotConnectedSince = millis();
}

void startEthernet()
{
  lastEthernetStartAttempt = millis();
  resetEthernetState();

  SerialLog.println("Starting W5500 Ethernet...");

  SPI.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);

  bool result = ETH.begin(
    ETH_PHY_W5500,
    1,
    W5500_CS,
    W5500_INT,
    W5500_RST,
    SPI
  );

  if (!result)
  {
    ethernetStarted = false;
    ethernetStartFailed = true;
    SerialLog.println("ERROR: W5500 Ethernet start failed - retrying in 30 s");
  }
  else
  {
    ethernetStarted = true;
    ethernetStartFailed = false;
    SerialLog.println("W5500 Ethernet initialization started");

    if (ethStaticEnabled)
    {
      if (applyEthernetIPConfig())
      {
        SerialLog.print("Ethernet : static IP ");
        SerialLog.println(ethStaticIP);
      }
      else
      {
        SerialLog.println("ERROR: Ethernet static IP could not be applied - check the Network page");
      }
    }
  }
}

void stopEthernet(const char *why)
{
  if (ethernetStarted)
  {
    ETH.end();
  }

  ethernetStarted = false;
  ethernetStartFailed = false;
  resetEthernetState();

  SerialLog.print("Ethernet : ");
  SerialLog.println(why);
}

void handleEthernetEnable()
{
  readEthernetEnable();

  if (ethernetEnabled)
  {
    bool retryDue = millis() - lastEthernetStartAttempt >= ETHERNET_RETRY_INTERVAL;

    if (!ethernetStarted && (ethernetFirstAttempt || retryDue))
    {
      ethernetFirstAttempt = false;
      startEthernet();
    }
  }
  else if (ethernetStarted || ethernetStartFailed)
  {
    stopEthernet("DISABLED (enable pin LOW)");
    ethernetFirstAttempt = true;
  }
}

// Called every loop: works out whether Ethernet is usable,
// applies the fallback address and restarts a stuck W5500.
void updateEthernetStatus()
{
  // New IP settings saved on the web page
  if (ethConfigPending && millis() - ethConfigPendingAt >= 1000UL)
  {
    ethConfigPending = false;

    if (ethernetStarted)
    {
      bool ok = applyEthernetIPConfig();

      SerialLog.print("Ethernet : IP settings applied - ");
      SerialLog.println(ethStaticEnabled ? "static " + ethStaticIP : String("DHCP"));

      if (!ok)
      {
        SerialLog.println("ERROR: Ethernet IP settings could not be applied");
      }
    }
  }

  bool linkUp = ethernetStarted && ethernetLinkUp;

  // Cable plugged in
  if (linkUp && !ethLinkWasUp)
  {
    ethLinkUpSince = millis();
  }

  // Cable removed: use DHCP again for the next network
  if (!linkUp && ethLinkWasUp && ethFallbackActive && ethernetStarted && !ethStaticEnabled)
  {
    ETH.config();
    ethFallbackActive = false;
    SerialLog.println("Ethernet : fallback address removed, DHCP enabled again");
  }

  ethLinkWasUp = linkUp;

  bool hasIP = linkUp && ipIsSet(ETH.localIP());

  // No DHCP answer (e.g. cable straight to a laptop): fallback address
  if (linkUp && !hasIP && !ethFallbackActive && !ethStaticEnabled &&
      millis() - ethLinkUpSince >= ETHERNET_DHCP_TIMEOUT)
  {
    if (ETH.config(ETH_FALLBACK_IP, ETH_FALLBACK_GATEWAY, ETH_FALLBACK_SUBNET, ETH_FALLBACK_GATEWAY))
    {
      ethFallbackActive = true;
      SerialLog.println("Ethernet : no DHCP server - using fallback address 192.168.50.2");
      hasIP = ipIsSet(ETH.localIP());
    }
    else
    {
      SerialLog.println("Ethernet : could not set fallback address - retrying in 15 s");
      ethLinkUpSince = millis();
    }
  }

  bool connected = hasIP;

  if (connected && !ethernetConnected)
  {
    ethernetConnectedSince = millis();
    ethernetStartFailed = false;
    ethConnectCount++;

    SerialLog.println();
    SerialLog.println("==============================");
    SerialLog.println("Ethernet CONNECTED");
    SerialLog.print("IP      : "); SerialLog.println(ETH.localIP());
    SerialLog.print("Mode    : "); SerialLog.println(ethernetModeText());
    SerialLog.println("==============================");
  }
  else if (!connected && ethernetConnected)
  {
    SerialLog.println("Ethernet : NOT CONNECTED anymore");
  }

  ethernetConnected = connected;

  // Enabled but not connected for a long time: restart the W5500
  if (ethernetEnabled && ethernetStarted && !connected)
  {
    if (millis() - ethNotConnectedSince >= ETHERNET_REINIT_TIME)
    {
      stopEthernet("not connected for 60 s - restarting W5500");
      ethRestartCount++;
      startEthernet();
    }
  }
  else
  {
    ethNotConnectedSince = millis();
  }
}


// ============================================================
// SYSTEM HEALTH
// ============================================================

void updateSystemHealth()
{
  systemHealthy = true;
  systemHealthMessage = "System operating normally";

  // SHT40 affects health
  if (!sht40OK)
  {
    systemHealthy = false;
    systemHealthMessage = "ERROR: SHT40 sensor not responding";
  }

  // DHT11 does not affect health

  // Only check battery voltage when running on battery
  if (batteryPower && batteryVoltage < 2.50f)
  {
    systemHealthy = false;
    systemHealthMessage = "ERROR: Battery voltage extremely low";
  }

  if (ethernetEnabled && ethernetStartFailed)
  {
    systemHealthy = false;
    systemHealthMessage = "ERROR: W5500 Ethernet initialization failed";
  }

  if (systemHealthMessage == "System operating normally")
  {
    if (!wifiConnected && !ethernetConnected)
    {
      systemHealthMessage = setupAPActive
        ? "WARNING: No network connected - setup AP ESP32-SETUP is active"
        : "WARNING: No network connected - trying WiFi and Ethernet";
    }
    else if (ethernetEnabled && ethernetStarted && !ethernetConnected)
    {
      systemHealthMessage = "WARNING: Ethernet enabled but network not connected";
    }
    else if (!wifiConnected && !wifiSuspended())
    {
      if (wifiState == WIFI_STATE_CONNECTING && isValidIndex(attemptIndex))
      {
        systemHealthMessage = "WARNING: Connecting to WiFi " + savedWiFi[attemptIndex].name + " ...";
      }
      else
      {
        systemHealthMessage = "WARNING: WiFi not connected - " + wifiFailureReason;
      }
    }
  }
}


// ============================================================
// LEDS
// ============================================================

void updateWiFiLED()
{
  // WiFi deliberately off: LED off
  if (wifiSuspended())
  {
    digitalWrite(WIFI_LED_PIN, LOW);
    wifiLedState = false;
    return;
  }

  if (wifiConnected)
  {
    digitalWrite(WIFI_LED_PIN, HIGH);
    wifiLedState = true;
    return;
  }

  if (millis() - lastWiFiBlink >= WIFI_BLINK_INTERVAL)
  {
    lastWiFiBlink = millis();
    wifiLedState = !wifiLedState;
    digitalWrite(WIFI_LED_PIN, wifiLedState);
  }
}

void updateEthernetLED()
{
  if (!ethernetEnabled)
  {
    digitalWrite(ETHERNET_LED_PIN, LOW);
    ethernetLedState = false;
    return;
  }

  if (!ethernetConnected)
  {
    if (millis() - lastEthernetBlink >= ETHERNET_BLINK_INTERVAL)
    {
      lastEthernetBlink = millis();
      ethernetLedState = !ethernetLedState;
      digitalWrite(ETHERNET_LED_PIN, ethernetLedState);
    }
    return;
  }

  digitalWrite(ETHERNET_LED_PIN, HIGH);
  ethernetLedState = true;
}

void updateSystemHealthLED()
{
  if (systemHealthy)
  {
    digitalWrite(SYSTEM_HEALTH_LED_PIN, HIGH);
    healthLedState = true;
    return;
  }

  if (millis() - lastHealthBlink >= HEALTH_BLINK_INTERVAL)
  {
    lastHealthBlink = millis();
    healthLedState = !healthLedState;
    digitalWrite(SYSTEM_HEALTH_LED_PIN, healthLedState);
  }
}

void updateBatteryLED()
{
  if (!batteryPower || batteryPercent >= 80.0f)
  {
    digitalWrite(BATTERY_LED_PIN, LOW);
    batteryLedState = false;
    return;
  }

  unsigned long period;
  unsigned long onTime;

  if (batteryPercent < 20.0f)
  {
    period = BATTERY_LEVEL_3_PERIOD;
    onTime = BATTERY_LEVEL_3_ON_TIME;
  }
  else if (batteryPercent < 40.0f)
  {
    period = BATTERY_LEVEL_2_PERIOD;
    onTime = BATTERY_LEVEL_2_ON_TIME;
  }
  else
  {
    period = BATTERY_LEVEL_1_PERIOD;
    onTime = BATTERY_LEVEL_1_ON_TIME;
  }

  unsigned long elapsed = millis() - batteryBlinkStart;

  if (elapsed >= period)
  {
    batteryBlinkStart = millis();
    elapsed = 0;
  }

  batteryLedState = elapsed < onTime;
  digitalWrite(BATTERY_LED_PIN, batteryLedState ? HIGH : LOW);
}

void updateLEDs()
{
  updateWiFiLED();
  updateEthernetLED();
  updateSystemHealthLED();
  updateBatteryLED();
}


// ============================================================
// WEB: ICONS AND SCRIPTS
// ============================================================

const char EYE_ICON_SHOW[] =
  "<svg class='i1' width='22' height='22' viewBox='0 0 24 24' fill='none' "
  "stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'>"
  "<path d='M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z'/>"
  "<circle cx='12' cy='12' r='3'/></svg>";

const char EYE_ICON_HIDE[] =
  "<svg class='i2' width='22' height='22' viewBox='0 0 24 24' fill='none' "
  "stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'>"
  "<path d='M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 "
  "5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19"
  "m-6.72-1.07a3 3 0 1 1-4.24-4.24'/>"
  "<line x1='1' y1='1' x2='23' y2='23'/></svg>";

// Show / hide password
const char PASSWORD_SCRIPT[] = R"JS(
<script>
function tp(id,b){
  var e=document.getElementById(id);
  var show=e.type==='password';
  e.type=show?'text':'password';
  b.classList.toggle('vis',show);
  b.title=show?'Hide password':'Show password';
  b.setAttribute('aria-label',b.title);
}
</script>
)JS";

// Live update of the home page
const char HOME_SCRIPT[] = R"JS(
<script>
function el(i){return document.getElementById(i);}
function st(i,t,c){var e=el(i);if(!e)return;e.textContent=t;if(c!==undefined)e.className=c;}
function upd(){
  fetch('/data',{cache:'no-store'}).then(function(r){
    if(r.status===401){location.href='/login';throw new Error('login');}
    return r.json();
  }).then(function(d){
    st('hState',d.system_healthy?'SYSTEM HEALTHY':'SYSTEM PROBLEM',d.system_healthy?'green big':'red big');
    st('hMsg',d.system_health_message);
    st('shtT',d.sht40_ok?d.sht40_temperature.toFixed(2)+' \u00B0C':'--');
    st('shtH',d.sht40_ok?d.sht40_humidity.toFixed(2)+' %RH':'Sensor not responding');
    st('dhtT',d.dht11_ok?d.dht11_temperature.toFixed(2)+' \u00B0C':'--');
    st('dhtH',d.dht11_ok?d.dht11_humidity.toFixed(2)+' %RH':'Sensor not responding');
    st('batV',d.battery_voltage.toFixed(3)+' V');
    st('batP',d.battery_percent.toFixed(1)+' %');
    if(d.wifi_state==='connected'){
      st('wState','CONNECTED','green');
      st('wInfo','SSID: '+d.wifi_ssid+' | IP: '+d.wifi_ip+' | RSSI: '+d.wifi_rssi+' dBm');
    }else if(d.wifi_state==='off'){
      st('wState','OFF - USING ETHERNET','yellow');
      st('wInfo','WiFi turns on again automatically if Ethernet disconnects.');
    }else if(d.wifi_state==='connecting'){
      st('wState','CONNECTING...','yellow');
      st('wInfo','Trying: '+d.wifi_ssid);
    }else{
      st('wState','NOT CONNECTED','red');
      st('wInfo',d.wifi_reason);
    }
    el('apBox').style.display=d.setup_ap?'block':'none';
    if(!d.ethernet_enabled)st('eState','DISABLED','yellow');
    else if(d.ethernet_connected)st('eState','CONNECTED','green');
    else if(d.ethernet_link)st('eState','CABLE CONNECTED - WAITING FOR IP','yellow');
    else st('eState','ENABLED - NOT CONNECTED','red');
    st('eInfo',d.ethernet_connected?'IP: '+d.ethernet_ip+' ('+d.ethernet_mode+')':'');
    st('pState',d.battery_power?'BATTERY POWER':'MAIN AC/DC POWER',d.battery_power?'yellow':'green');
    el('sw').style.display=d.switch_available?'block':'none';
    if(d.switch_available){st('swName',d.switch_ssid);el('swIdx').value=d.switch_index;}
    st('upd','Live - last update '+new Date().toLocaleTimeString(),'small');
  }).catch(function(){
    st('upd','Device not reachable - retrying...','red');
  });
}
setInterval(upd,2000);
</script>
)JS";


// ============================================================
// WEB: COMMON PAGE PARTS
// ============================================================

String pageHeader(const String &title)
{
  String html;
  html.reserve(4096);

  html += "<!DOCTYPE html><html><head>";
  html += "<meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>" + htmlEscape(title) + "</title>";

  html += "<style>";
  html += "*{box-sizing:border-box;}";
  html += "body{font-family:Arial,sans-serif;background:#101820;color:white;margin:0;padding:15px;}";
  html += "h1{color:#00d9ff;}";
  html += ".card{background:#1f2d38;padding:20px;margin:12px auto;border-radius:15px;max-width:700px;box-shadow:0 4px 15px rgba(0,0,0,.3);}";
  html += "label{display:block;color:#ccc;font-size:14px;}";
  html += "input,select{width:100%;padding:12px;margin:6px 0 14px;border-radius:8px;border:1px solid #52606b;background:#101820;color:white;font-size:16px;}";
  html += "button,.btn{display:inline-block;background:#00a8cc;color:white;padding:12px 18px;border:0;border-radius:8px;text-decoration:none;margin:4px;cursor:pointer;font-size:15px;}";
  html += ".danger{background:#d64545;}";
  html += ".grey{background:#52606b;}";
  html += ".green{color:#00ff88;}";
  html += ".yellow{color:#ffff00;}";
  html += ".red{color:#ff5555;}";
  html += ".big{font-size:22px;}";
  html += ".nav{max-width:1100px;margin:auto;text-align:center;}";
  html += ".network{background:#17232c;padding:15px;margin:10px 0;border-radius:10px;}";
  html += ".network h3{margin:0 0 10px;}";
  html += ".value{font-size:32px;font-weight:bold;color:#00d9ff;}";
  html += ".small{color:#aaa;font-size:14px;}";
  html += ".inline{display:inline;}";
  html += ".badge{display:inline-block;font-size:12px;font-weight:normal;padding:3px 8px;border-radius:10px;margin-left:6px;background:#2d4250;color:#ddd;vertical-align:middle;}";
  html += ".badge.g{background:#0b5a3a;color:#00ff88;}";
  html += ".badge.y{background:#5a5000;color:#ffff00;}";

  // Password field with eye icon
  html += ".pw{position:relative;}";
  html += ".pw input{padding-right:54px;}";
  html += ".eye{position:absolute;right:4px;top:6px;width:46px;height:45px;margin:0;padding:0;background:transparent;color:#aaa;display:flex;align-items:center;justify-content:center;border-radius:8px;}";
  html += ".eye:hover,.eye:focus{color:#00d9ff;}";
  html += ".eye .i2{display:none;}";
  html += ".eye.vis .i1{display:none;}";
  html += ".eye.vis .i2{display:block;}";

  // Switch notification
  html += ".banner{background:#3a3200;border:1px solid #ffd000;padding:15px 20px;margin:12px auto;border-radius:15px;max-width:700px;}";
  html += ".banner b{color:#ffd000;}";

  // Status tables and serial log
  html += "table.kv{width:100%;border-collapse:collapse;}";
  html += ".kv td{padding:7px 8px;border-bottom:1px solid #2d4250;vertical-align:top;word-break:break-word;}";
  html += ".kv td:first-child{color:#aaa;width:42%;}";
  html += ".log{background:#05090c;color:#9dffb0;font-family:Consolas,Menlo,monospace;font-size:13px;height:60vh;overflow:auto;white-space:pre-wrap;word-break:break-word;padding:12px;border-radius:8px;margin:10px 0;border:1px solid #2d4250;}";
  html += ".chk{display:inline-flex;align-items:center;gap:6px;margin:4px 10px 4px 0;color:#ddd;}";
  html += ".chk input{width:auto;margin:0;}";
  html += "</style>";

  html += PASSWORD_SCRIPT;

  html += "</head><body>";

  return html;
}

String navigation()
{
  String html;
  html += "<div class='nav'>";
  html += "<a class='btn' href='/'>Home</a>";
  html += "<a class='btn' href='/wifi'>WiFi</a>";
  html += "<a class='btn' href='/network'>Network</a>";
  html += "<a class='btn' href='/status'>Status</a>";
  html += "<a class='btn' href='/log'>Serial Log</a>";
  html += "<a class='btn' href='/users'>Users</a>";

  if (currentSession >= 0 && sessions[currentSession].active)
  {
    html += "<form class='inline' method='POST' action='/logout'>";
    html += "<button type='submit' class='grey'>Logout (";
    html += htmlEscape(sessions[currentSession].username);
    html += ")</button></form>";
  }

  html += "</div>";
  return html;
}

// Password input with eye icon (show / hide)
String passwordInput(const String &id, const String &name, const String &value, int minLength, const String &autocomplete)
{
  String html;
  html += "<div class='pw'>";
  html += "<input type='password' id='" + id + "' name='" + name + "' maxlength='64'";

  if (minLength > 0)
  {
    html += " minlength='" + String(minLength) + "'";
  }

  html += " autocomplete='" + autocomplete + "' autocapitalize='none' autocorrect='off' spellcheck='false' value='";
  html += htmlEscape(value);
  html += "'>";
  html += "<button type='button' class='eye' title='Show password' aria-label='Show password' ";
  html += "onclick=\"tp('" + id + "',this)\">";
  html += EYE_ICON_SHOW;
  html += EYE_ICON_HIDE;
  html += "</button></div>";
  return html;
}

// WiFi password field (WPA: 8-63 characters, or empty for open networks)
String passwordField(const String &id, const String &value)
{
  return passwordInput(id, "password", value, 8, "new-password");
}

String prioritySelect(int selected)
{
  String html = "<select name='priority'>";

  for (int p = 1; p <= MAX_WIFI_NETWORKS; p++)
  {
    html += "<option value='" + String(p) + "'";
    if (p == selected) html += " selected";
    html += ">" + String(p);
    if (p == 1) html += " - Highest";
    else if (p == MAX_WIFI_NETWORKS) html += " - Lowest";
    html += "</option>";
  }

  html += "</select>";
  return html;
}

bool switchAvailable()
{
  return !defaultOnlyMode && wifiConnected && isValidIndex(pendingSwitchIndex);
}

// Always rendered (hidden when empty) so the live page can show it
String switchBanner(const String &back)
{
  bool show = switchAvailable();
  String name = show ? savedWiFi[pendingSwitchIndex].name : String("");

  String html;
  html += "<div id='sw' class='banner' style='display:";
  html += show ? "block" : "none";
  html += "'>";
  html += "<b>Higher priority WiFi in range</b>";
  html += "<p><span id='swName'>" + htmlEscape(name) + "</span> is available. Switch to it now?</p>";

  html += "<form class='inline' method='POST' action='/wifi/switch'>";
  html += "<input type='hidden' id='swIdx' name='i' value='" + String(show ? pendingSwitchIndex : -1) + "'>";
  html += "<input type='hidden' name='back' value='" + back + "'>";
  html += "<button type='submit'>Switch now</button></form>";

  html += "<form class='inline' method='POST' action='/wifi/dismiss'>";
  html += "<input type='hidden' name='back' value='" + back + "'>";
  html += "<button type='submit' class='grey'>Not now</button></form>";

  html += "</div>";
  return html;
}

String wifiStateLabel()
{
  if (wifiSuspended()) return "OFF - USING ETHERNET";
  if (wifiConnected) return "CONNECTED";
  if (wifiState == WIFI_STATE_CONNECTING) return "CONNECTING...";
  return "NOT CONNECTED";
}

String wifiStateClass()
{
  if (wifiSuspended()) return "yellow";
  if (wifiConnected) return "green";
  if (wifiState == WIFI_STATE_CONNECTING) return "yellow";
  return "red";
}

String wifiInfoText()
{
  if (wifiSuspended())
  {
    return "WiFi turns on again automatically if Ethernet disconnects.";
  }

  if (wifiConnected && isValidIndex(currentWiFiIndex))
  {
    return "SSID: " + savedWiFi[currentWiFiIndex].name +
           " | IP: " + WiFi.localIP().toString() +
           " | RSSI: " + String(WiFi.RSSI()) + " dBm";
  }

  if (wifiState == WIFI_STATE_CONNECTING && isValidIndex(attemptIndex))
  {
    return "Trying: " + savedWiFi[attemptIndex].name;
  }

  return wifiFailureReason;
}

String ethernetStateLabel()
{
  if (!ethernetEnabled) return "DISABLED";
  if (ethernetConnected) return "CONNECTED";
  if (ethernetStarted && ethernetLinkUp) return "CABLE CONNECTED - WAITING FOR IP";
  return "ENABLED - NOT CONNECTED";
}

String ethernetStateClass()
{
  if (!ethernetEnabled) return "yellow";
  if (ethernetConnected) return "green";
  if (ethernetStarted && ethernetLinkUp) return "yellow";
  return "red";
}

String ethernetInfoText()
{
  if (!ethernetConnected) return "";

  return "IP: " + ETH.localIP().toString() + " (" + ethernetModeText() + ")";
}

void sendErrorPage(const String &message, const String &back)
{
  String html = pageHeader("Error");
  html += navigation();
  html += "<div class='card'><h1>Could not save</h1>";
  html += "<p class='red'>" + htmlEscape(message) + "</p>";
  html += "<a class='btn' href='" + back + "'>Back</a></div>";
  html += "</body></html>";
  server.send(400, "text/html", html);
}

// Returns an error message, or "" when OK
String validateWiFiInput(const String &ssid, const String &password, int ignoreIndex)
{
  String check = ssid;
  check.trim();

  if (check.length() == 0)   return "WiFi name (SSID) cannot be empty";
  if (ssid.length() > 32)    return "WiFi name (SSID) can be at most 32 characters";

  if (password.length() > 0 && password.length() < 8)
  {
    return "WiFi password must be at least 8 characters (or empty for an open network)";
  }

  if (password.length() > 63) return "WiFi password can be at most 63 characters";

  for (int i = 0; i < savedWiFiCount; i++)
  {
    if (i != ignoreIndex && savedWiFi[i].name == ssid)
    {
      return "This WiFi network is already saved";
    }
  }

  return "";
}


// ============================================================
// WIFI PAGE
// ============================================================

void handleWiFiPage()
{
  String html = pageHeader("WiFi Settings");
  html.reserve(10000);

  html += navigation();
  html += switchBanner("/wifi");

  html += "<div class='card'>";
  html += "<h1>WiFi Settings</h1>";

  html += "<p>Status: <span class='" + wifiStateClass() + "'>" + wifiStateLabel() + "</span><br>";
  html += "<span class='small'>" + htmlEscape(wifiInfoText()) + "</span></p>";

  if (setupAPActive)
  {
    html += "<p class='yellow'>Setup AP active: ESP32-SETUP (http://192.168.4.1)</p>";
  }

  // ----------------------------------------------------------
  // WiFi while Ethernet is connected
  // ----------------------------------------------------------

  html += "<h2>WiFi while Ethernet is connected</h2>";
  html += "<form method='POST' action='/wifi/eth-mode'>";
  html += "<select name='ethmode'>";

  html += "<option value='keep'";
  if (!wifiOffWhenEthernet) html += " selected";
  html += ">Keep WiFi on (WiFi and Ethernet together)</option>";

  html += "<option value='off'";
  if (wifiOffWhenEthernet) html += " selected";
  html += ">Turn WiFi off while Ethernet is connected</option>";

  html += "</select>";
  html += "<button type='submit'>Save</button>";
  html += "</form>";

  html += "<p class='small'>With \"Turn WiFi off\", WiFi and the setup AP switch off once Ethernet has been connected for 5 seconds. ";
  html += "If Ethernet disconnects or is disabled, WiFi turns on again and reconnects automatically, and the setup AP starts if no network connects. ";
  html += "User management needs WiFi, so choose \"Keep WiFi on\" while you add or edit users.</p>";

  // ----------------------------------------------------------
  // Connection mode
  // ----------------------------------------------------------

  html += "<h2>Connection Mode</h2>";
  html += "<form method='POST' action='/wifi/save-mode'>";
  html += "<select name='mode'>";

  html += "<option value='default'";
  if (defaultOnlyMode) html += " selected";
  html += ">Default WiFi only</option>";

  html += "<option value='priority'";
  if (!defaultOnlyMode) html += " selected";
  html += ">Priority mode (default first, then by priority)</option>";

  html += "</select>";
  html += "<button type='submit'>Save Mode</button>";
  html += "</form>";

  html += "<p class='small'>In priority mode the default network is tried first, then the others from highest to lowest priority. ";
  html += "When the default network comes back in range, the device switches to it automatically. ";
  html += "When another higher priority network comes in range, you are asked before switching.</p>";

  // ----------------------------------------------------------
  // Saved networks (in connection order)
  // ----------------------------------------------------------

  html += "<h2>Saved Networks</h2>";

  if (savedWiFiCount == 0)
  {
    html += "<p class='yellow'>No WiFi networks saved. Add one below.</p>";
  }
  else
  {
    html += "<p class='small'>Listed in the order the device tries them.</p>";
  }

  int order[MAX_WIFI_NETWORKS];
  int n = buildPriorityOrder(order);

  for (int k = 0; k < n; k++)
  {
    int i = order[k];
    bool isCurrent = wifiConnected && currentWiFiIndex == i;

    html += "<div class='network'>";
    html += "<h3>" + String(k + 1) + ". " + htmlEscape(savedWiFi[i].name);

    if (i == defaultWiFiIndex) html += "<span class='badge g'>DEFAULT</span>";
    html += "<span class='badge'>Priority " + String(savedWiFi[i].priority) + "</span>";
    if (isCurrent) html += "<span class='badge g'>CONNECTED</span>";
    if (!wifiConnected && wifiState == WIFI_STATE_CONNECTING && attemptIndex == i)
    {
      html += "<span class='badge y'>CONNECTING</span>";
    }

    html += "</h3>";

    html += "<a class='btn' href='/wifi/edit?i=" + String(i) + "'>Edit</a>";

    if (i != defaultWiFiIndex)
    {
      html += "<form class='inline' method='POST' action='/wifi/default'>";
      html += "<input type='hidden' name='i' value='" + String(i) + "'>";
      html += "<button type='submit'>Set Default</button></form>";
    }

    if (!defaultOnlyMode && !isCurrent && !wifiSuspended())
    {
      html += "<form class='inline' method='POST' action='/wifi/switch'>";
      html += "<input type='hidden' name='i' value='" + String(i) + "'>";
      html += "<input type='hidden' name='back' value='/wifi'>";
      html += "<button type='submit'>Connect now</button></form>";
    }

    html += "<form class='inline' method='POST' action='/wifi/delete' ";
    html += "onsubmit=\"return confirm('Delete this WiFi network?')\">";
    html += "<input type='hidden' name='i' value='" + String(i) + "'>";
    html += "<button type='submit' class='danger'>Delete</button></form>";

    html += "</div>";
  }

  // ----------------------------------------------------------
  // Add network
  // ----------------------------------------------------------

  if (savedWiFiCount < MAX_WIFI_NETWORKS)
  {
    int suggested = clampPriority(savedWiFiCount + 1);

    html += "<h2>Add WiFi Network</h2>";
    html += "<form method='POST' action='/wifi/add'>";

    html += "<label>WiFi Name / SSID</label>";
    html += "<input name='ssid' required maxlength='32' autocapitalize='none' autocorrect='off' spellcheck='false'>";

    html += "<label>WiFi Password</label>";
    html += passwordField("addPass", "");

    html += "<label>Priority</label>";
    html += prioritySelect(suggested);

    html += "<button type='submit'>Add Network</button>";
    html += "</form>";
  }
  else
  {
    html += "<p class='yellow'>Maximum 5 WiFi networks stored. Delete one to add another.</p>";
  }

  html += "</div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}


// ============================================================
// SAVE "WIFI WHILE ETHERNET IS CONNECTED"
// ============================================================

void handleSaveEthMode()
{
  bool newOff = server.arg("ethmode") == "off";
  bool changed = newOff != wifiOffWhenEthernet;

  wifiOffWhenEthernet = newOff;
  saveWiFiSettings();

  // Give the reply time to reach the browser before WiFi goes off
  wifiOffHoldStart = millis();

  bool viaWiFi = wifiConnected && server.client().localIP() == WiFi.localIP();

  if (!(changed && newOff && ethernetConnected && viaWiFi))
  {
    redirectTo("/wifi");
    return;
  }

  // The user is on WiFi and WiFi is about to turn off: show the Ethernet address
  String ethURL = "http://" + ETH.localIP().toString() + "/";

  String html = pageHeader("WiFi turning off");
  html += navigation();
  html += "<div class='card'>";
  html += "<h1>WiFi is turning off</h1>";
  html += "<p>Ethernet is connected, so WiFi switches off in a few seconds and this WiFi connection will stop working.</p>";
  html += "<p>Continue through Ethernet: <a class='green' href='" + ethURL + "'>" + ethURL + "</a></p>";
  html += "<p class='small'>Your computer must be on the same wired network as the device. ";
  html += "If Ethernet disconnects, WiFi turns on again automatically.</p>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}


// ============================================================
// ADD WIFI
// ============================================================

void handleAddWiFi()
{
  if (savedWiFiCount >= MAX_WIFI_NETWORKS)
  {
    sendErrorPage("Maximum WiFi networks reached", "/wifi");
    return;
  }

  String ssid = server.arg("ssid");
  String password = server.arg("password");
  int priority = clampPriority(server.arg("priority").toInt());

  String error = validateWiFiInput(ssid, password, -1);

  if (error.length() > 0)
  {
    sendErrorPage(error, "/wifi");
    return;
  }

  savedWiFi[savedWiFiCount].name = ssid;
  savedWiFi[savedWiFiCount].password = password;
  savedWiFi[savedWiFiCount].priority = priority;

  if (savedWiFiCount == 0)
  {
    defaultWiFiIndex = 0;
  }

  savedWiFiCount++;

  saveWiFiSettings();
  settingsChanged(false);

  redirectTo("/wifi");
}


// ============================================================
// EDIT WIFI PAGE
// ============================================================

void handleEditWiFi()
{
  int index = argIndex();

  if (!isValidIndex(index))
  {
    sendErrorPage("WiFi network not found", "/wifi");
    return;
  }

  String html = pageHeader("Edit WiFi");
  html += navigation();

  html += "<div class='card'>";
  html += "<h1>Edit WiFi Network</h1>";

  html += "<form method='POST' action='/wifi/edit-save'>";
  html += "<input type='hidden' name='i' value='" + String(index) + "'>";

  html += "<label>WiFi Name / SSID</label>";
  html += "<input name='ssid' value='" + htmlEscape(savedWiFi[index].name) + "' ";
  html += "required maxlength='32' autocapitalize='none' autocorrect='off' spellcheck='false'>";

  html += "<label>WiFi Password</label>";
  html += passwordField("editPass", savedWiFi[index].password);

  html += "<label>Priority</label>";
  html += prioritySelect(savedWiFi[index].priority);

  if (index == defaultWiFiIndex)
  {
    html += "<p class='small'>This is the default network, so it is always tried first.</p>";
  }

  html += "<button type='submit'>Save Changes</button>";
  html += "<a class='btn grey' href='/wifi'>Cancel</a>";
  html += "</form>";

  html += "</div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}


// ============================================================
// SAVE EDITED WIFI
// ============================================================

void handleEditWiFiSave()
{
  int index = argIndex();

  if (!isValidIndex(index))
  {
    sendErrorPage("WiFi network not found", "/wifi");
    return;
  }

  String ssid = server.arg("ssid");
  String password = server.arg("password");
  int priority = clampPriority(server.arg("priority").toInt());

  String error = validateWiFiInput(ssid, password, index);

  if (error.length() > 0)
  {
    sendErrorPage(error, "/wifi/edit?i=" + String(index));
    return;
  }

  bool credentialsChanged =
    ssid != savedWiFi[index].name ||
    password != savedWiFi[index].password;

  savedWiFi[index].name = ssid;
  savedWiFi[index].password = password;
  savedWiFi[index].priority = priority;

  saveWiFiSettings();

  bool currentAffected = wifiConnected && index == currentWiFiIndex && credentialsChanged;
  settingsChanged(currentAffected);

  redirectTo("/wifi");
}


// ============================================================
// DELETE WIFI
// ============================================================

void shiftIndexAfterDelete(int &ref, int deleted)
{
  if (ref == deleted)     ref = -1;
  else if (ref > deleted) ref--;
}

void handleDeleteWiFi()
{
  int index = argIndex();

  if (!isValidIndex(index))
  {
    redirectTo("/wifi");
    return;
  }

  bool wasCurrent = wifiConnected && index == currentWiFiIndex;
  bool wasDefault = index == defaultWiFiIndex;

  for (int i = index; i < savedWiFiCount - 1; i++)
  {
    savedWiFi[i] = savedWiFi[i + 1];
  }

  savedWiFiCount--;

  // Default: if deleted, pick the remaining network with the best priority
  if (savedWiFiCount == 0)
  {
    defaultWiFiIndex = -1;
  }
  else if (wasDefault)
  {
    int best = 0;
    for (int i = 1; i < savedWiFiCount; i++)
    {
      if (savedWiFi[i].priority < savedWiFi[best].priority) best = i;
    }
    defaultWiFiIndex = best;
  }
  else if (index < defaultWiFiIndex)
  {
    defaultWiFiIndex--;
  }

  shiftIndexAfterDelete(currentWiFiIndex, index);
  shiftIndexAfterDelete(lastConnectedIndex, index);
  shiftIndexAfterDelete(previousWiFiIndex, index);
  shiftIndexAfterDelete(pendingSwitchIndex, index);
  shiftIndexAfterDelete(dismissedSwitchIndex, index);
  shiftIndexAfterDelete(requestedSwitchIndex, index);
  shiftIndexAfterDelete(switchTargetIndex, index);
  shiftIndexAfterDelete(attemptIndex, index);

  saveWiFiSettings();
  settingsChanged(wasCurrent);

  redirectTo("/wifi");
}


// ============================================================
// SET DEFAULT WIFI
// ============================================================
// Does not change the connection mode. In priority mode,
// if the new default is in range, the device switches to it
// automatically within a few seconds.

void handleSetDefaultWiFi()
{
  int index = argIndex();

  if (isValidIndex(index))
  {
    defaultWiFiIndex = index;
    saveWiFiSettings();
    settingsChanged(false);
  }

  redirectTo("/wifi");
}


// ============================================================
// SAVE WIFI MODE
// ============================================================

void handleSaveWiFiMode()
{
  defaultOnlyMode = server.arg("mode") == "default";

  saveWiFiSettings();
  settingsChanged(false);

  redirectTo("/wifi");
}


// ============================================================
// SWITCH / DISMISS
// ============================================================

void handleSwitchWiFi()
{
  int index = argIndex();

  if (!isValidIndex(index))
  {
    redirectTo(safeBack());
    return;
  }

  if (wifiSuspended())
  {
    sendErrorPage("WiFi is turned off because Ethernet is connected. "
                  "Choose \"Keep WiFi on\" on the WiFi page first.", "/wifi");
    return;
  }

  requestedSwitchIndex = index;
  pendingSwitchIndex = -1;

  // The user chose this network: clear any "Not now" for it
  if (dismissedSwitchIndex == index)
  {
    dismissedSwitchIndex = -1;
  }

  String html = pageHeader("Switching WiFi");
  html += navigation();
  html += "<div class='card'>";
  html += "<h1>Switching WiFi</h1>";
  html += "<p>The device is connecting to <b>" + htmlEscape(savedWiFi[index].name) + "</b>.</p>";
  html += "<p>If it cannot connect within about 15 seconds, it goes back to the network it was using.</p>";
  html += "<p class='small'>If you opened this page over WiFi, the device gets a new IP address on the new network. ";
  html += "Find it in your router or the Serial Monitor. Ethernet is not affected.</p>";
  html += "<a class='btn' href='" + safeBack() + "'>Back</a>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleDismissSwitch()
{
  if (isValidIndex(pendingSwitchIndex))
  {
    dismissedSwitchIndex = pendingSwitchIndex;
    dismissedAt = millis();

    SerialLog.print("Switch to ");
    SerialLog.print(savedWiFi[pendingSwitchIndex].name);
    SerialLog.println(" postponed for 10 minutes");
  }

  pendingSwitchIndex = -1;

  redirectTo(safeBack());
}


// ============================================================
// NETWORK SETTINGS PAGE
// ============================================================

void handleNetworkPage()
{
  String html = pageHeader("Network Settings");
  html.reserve(7000);

  html += navigation();

  html += "<div class='card'>";
  html += "<h1>Network Settings</h1>";
  html += "<h2>WiFi IP</h2>";
  html += "<p class='small'>These settings apply to WiFi only. Ethernet has its own settings below.</p>";

  html += "<form method='POST' action='/network/save'>";

  html += "<label>IP Configuration</label>";
  html += "<select name='ipmode'>";
  html += "<option value='dhcp'";
  if (!staticIPEnabled) html += " selected";
  html += ">DHCP / Automatic</option>";
  html += "<option value='static'";
  if (staticIPEnabled) html += " selected";
  html += ">Static IP</option>";
  html += "</select>";

  html += "<label>IP Address</label>";
  html += "<input name='ip' value='" + htmlEscape(staticIPAddress) + "' placeholder='192.168.1.100'>";

  html += "<label>Gateway</label>";
  html += "<input name='gateway' value='" + htmlEscape(staticGateway) + "' placeholder='192.168.1.1'>";

  html += "<label>Subnet Mask</label>";
  html += "<input name='subnet' value='" + htmlEscape(staticSubnet) + "' placeholder='255.255.255.0'>";

  html += "<label>DNS</label>";
  html += "<input name='dns' value='" + htmlEscape(staticDNS) + "' placeholder='8.8.8.8'>";

  html += "<button type='submit'>Save Network Settings</button>";
  html += "</form>";
  html += "</div>";

  // Current status
  html += "<div class='card'>";
  html += "<h2>Current WiFi Status</h2>";
  html += "<p class='" + wifiStateClass() + "'>" + wifiStateLabel() + "</p>";
  html += "<p>" + htmlEscape(wifiInfoText()) + "</p>";

  if (setupAPActive)
  {
    html += "<hr><p class='yellow'>Setup AP active</p><p>Setup IP: 192.168.4.1</p>";
  }

  html += "</div>";

  // Ethernet IP settings
  html += "<div class='card'>";
  html += "<h2>Ethernet IP</h2>";
  html += "<p>Status: <span class='" + ethernetStateClass() + "'>" + ethernetStateLabel() + "</span><br>";
  html += "<span class='small'>" + htmlEscape(ethernetInfoText()) + "</span></p>";

  html += "<form method='POST' action='/network/eth-save'>";

  html += "<label>IP Configuration</label>";
  html += "<select name='ethipmode'>";
  html += "<option value='dhcp'";
  if (!ethStaticEnabled) html += " selected";
  html += ">DHCP / Automatic</option>";
  html += "<option value='static'";
  if (ethStaticEnabled) html += " selected";
  html += ">Static IP</option>";
  html += "</select>";

  html += "<label>IP Address</label>";
  html += "<input name='ethip' value='" + htmlEscape(ethStaticIP) + "' placeholder='192.168.1.50'>";

  html += "<label>Subnet Mask</label>";
  html += "<input name='ethsubnet' value='" + htmlEscape(ethStaticSubnet) + "' placeholder='255.255.255.0'>";

  html += "<label>Gateway (optional)</label>";
  html += "<input name='ethgateway' value='" + htmlEscape(ethStaticGateway) + "' placeholder='192.168.1.1'>";

  html += "<label>DNS (optional)</label>";
  html += "<input name='ethdns' value='" + htmlEscape(ethStaticDNS) + "' placeholder='192.168.1.1'>";

  html += "<button type='submit'>Save Ethernet Settings</button>";
  html += "</form>";

  html += "<p class='small'><b>DHCP:</b> the router gives the address. If no DHCP server answers within 15 seconds ";
  html += "(for example a cable straight to a laptop), the device uses <b>192.168.50.2</b>; set the laptop to 192.168.50.1 / 255.255.255.0.</p>";
  html += "<p class='small'><b>Static IP:</b> the address you enter is always used. Ask the network administrator for a free address ";
  html += "outside the router's DHCP range. Gateway and DNS can stay empty on a local network. Changes apply within a second, no restart.</p>";
  html += "</div>";

  // Restart
  html += "<div class='card'>";
  html += "<h2>Device</h2>";
  html += "<form method='POST' action='/restart' onsubmit=\"return confirm('Restart the ESP32?')\">";
  html += "<button type='submit' class='danger'>Restart ESP32</button></form>";
  html += "</div>";

  html += "</body></html>";

  server.send(200, "text/html", html);
}


// ============================================================
// SAVE NETWORK SETTINGS
// ============================================================

void handleNetworkSave()
{
  bool newStatic = server.arg("ipmode") == "static";

  String ip      = server.arg("ip");
  String gateway = server.arg("gateway");
  String subnet  = server.arg("subnet");
  String dns     = server.arg("dns");

  ip.trim();
  gateway.trim();
  subnet.trim();
  dns.trim();

  // Validate before changing anything
  if (newStatic)
  {
    IPAddress test;

    if (!parseIP(ip, test))      { sendErrorPage("Invalid IP address", "/network"); return; }
    if (!parseIP(gateway, test)) { sendErrorPage("Invalid gateway", "/network");    return; }
    if (!parseIP(subnet, test))  { sendErrorPage("Invalid subnet mask", "/network"); return; }
    if (!parseIP(dns, test))     { sendErrorPage("Invalid DNS", "/network");        return; }
  }

  bool changed =
    newStatic != staticIPEnabled ||
    ip != staticIPAddress ||
    gateway != staticGateway ||
    subnet != staticSubnet ||
    dns != staticDNS;

  staticIPEnabled = newStatic;
  staticIPAddress = ip;
  staticGateway   = gateway;
  staticSubnet    = subnet;
  staticDNS       = dns;

  saveWiFiSettings();

  if (changed && savedWiFiCount > 0 && !wifiSuspended())
  {
    reconnectRequested = true;
  }

  String html = pageHeader("Network Saved");
  html += navigation();
  html += "<div class='card'>";
  html += "<h1>Network Settings Saved</h1>";

  if (changed && wifiSuspended())
  {
    html += "<p>Saved. WiFi is off while Ethernet is connected, so the new settings are used the next time WiFi turns on.</p>";
  }
  else if (changed)
  {
    html += "<p>WiFi is reconnecting with the new settings. No restart needed.</p>";

    if (newStatic)
    {
      html += "<p>New address: <a class='green' href='http://" + ip + "/'>http://" + ip + "</a></p>";
    }
    else
    {
      html += "<p class='small'>The new DHCP address is shown in your router or the Serial Monitor.</p>";
    }
  }
  else
  {
    html += "<p>Nothing changed.</p>";
  }

  html += "<a class='btn' href='/network'>Back</a>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}


// ============================================================
// SAVE ETHERNET IP SETTINGS
// ============================================================

void handleEthernetSave()
{
  bool newStatic = server.arg("ethipmode") == "static";

  String ip      = server.arg("ethip");
  String gateway = server.arg("ethgateway");
  String subnet  = server.arg("ethsubnet");
  String dns     = server.arg("ethdns");

  ip.trim();
  gateway.trim();
  subnet.trim();
  dns.trim();

  // Validate before changing anything
  if (newStatic)
  {
    IPAddress testIP, test;

    if (!parseIP(ip, testIP))            { sendErrorPage("Invalid Ethernet IP address", "/network"); return; }
    if (!parseIP(subnet, test))          { sendErrorPage("Invalid Ethernet subnet mask", "/network"); return; }
    if (!parseOptionalIP(gateway, test)) { sendErrorPage("Invalid Ethernet gateway", "/network"); return; }
    if (!parseOptionalIP(dns, test))     { sendErrorPage("Invalid Ethernet DNS", "/network"); return; }

    if (testIP[0] == 192 && testIP[1] == 168 && testIP[2] == 4)
    {
      sendErrorPage("192.168.4.x is used by the setup AP. Choose an address in another range.", "/network");
      return;
    }
  }

  bool changed =
    newStatic != ethStaticEnabled ||
    ip != ethStaticIP ||
    gateway != ethStaticGateway ||
    subnet != ethStaticSubnet ||
    dns != ethStaticDNS;

  // Is this browser using Ethernet right now?
  bool viaEthernet = ethernetConnected && server.client().localIP() == ETH.localIP();

  ethStaticEnabled = newStatic;
  ethStaticIP      = ip;
  ethStaticGateway = gateway;
  ethStaticSubnet  = subnet;
  ethStaticDNS     = dns;

  saveEthernetSettings();

  if (changed)
  {
    ethConfigPending = true;
    ethConfigPendingAt = millis();
  }

  String html = pageHeader("Ethernet Saved");
  html += navigation();
  html += "<div class='card'>";
  html += "<h1>Ethernet Settings Saved</h1>";

  if (!changed)
  {
    html += "<p>Nothing changed.</p>";
  }
  else
  {
    if (ethernetStarted)
    {
      html += "<p>The new settings are applied in about one second. No restart needed.</p>";
    }
    else
    {
      html += "<p>Ethernet is not running right now. The settings are used when it starts.</p>";
    }

    if (newStatic)
    {
      html += "<p>Ethernet address: <a class='green' href='http://" + ip + "/'>http://" + ip + "</a></p>";
    }
    else
    {
      html += "<p class='small'>The new DHCP address is shown in the router, the Serial Monitor and the Status page (over WiFi).</p>";
    }

    if (viaEthernet)
    {
      html += "<p class='yellow'>You are connected through Ethernet. If the address changes, this connection stops; ";
      html += "open the new address instead. Your computer must be in the same address range.</p>";
    }
  }

  html += "<a class='btn' href='/network'>Back</a>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}


// ============================================================
// STATUS PAGE (network communication information)
// ============================================================

String kvRow(const char *key, const String &value)
{
  return String("<tr><td>") + key + "</td><td>" + htmlEscape(value) + "</td></tr>";
}

String ipText(const IPAddress &ip)
{
  return ipIsSet(ip) ? ip.toString() : String("-");
}

String formatDuration(unsigned long ms)
{
  unsigned long s = ms / 1000UL;
  unsigned long days = s / 86400UL;
  s %= 86400UL;

  char buf[40];
  snprintf(buf, sizeof(buf), "%lud %02lu:%02lu:%02lu", days, s / 3600UL, (s / 60UL) % 60UL, s % 60UL);
  return String(buf);
}

String macText(esp_mac_type_t type)
{
  uint8_t m[6];

  if (esp_read_mac(m, type) != ESP_OK) return "-";

  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
  return String(buf);
}

String resetReasonText(esp_reset_reason_t reason)
{
  switch (reason)
  {
    case ESP_RST_POWERON:   return "Power on";
    case ESP_RST_EXT:       return "External reset pin";
    case ESP_RST_SW:        return "Software restart";
    case ESP_RST_PANIC:     return "Crash (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:       return "Watchdog";
    case ESP_RST_DEEPSLEEP: return "Wake from deep sleep";
    case ESP_RST_BROWNOUT:  return "Brownout (supply voltage too low)";
    default:                return "Other (" + String((int)reason) + ")";
  }
}

int activeSessionCount()
{
  int n = 0;

  for (int i = 0; i < MAX_SESSIONS; i++)
  {
    if (sessions[i].active && millis() - sessions[i].lastActive < SESSION_TIMEOUT) n++;
  }

  return n;
}

// The inner part of the Status page (also sent every 3 s)
String statusFragment()
{
  String h;
  h.reserve(7000);

  IPAddress via = server.client().localIP();

  String viaText;
  if (wifiConnected && via == WiFi.localIP())             viaText = "WiFi";
  else if (ethernetConnected && via == ETH.localIP())    viaText = "Ethernet";
  else if (setupAPActive && via == AP_IP)                viaText = "Setup AP";
  else                                                   viaText = via.toString();

  // ----------------------------------------------------------
  // Reach the device at
  // ----------------------------------------------------------

  h += "<div class='card'><h2>Reach the device at</h2><table class='kv'>";

  bool any = false;

  if (wifiConnected)
  {
    h += kvRow("WiFi", "http://" + WiFi.localIP().toString());
    any = true;
  }

  if (ethernetConnected)
  {
    h += kvRow("Ethernet", "http://" + ETH.localIP().toString());
    any = true;
  }

  if (setupAPActive)
  {
    h += kvRow("Setup AP", "http://192.168.4.1 (WiFi " + String(SETUP_AP_SSID) + ")");
    any = true;
  }

  if (!any) h += kvRow("Network", "No network connected");

  h += kvRow("Your browser uses", viaText);
  h += kvRow("Your browser's address", server.client().remoteIP().toString());
  h += "</table></div>";

  // ----------------------------------------------------------
  // Device
  // ----------------------------------------------------------

  h += "<div class='card'><h2>Device</h2><table class='kv'>";
  h += kvRow("Firmware", "ESP32 System Monitor v4");
  h += kvRow("Uptime", formatDuration(millis()));
  h += kvRow("Last restart reason", resetReasonText(bootReason));
  h += kvRow("Chip", String(ESP.getChipModel()) + " rev " + String(ESP.getChipRevision()) +
                     ", " + String(ESP.getCpuFreqMHz()) + " MHz");
  h += kvRow("Free memory", String(ESP.getFreeHeap() / 1024) + " KB (lowest " +
                            String(ESP.getMinFreeHeap() / 1024) + " KB)");
  h += kvRow("Logged-in sessions", String(activeSessionCount()) + " of " + String(MAX_SESSIONS));
  h += kvRow("System health", systemHealthMessage);
  h += "</table></div>";

  // ----------------------------------------------------------
  // WiFi
  // ----------------------------------------------------------

  h += "<div class='card'><h2>WiFi</h2><table class='kv'>";
  h += kvRow("Status", wifiStateLabel());
  h += kvRow("Connection mode", defaultOnlyMode ? "Default WiFi only" : "Priority mode");
  h += kvRow("While Ethernet is connected", wifiOffWhenEthernet ? "WiFi turned off" : "WiFi kept on");
  h += kvRow("Saved networks", String(savedWiFiCount) + " of " + String(MAX_WIFI_NETWORKS));

  if (wifiConnected && isValidIndex(currentWiFiIndex))
  {
    String net = savedWiFi[currentWiFiIndex].name;
    if (currentWiFiIndex == defaultWiFiIndex) net += " (default)";
    net += ", priority " + String(savedWiFi[currentWiFiIndex].priority);

    int rssi = WiFi.RSSI();
    int quality = constrain(2 * (rssi + 100), 0, 100);

    h += kvRow("Network", net);
    h += kvRow("Router (BSSID)", WiFi.BSSIDstr());
    h += kvRow("Channel", String(WiFi.channel()));
    h += kvRow("Signal", String(rssi) + " dBm (" + String(quality) + " %)");
    h += kvRow("IP mode", staticIPEnabled ? "Static IP" : "DHCP");
    h += kvRow("IP address", ipText(WiFi.localIP()));
    h += kvRow("Subnet mask", ipText(WiFi.subnetMask()));
    h += kvRow("Gateway", ipText(WiFi.gatewayIP()));
    h += kvRow("DNS", ipText(WiFi.dnsIP(0)));
    h += kvRow("Connected for", formatDuration(millis() - wifiConnectedAt));
  }
  else
  {
    if (wifiState == WIFI_STATE_CONNECTING && isValidIndex(attemptIndex))
    {
      h += kvRow("Trying", savedWiFi[attemptIndex].name);
    }

    h += kvRow("IP mode", staticIPEnabled ? "Static IP (" + staticIPAddress + ")" : String("DHCP"));
  }

  const char *host = WiFi.getHostname();

  h += kvRow("MAC address", macText(ESP_MAC_WIFI_STA));
  h += kvRow("Hostname", host ? String(host) : String("-"));
  h += kvRow("Connections since boot", String(wifiConnectCount));
  h += kvRow("Connection losses", String(wifiLostCount));
  h += kvRow("Last problem", wifiFailureReason.length() ? wifiFailureReason : String("-"));
  h += kvRow("Previous network", isValidIndex(previousWiFiIndex) ? savedWiFi[previousWiFiIndex].name : String("-"));
  h += kvRow("Higher priority network waiting", switchAvailable() ? savedWiFi[pendingSwitchIndex].name : String("-"));
  h += "</table></div>";

  // ----------------------------------------------------------
  // Ethernet
  // ----------------------------------------------------------

  bool linkUp = ethernetStarted && ethernetLinkUp;

  String driver;
  if (!ethernetEnabled)          driver = "Off (enable pin GPIO33 LOW)";
  else if (ethernetStartFailed)  driver = "W5500 start failed - retrying every 30 s";
  else if (ethernetStarted)      driver = "Running";
  else                           driver = "Stopped";

  h += "<div class='card'><h2>Ethernet (W5500)</h2><table class='kv'>";
  h += kvRow("Status", ethernetStateLabel());
  h += kvRow("Driver", driver);
  h += kvRow("Cable link", linkUp ? String("Up") : String("Down (no cable or no link)"));

  if (linkUp)
  {
    h += kvRow("Speed", String(ETH.linkSpeed()) + " Mbit/s, " + (ETH.fullDuplex() ? "full" : "half") + " duplex");
  }

  h += kvRow("IP setting", ethStaticEnabled
                           ? "Static IP (" + ethStaticIP + ")"
                           : String("DHCP (fallback 192.168.50.2 after 15 s)"));

  if (ethernetConnected)
  {
    h += kvRow("Address in use", ethernetModeText());
    h += kvRow("IP address", ipText(ETH.localIP()));
    h += kvRow("Subnet mask", ipText(ETH.subnetMask()));
    h += kvRow("Gateway", ipText(ETH.gatewayIP()));
    h += kvRow("DNS", ipText(ETH.dnsIP(0)));
    h += kvRow("Connected for", formatDuration(millis() - ethernetConnectedSince));
  }

  h += kvRow("MAC address", ethernetStarted ? ETH.macAddress() : String("- (shown while Ethernet runs)"));
  h += kvRow("Connections since boot", String(ethConnectCount));
  h += kvRow("W5500 restarts", String(ethRestartCount));
  h += "</table></div>";

  // ----------------------------------------------------------
  // Setup AP
  // ----------------------------------------------------------

  h += "<div class='card'><h2>Setup AP</h2><table class='kv'>";

  if (setupAPActive)
  {
    h += kvRow("Status", "ON (no network was connected)");
    h += kvRow("WiFi name", SETUP_AP_SSID);
    h += kvRow("IP address", WiFi.softAPIP().toString());
    h += kvRow("MAC address", macText(ESP_MAC_WIFI_SOFTAP));
    h += kvRow("Connected devices", String(WiFi.softAPgetStationNum()));
  }
  else
  {
    h += kvRow("Status", "OFF - starts automatically when no network is connected for 5 s");
  }

  h += "</table></div>";

  return h;
}

const char STATUS_SCRIPT[] = R"JS(
<script>
function rs(){
  fetch('/status/data',{cache:'no-store'}).then(function(r){
    if(r.status===401){location.href='/login';throw 0;}
    return r.text();
  }).then(function(h){
    document.getElementById('st').innerHTML=h;
    var u=document.getElementById('su');u.textContent='Live - last update '+new Date().toLocaleTimeString();u.className='small';
  }).catch(function(e){
    if(e!==0){var u=document.getElementById('su');u.textContent='Device not reachable - retrying...';u.className='red';}
  });
}
setInterval(rs,3000);
</script>
)JS";

void handleStatusPage()
{
  String html = pageHeader("Network Status");
  html.reserve(10000);

  html += navigation();
  html += "<div class='card'><h1>Network Status</h1>";
  html += "<p id='su' class='small'>Updates every 3 seconds.</p></div>";
  html += "<div id='st'>" + statusFragment() + "</div>";
  html += STATUS_SCRIPT;
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleStatusData()
{
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", statusFragment());
}


// ============================================================
// SERIAL LOG PAGE (needs the user's own password again)
// ============================================================

// Copy of the log from position "from". "next" = position to ask
// for next time. "cut" = some text was overwritten / device restarted.
char logReadBuffer[LOG_BUFFER_SIZE + 1];

String readLog(uint32_t from, uint32_t &next, bool &cut)
{
  size_t n;

  portENTER_CRITICAL(&logMux);

  uint32_t total = logTotal;
  uint32_t oldest = total > LOG_BUFFER_SIZE ? total - LOG_BUFFER_SIZE : 0;

  cut = false;

  if (from < oldest || from > total)
  {
    cut = true;
    from = oldest;
  }

  n = total - from;

  for (size_t i = 0; i < n; i++)
  {
    logReadBuffer[i] = logBuffer[(from + i) % LOG_BUFFER_SIZE];
  }

  portEXIT_CRITICAL(&logMux);

  logReadBuffer[n] = 0;
  next = total;

  return String(logReadBuffer);
}

// Forward declarations (defined in the login section)
String hashPassword(const String &salt, const String &password);
bool secureEquals(const String &a, const String &b);
int findUser(const String &username);
unsigned long loginLockRemaining();
void registerLoginFailure();
String lockMessage(unsigned long seconds);

bool logUnlocked()
{
  if (currentSession < 0) return false;

  Session &s = sessions[currentSession];

  if (!s.logUnlocked) return false;

  if (millis() - s.logUnlockedAt >= LOG_UNLOCK_TIME)
  {
    s.logUnlocked = false;
    return false;
  }

  return true;
}

void sendLogUnlockPage(const String &message, int statusCode)
{
  String html = pageHeader("Serial Log");
  html += navigation();

  html += "<div class='card'>";
  html += "<h1>Serial Log</h1>";
  html += "<p>For security, enter your password again to view the serial log.</p>";
  html += "<p class='small'>Signed in as <b>" + htmlEscape(sessions[currentSession].username) + "</b>. ";
  html += "The log stays unlocked for 10 minutes without activity.</p>";

  if (message.length() > 0)
  {
    html += "<p class='red'>" + htmlEscape(message) + "</p>";
  }

  html += "<form method='POST' action='/log/unlock'>";
  html += "<label>Your password</label>";
  html += passwordInput("logPass", "password", "", 0, "current-password");
  html += "<button type='submit'>Open Serial Log</button>";
  html += "</form>";
  html += "</div></body></html>";

  server.send(statusCode, "text/html", html);
}

void handleLogUnlock()
{
  unsigned long wait = loginLockRemaining();

  if (wait > 0)
  {
    sendLogUnlockPage(lockMessage(wait), 429);
    return;
  }

  int index = findUser(sessions[currentSession].username);
  String password = server.arg("password");

  bool ok = index >= 0 &&
            secureEquals(hashPassword(users[index].salt, password), users[index].hash);

  if (!ok)
  {
    registerLoginFailure();
    sendLogUnlockPage("Wrong password.", 401);
    return;
  }

  loginFailures = 0;

  sessions[currentSession].logUnlocked = true;
  sessions[currentSession].logUnlockedAt = millis();

  SerialLog.print("Serial log opened by: ");
  SerialLog.println(sessions[currentSession].username);

  redirectTo("/log");
}

const char LOG_SCRIPT[] = R"JS(
<script>
var nx=0,ps=false,lk=false;
function g(i){return document.getElementById(i);}
function ad(t){
  var e=g('lg');
  e.textContent+=t;
  if(e.textContent.length>60000)e.textContent=e.textContent.slice(-50000);
  if(g('as').checked)e.scrollTop=e.scrollHeight;
}
function stt(t,c){var s=g('ls');s.textContent=t;s.className=c;}
function pl(){
  if(ps||lk)return;
  fetch('/log/data?from='+nx,{cache:'no-store'}).then(function(r){
    if(r.status===401){lk=true;stt('Locked - reload the page and enter your password again.','red');throw 0;}
    var n=r.headers.get('X-Log-Next'),c=r.headers.get('X-Log-Cut');
    return r.text().then(function(t){
      if(c==='1'&&nx>0)ad('\n--- some lines were missed (overwritten or device restarted) ---\n');
      if(t.length)ad(t);
      nx=parseInt(n,10)||0;
      stt('Live - last update '+new Date().toLocaleTimeString(),'small');
    });
  }).catch(function(e){if(e!==0)stt('Device not reachable - retrying...','red');});
}
function tg(){ps=!ps;g('pb').textContent=ps?'Resume':'Pause';if(!ps)pl();}
function cl(){g('lg').textContent='';}
setInterval(pl,1000);pl();
</script>
)JS";

void handleLogPage()
{
  if (!logUnlocked())
  {
    sendLogUnlockPage("", 200);
    return;
  }

  sessions[currentSession].logUnlockedAt = millis();

  String html = pageHeader("Serial Log");
  html.reserve(6000);
  html += navigation();

  html += "<div class='card' style='max-width:1000px'>";
  html += "<h1>Serial Log</h1>";
  html += "<p class='small'>Everything the device prints to the Serial Monitor (last 8 KB), with the time since boot. ";
  html += "Messages printed before this page was opened are included as long as they are still in the buffer.</p>";

  html += "<label class='chk'><input type='checkbox' id='as' checked>Auto-scroll</label>";
  html += "<button type='button' id='pb' onclick='tg()'>Pause</button>";
  html += "<button type='button' class='grey' onclick='cl()'>Clear view</button>";
  html += "<a class='btn grey' href='/log/download'>Download</a>";

  html += "<pre id='lg' class='log'></pre>";
  html += "<p id='ls' class='small'>Loading...</p>";
  html += "</div>";

  html += LOG_SCRIPT;
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleLogData()
{
  if (!logUnlocked())
  {
    server.send(401, "application/json", "{\"error\":\"locked\"}");
    return;
  }

  // An open log page keeps the unlock alive
  sessions[currentSession].logUnlockedAt = millis();

  uint32_t from = (uint32_t)strtoul(server.arg("from").c_str(), nullptr, 10);
  uint32_t next = 0;
  bool cut = false;

  String text = readLog(from, next, cut);

  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Log-Next", String(next));
  server.sendHeader("X-Log-Cut", cut ? "1" : "0");
  server.send(200, "text/plain; charset=utf-8", text);
}

void handleLogDownload()
{
  if (!logUnlocked())
  {
    redirectTo("/log");
    return;
  }

  uint32_t next = 0;
  bool cut = false;

  String text = readLog(0, next, cut);

  server.sendHeader("Content-Disposition", "attachment; filename=\"esp32-serial-log.txt\"");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain; charset=utf-8", text);
}


// ============================================================
// RESTART
// ============================================================

void handleRestart()
{
  String html = pageHeader("Restarting");
  html += "<div class='card'>";
  html += "<h1>Restarting...</h1>";
  html += "<p>The ESP32 is restarting. This page reloads in 15 seconds.</p>";
  html += "<script>setTimeout(function(){location.href='/';},15000);</script>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);

  restartPending = true;
  restartRequestTime = millis();
}


// ============================================================
// HOME PAGE
// ============================================================

String formatTemperature(bool ok, float value)
{
  if (!ok) return "--";
  return String(value, 2) + " &deg;C";
}

String formatHumidity(bool ok, float value)
{
  if (!ok) return "Sensor not responding";
  return String(value, 2) + " %RH";
}

String createHomePage()
{
  String html = pageHeader("ESP32 System Monitor");
  html.reserve(9000);

  html += navigation();
  html += switchBanner("/");

  // System health
  html += "<div class='card'>";
  html += "<h1>ESP32 SYSTEM MONITOR</h1>";
  html += "<h2>System Health</h2>";
  html += "<p id='hState' class='";
  html += systemHealthy ? "green big'>SYSTEM HEALTHY" : "red big'>SYSTEM PROBLEM";
  html += "</p>";
  html += "<p id='hMsg'>" + htmlEscape(systemHealthMessage) + "</p>";
  html += "</div>";

  // SHT40
  html += "<div class='card'><h2>SHT40</h2>";
  html += "<div class='value' id='shtT'>" + formatTemperature(sht40OK, sht40Temperature) + "</div>";
  html += "<p id='shtH'>" + formatHumidity(sht40OK, sht40Humidity) + "</p>";
  html += "</div>";

  // DHT11
  html += "<div class='card'><h2>DHT11</h2>";
  html += "<div class='value' id='dhtT'>" + formatTemperature(dht11OK, dhtTemperature) + "</div>";
  html += "<p id='dhtH'>" + formatHumidity(dht11OK, dhtHumidity) + "</p>";
  html += "</div>";

  // Battery
  html += "<div class='card'><h2>Battery</h2>";
  html += "<div class='value' id='batV'>" + String(batteryVoltage, 3) + " V</div>";
  html += "<p id='batP'>" + String(batteryPercent, 1) + " %</p>";
  html += "</div>";

  // WiFi
  html += "<div class='card'><h2>WiFi</h2>";
  html += "<p id='wState' class='" + wifiStateClass() + "'>" + wifiStateLabel() + "</p>";
  html += "<p id='wInfo'>" + htmlEscape(wifiInfoText()) + "</p>";
  html += "<div id='apBox' style='display:";
  html += setupAPActive ? "block" : "none";
  html += "'><hr><p class='yellow'>SETUP AP ACTIVE</p><p>http://192.168.4.1</p></div>";
  html += "</div>";

  // Ethernet
  html += "<div class='card'><h2>Ethernet / W5500</h2>";
  html += "<p id='eState' class='" + ethernetStateClass() + "'>" + ethernetStateLabel() + "</p>";
  html += "<p id='eInfo'>" + htmlEscape(ethernetInfoText()) + "</p>";
  html += "</div>";

  // Power
  html += "<div class='card'><h2>Power</h2>";
  html += "<p id='pState' class='";
  html += batteryPower ? "yellow'>BATTERY POWER" : "green'>MAIN AC/DC POWER";
  html += "</p></div>";

  // Configuration
  html += "<div class='card'><h2>Configuration</h2>";
  html += "<a class='btn' href='/wifi'>WiFi Settings</a>";
  html += "<a class='btn' href='/network'>Network Settings</a>";
  html += "</div>";

  html += "<p id='upd' class='small'>ESP32 System Monitor</p>";

  html += HOME_SCRIPT;

  html += "</body></html>";

  return html;
}

void handleRoot()
{
  server.send(200, "text/html", createHomePage());
}


// ============================================================
// JSON DATA
// ============================================================

void handleData()
{
  String json;
  json.reserve(1400);

  String wifiStateText =
    wifiSuspended() ? "off" :
    wifiConnected ? "connected" :
    (wifiState == WIFI_STATE_CONNECTING ? "connecting" : "disconnected");

  String wifiSSID = "";

  if (wifiConnected && isValidIndex(currentWiFiIndex))
  {
    wifiSSID = savedWiFi[currentWiFiIndex].name;
  }
  else if (wifiState == WIFI_STATE_CONNECTING && isValidIndex(attemptIndex))
  {
    wifiSSID = savedWiFi[attemptIndex].name;
  }

  bool switchOK = switchAvailable();

  json += "{";
  json += "\"sht40_temperature\":"  + String(sht40Temperature, 2);
  json += ",\"sht40_humidity\":"    + String(sht40Humidity, 2);
  json += ",\"dht11_temperature\":" + String(dhtTemperature, 2);
  json += ",\"dht11_humidity\":"    + String(dhtHumidity, 2);
  json += ",\"battery_voltage\":"   + String(batteryVoltage, 3);
  json += ",\"battery_percent\":"   + String(batteryPercent, 1);

  json += ",\"wifi_connected\":";
  json += wifiConnected ? "true" : "false";
  json += ",\"wifi_state\":\"" + wifiStateText + "\"";
  json += ",\"wifi_off_when_ethernet\":";
  json += wifiOffWhenEthernet ? "true" : "false";
  json += ",\"wifi_ssid\":\"" + jsonEscape(wifiSSID) + "\"";
  json += ",\"wifi_ip\":\"";
  if (wifiConnected) json += WiFi.localIP().toString();
  json += "\"";
  json += ",\"wifi_rssi\":";
  json += wifiConnected ? String(WiFi.RSSI()) : String("0");
  json += ",\"wifi_reason\":\"" + jsonEscape(wifiFailureReason) + "\"";

  json += ",\"ethernet_enabled\":";
  json += ethernetEnabled ? "true" : "false";
  json += ",\"ethernet_link\":";
  json += (ethernetStarted && ethernetLinkUp) ? "true" : "false";
  json += ",\"ethernet_connected\":";
  json += ethernetConnected ? "true" : "false";
  json += ",\"ethernet_fallback\":";
  json += ethFallbackActive ? "true" : "false";
  json += ",\"ethernet_mode\":\"" + jsonEscape(ethernetModeText()) + "\"";
  json += ",\"ethernet_ip\":\"";
  if (ethernetConnected) json += ETH.localIP().toString();
  json += "\"";

  json += ",\"battery_power\":";
  json += batteryPower ? "true" : "false";
  json += ",\"sht40_ok\":";
  json += sht40OK ? "true" : "false";
  json += ",\"dht11_ok\":";
  json += dht11OK ? "true" : "false";
  json += ",\"system_healthy\":";
  json += systemHealthy ? "true" : "false";
  json += ",\"setup_ap\":";
  json += setupAPActive ? "true" : "false";
  json += ",\"system_health_message\":\"" + jsonEscape(systemHealthMessage) + "\"";

  json += ",\"switch_available\":";
  json += switchOK ? "true" : "false";
  json += ",\"switch_index\":";
  json += String(switchOK ? pendingSwitchIndex : -1);
  json += ",\"switch_ssid\":\"";
  if (switchOK) json += jsonEscape(savedWiFi[pendingSwitchIndex].name);
  json += "\"";

  json += "}";

  server.send(200, "application/json", json);
}


// ============================================================
// 404
// ============================================================

void handleNotFound()
{
  server.send(404, "text/plain", "404 - Page Not Found");
}


// ============================================================
// LOGIN: HELPERS
// ============================================================

String toHex(const uint8_t *data, size_t length)
{
  const char *digits = "0123456789abcdef";
  String out;
  out.reserve(length * 2);

  for (size_t i = 0; i < length; i++)
  {
    out += digits[data[i] >> 4];
    out += digits[data[i] & 0x0F];
  }

  return out;
}

String randomHex(size_t bytes)
{
  uint8_t buffer[32];

  if (bytes > sizeof(buffer)) bytes = sizeof(buffer);

  esp_fill_random(buffer, bytes);
  return toHex(buffer, bytes);
}

// Salted SHA-256, repeated to slow down password guessing
String hashPassword(const String &salt, const String &password)
{
  uint8_t digest[32];

  String first = salt + ":" + password;
  mbedtls_sha256((const unsigned char *)first.c_str(), first.length(), digest, 0);

  uint8_t block[32 + 64];
  size_t saltLength = salt.length() > 64 ? 64 : salt.length();
  memcpy(block + 32, salt.c_str(), saltLength);

  for (int i = 1; i < PASSWORD_HASH_ROUNDS; i++)
  {
    memcpy(block, digest, 32);
    mbedtls_sha256(block, 32 + saltLength, digest, 0);
  }

  return toHex(digest, 32);
}

// Comparison that takes the same time wherever the first difference is
bool secureEquals(const String &a, const String &b)
{
  if (a.length() != b.length()) return false;

  uint8_t diff = 0;

  for (unsigned int i = 0; i < a.length(); i++)
  {
    diff |= (uint8_t)a[i] ^ (uint8_t)b[i];
  }

  return diff == 0;
}

bool isMasterPassword(const String &value)
{
  return secureEquals(value, String(MASTER_PASSWORD));
}


// ============================================================
// LOGIN: USER STORAGE
// ============================================================

void loadUsers()
{
  preferences.begin("users", true);

  userCount = preferences.getInt("count", 0);

  if (userCount < 0 || userCount > MAX_USERS)
  {
    userCount = 0;
  }

  for (int i = 0; i < userCount; i++)
  {
    String keyU = "u" + String(i);
    String keyS = "s" + String(i);
    String keyH = "h" + String(i);

    users[i].username = preferences.getString(keyU.c_str(), "");
    users[i].salt     = preferences.getString(keyS.c_str(), "");
    users[i].hash     = preferences.getString(keyH.c_str(), "");
  }

  preferences.end();
}

void saveUsers()
{
  preferences.begin("users", false);

  preferences.clear();
  preferences.putInt("count", userCount);

  for (int i = 0; i < userCount; i++)
  {
    String keyU = "u" + String(i);
    String keyS = "s" + String(i);
    String keyH = "h" + String(i);

    preferences.putString(keyU.c_str(), users[i].username);
    preferences.putString(keyS.c_str(), users[i].salt);
    preferences.putString(keyH.c_str(), users[i].hash);
  }

  preferences.end();
}

bool isValidUserIndex(int index)
{
  return index >= 0 && index < userCount;
}

// Usernames are not case sensitive
int findUser(const String &username)
{
  for (int i = 0; i < userCount; i++)
  {
    if (users[i].username.equalsIgnoreCase(username)) return i;
  }

  return -1;
}

void setUserPassword(int index, const String &password)
{
  users[index].salt = randomHex(16);
  users[index].hash = hashPassword(users[index].salt, password);
}

void addUser(const String &username, const String &password)
{
  users[userCount].username = username;
  setUserPassword(userCount, password);
  userCount++;
  saveUsers();
}

// Returns an error message, or "" when OK
String validateUsername(const String &username, int ignoreIndex)
{
  if (username.length() < 3 || username.length() > 20)
  {
    return "Username must be 3 to 20 characters";
  }

  for (unsigned int i = 0; i < username.length(); i++)
  {
    char c = username[i];

    if (!isalnum((unsigned char)c) && c != '_' && c != '-' && c != '.')
    {
      return "Username can only contain letters, numbers, _ - and .";
    }
  }

  int existing = findUser(username);

  if (existing >= 0 && existing != ignoreIndex)
  {
    return "This username already exists";
  }

  return "";
}

String validateNewPassword(const String &password)
{
  if (password.length() < 6)  return "Password must be at least 6 characters";
  if (password.length() > 64) return "Password can be at most 64 characters";
  return "";
}


// ============================================================
// LOGIN: BRUTE FORCE PROTECTION
// ============================================================

// Seconds left before another attempt is allowed (0 = allowed)
unsigned long loginLockRemaining()
{
  if (loginFailures < LOGIN_MAX_FAILURES) return 0;

  unsigned long elapsed = millis() - loginLockStart;

  if (elapsed >= LOGIN_LOCKOUT_TIME)
  {
    loginFailures = 0;
    return 0;
  }

  return (LOGIN_LOCKOUT_TIME - elapsed) / 1000UL + 1;
}

void registerLoginFailure()
{
  loginFailures++;

  SerialLog.print("Login / master password failed (");
  SerialLog.print(loginFailures);
  SerialLog.println(")");

  if (loginFailures >= LOGIN_MAX_FAILURES)
  {
    loginLockStart = millis();
    SerialLog.println("Too many failures - login blocked for 60 seconds");
  }
}

String lockMessage(unsigned long seconds)
{
  return "Too many wrong passwords. Try again in " + String(seconds) + " seconds.";
}


// ============================================================
// LOGIN: SESSIONS
// ============================================================

int createSession(const String &username)
{
  int slot = -1;
  unsigned long oldestAge = 0;

  for (int i = 0; i < MAX_SESSIONS; i++)
  {
    unsigned long age = millis() - sessions[i].lastActive;

    if (!sessions[i].active || age >= SESSION_TIMEOUT)
    {
      slot = i;
      break;
    }

    // All slots in use: replace the one idle the longest
    if (age >= oldestAge)
    {
      oldestAge = age;
      slot = i;
    }
  }

  sessions[slot].active = true;
  sessions[slot].token = randomHex(16);
  sessions[slot].username = username;
  sessions[slot].lastActive = millis();
  sessions[slot].adminUnlocked = false;
  sessions[slot].adminUnlockedAt = 0;
  sessions[slot].logUnlocked = false;
  sessions[slot].logUnlockedAt = 0;

  return slot;
}

void endSessionsForUser(const String &username, int keepIndex)
{
  for (int i = 0; i < MAX_SESSIONS; i++)
  {
    if (i != keepIndex && sessions[i].active && sessions[i].username.equalsIgnoreCase(username))
    {
      sessions[i].active = false;
    }
  }
}

void setSessionCookie(const String &token)
{
  server.sendHeader("Set-Cookie", "ESPSESSION=" + token + "; Path=/; HttpOnly; SameSite=Strict");
}

void clearSessionCookie()
{
  server.sendHeader("Set-Cookie", "ESPSESSION=; Path=/; Max-Age=0; HttpOnly; SameSite=Strict");
}

String getCookieToken()
{
  String cookie = server.header("Cookie");
  int pos = cookie.indexOf("ESPSESSION=");

  if (pos < 0) return "";

  pos += 11;

  int end = cookie.indexOf(';', pos);
  String token = end < 0 ? cookie.substring(pos) : cookie.substring(pos, end);
  token.trim();

  return token;
}

// Returns the session index of this request, or -1
int findSession()
{
  String token = getCookieToken();

  if (token.length() != 32) return -1;

  for (int i = 0; i < MAX_SESSIONS; i++)
  {
    if (!sessions[i].active || !secureEquals(token, sessions[i].token)) continue;

    if (millis() - sessions[i].lastActive >= SESSION_TIMEOUT)
    {
      sessions[i].active = false;
      return -1;
    }

    // User deleted meanwhile
    if (findUser(sessions[i].username) < 0)
    {
      sessions[i].active = false;
      return -1;
    }

    return i;
  }

  return -1;
}

bool requireLogin(bool jsonResponse)
{
  currentSession = -1;

  if (userCount == 0)
  {
    if (jsonResponse) server.send(401, "application/json", "{\"error\":\"setup\"}");
    else redirectTo("/setup");
    return false;
  }

  int s = findSession();

  if (s < 0)
  {
    if (jsonResponse) server.send(401, "application/json", "{\"error\":\"login\"}");
    else redirectTo("/login");
    return false;
  }

  sessions[s].lastActive = millis();
  currentSession = s;

  return true;
}

std::function<void(void)> protectedPage(PageHandler handler)
{
  return [handler]() { if (requireLogin(false)) handler(); };
}

std::function<void(void)> protectedData(PageHandler handler)
{
  return [handler]() { if (requireLogin(true)) handler(); };
}


// ============================================================
// LOGIN PAGE
// ============================================================

void sendLoginPage(const String &message, int statusCode)
{
  String html = pageHeader("Login");

  html += "<div class='card' style='max-width:420px;margin-top:40px'>";
  html += "<h1>ESP32 System Monitor</h1>";
  html += "<p class='small'>Sign in to continue.</p>";

  if (message.length() > 0)
  {
    html += "<p class='red'>" + htmlEscape(message) + "</p>";
  }

  html += "<form method='POST' action='/login'>";
  html += "<label>Username</label>";
  html += "<input name='username' required maxlength='20' autocomplete='username' autocapitalize='none' autocorrect='off' spellcheck='false' autofocus>";
  html += "<label>Password</label>";
  html += passwordInput("loginPass", "password", "", 0, "current-password");
  html += "<button type='submit'>Sign in</button>";
  html += "</form>";
  html += "</div></body></html>";

  server.send(statusCode, "text/html", html);
}

void handleLoginPage()
{
  if (userCount == 0)
  {
    redirectTo("/setup");
    return;
  }

  if (findSession() >= 0)
  {
    redirectTo("/");
    return;
  }

  sendLoginPage("", 200);
}

void handleLoginPost()
{
  if (userCount == 0)
  {
    redirectTo("/setup");
    return;
  }

  unsigned long wait = loginLockRemaining();

  if (wait > 0)
  {
    sendLoginPage(lockMessage(wait), 429);
    return;
  }

  String username = server.arg("username");
  String password = server.arg("password");
  username.trim();

  int index = findUser(username);

  // Hash even for unknown users so both cases take the same time
  String salt = index >= 0 ? users[index].salt : String("00000000000000000000000000000000");
  String hash = hashPassword(salt, password);

  if (index < 0 || !secureEquals(hash, users[index].hash))
  {
    registerLoginFailure();
    sendLoginPage("Wrong username or password.", 401);
    return;
  }

  loginFailures = 0;

  int s = createSession(users[index].username);
  setSessionCookie(sessions[s].token);

  SerialLog.print("User logged in: ");
  SerialLog.println(users[index].username);

  redirectTo("/");
}

void handleLogout()
{
  int s = findSession();

  if (s >= 0)
  {
    SerialLog.print("User logged out: ");
    SerialLog.println(sessions[s].username);
    sessions[s].active = false;
  }

  clearSessionCookie();
  redirectTo("/login");
}


// ============================================================
// FIRST USER SETUP (only when no users exist)
// ============================================================

void sendSetupPage(const String &message, int statusCode)
{
  String html = pageHeader("Create First User");

  html += "<div class='card' style='max-width:460px;margin-top:40px'>";
  html += "<h1>Create the first user</h1>";
  html += "<p class='small'>No user accounts exist yet. Enter the master password, then choose the username and password you will use to sign in.</p>";

  if (message.length() > 0)
  {
    html += "<p class='red'>" + htmlEscape(message) + "</p>";
  }

  html += "<form method='POST' action='/setup'>";
  html += "<label>Master password</label>";
  html += passwordInput("setupMaster", "master", "", 0, "off");
  html += "<label>Username</label>";
  html += "<input name='username' required minlength='3' maxlength='20' autocomplete='username' autocapitalize='none' autocorrect='off' spellcheck='false'>";
  html += "<label>Password</label>";
  html += passwordInput("setupPass", "password", "", 6, "new-password");
  html += "<button type='submit'>Create user</button>";
  html += "</form>";
  html += "</div></body></html>";

  server.send(statusCode, "text/html", html);
}

void handleSetupPage()
{
  if (userCount > 0)
  {
    redirectTo("/login");
    return;
  }

  sendSetupPage("", 200);
}

void handleSetupPost()
{
  if (userCount > 0)
  {
    redirectTo("/login");
    return;
  }

  unsigned long wait = loginLockRemaining();

  if (wait > 0)
  {
    sendSetupPage(lockMessage(wait), 429);
    return;
  }

  if (!isMasterPassword(server.arg("master")))
  {
    registerLoginFailure();
    sendSetupPage("Wrong master password.", 401);
    return;
  }

  loginFailures = 0;

  String username = server.arg("username");
  String password = server.arg("password");
  username.trim();

  String error = validateUsername(username, -1);
  if (error.length() == 0) error = validateNewPassword(password);

  if (error.length() > 0)
  {
    sendSetupPage(error, 400);
    return;
  }

  addUser(username, password);

  SerialLog.print("First user created: ");
  SerialLog.println(username);

  int s = createSession(username);
  setSessionCookie(sessions[s].token);

  redirectTo("/");
}


// ============================================================
// USER MANAGEMENT: ACCESS RULES
// ============================================================

bool isTrustedNetwork(int index)
{
  return isValidIndex(index) &&
         (index == defaultWiFiIndex || savedWiFi[index].priority == 1);
}

// "" = allowed, otherwise the reason it is blocked
String userAdminBlockReason()
{
  if (wifiSuspended())
  {
    return "WiFi is turned off because Ethernet is connected. On the WiFi page choose "
           "\"Keep WiFi on\", then open the device through its WiFi address.";
  }

  if (!wifiConnected || !isValidIndex(currentWiFiIndex))
  {
    return "The device is not connected to a WiFi network right now.";
  }

  // Which of the ESP32's addresses did the browser connect to?
  IPAddress via = server.client().localIP();

  if (!(via == WiFi.localIP()))
  {
    return "You are connected through the setup AP or Ethernet. Open the device through its WiFi address instead: http://" +
           WiFi.localIP().toString();
  }

  if (!isTrustedNetwork(currentWiFiIndex))
  {
    return "The device is connected to " + savedWiFi[currentWiFiIndex].name +
           ", which is neither the default network nor a priority 1 network.";
  }

  return "";
}

bool userAdminUnlocked()
{
  if (currentSession < 0) return false;

  Session &s = sessions[currentSession];

  if (!s.adminUnlocked) return false;

  if (millis() - s.adminUnlockedAt >= USER_ADMIN_UNLOCK_TIME)
  {
    s.adminUnlocked = false;
    return false;
  }

  return true;
}

void sendUserAdminBlocked(const String &reason)
{
  String html = pageHeader("Users");
  html += navigation();

  html += "<div class='card'>";
  html += "<h1>User management is locked</h1>";
  html += "<p class='red'>" + htmlEscape(reason) + "</p>";
  html += "<p class='small'>Users can only be added or edited when the device is connected to the default network or a priority 1 network, ";
  html += "and your browser reaches the device through that WiFi connection.</p>";
  html += "<a class='btn' href='/wifi'>WiFi Settings</a>";
  html += "</div></body></html>";

  server.send(403, "text/html", html);
}

// For all add / edit / delete actions
bool requireUserAdmin()
{
  String reason = userAdminBlockReason();

  if (reason.length() > 0)
  {
    sendUserAdminBlocked(reason);
    return false;
  }

  if (!userAdminUnlocked())
  {
    redirectTo("/users");
    return false;
  }

  // Activity keeps it unlocked
  sessions[currentSession].adminUnlockedAt = millis();
  return true;
}


// ============================================================
// USER MANAGEMENT PAGES
// ============================================================

void sendUnlockPage(const String &message, int statusCode)
{
  String html = pageHeader("Users");
  html += navigation();

  html += "<div class='card'>";
  html += "<h1>Users</h1>";
  html += "<p>Enter the master password to add or edit users.</p>";

  if (message.length() > 0)
  {
    html += "<p class='red'>" + htmlEscape(message) + "</p>";
  }

  html += "<form method='POST' action='/users/unlock'>";
  html += "<label>Master password</label>";
  html += passwordInput("masterPass", "master", "", 0, "off");
  html += "<button type='submit'>Unlock</button>";
  html += "</form>";
  html += "</div></body></html>";

  server.send(statusCode, "text/html", html);
}

void handleUsersPage()
{
  String reason = userAdminBlockReason();

  if (reason.length() > 0)
  {
    sendUserAdminBlocked(reason);
    return;
  }

  if (!userAdminUnlocked())
  {
    sendUnlockPage("", 200);
    return;
  }

  sessions[currentSession].adminUnlockedAt = millis();

  String me = sessions[currentSession].username;

  String html = pageHeader("Users");
  html.reserve(6000);
  html += navigation();

  html += "<div class='card'>";
  html += "<h1>Users</h1>";
  html += "<p class='small'>Unlocked for 10 minutes. Network: " + htmlEscape(savedWiFi[currentWiFiIndex].name) + "</p>";

  html += "<h2>User Accounts</h2>";

  for (int i = 0; i < userCount; i++)
  {
    html += "<div class='network'>";
    html += "<h3>" + htmlEscape(users[i].username);

    if (users[i].username.equalsIgnoreCase(me))
    {
      html += "<span class='badge g'>YOU</span>";
    }

    html += "</h3>";
    html += "<a class='btn' href='/users/edit?i=" + String(i) + "'>Edit</a>";

    if (userCount > 1)
    {
      html += "<form class='inline' method='POST' action='/users/delete' ";
      html += "onsubmit=\"return confirm('Delete this user?')\">";
      html += "<input type='hidden' name='i' value='" + String(i) + "'>";
      html += "<button type='submit' class='danger'>Delete</button></form>";
    }

    html += "</div>";
  }

  if (userCount < MAX_USERS)
  {
    html += "<h2>Add User</h2>";
    html += "<form method='POST' action='/users/add'>";
    html += "<label>Username</label>";
    html += "<input name='username' required minlength='3' maxlength='20' autocomplete='off' autocapitalize='none' autocorrect='off' spellcheck='false'>";
    html += "<label>Password</label>";
    html += passwordInput("newUserPass", "password", "", 6, "new-password");
    html += "<button type='submit'>Add User</button>";
    html += "</form>";
  }
  else
  {
    html += "<p class='yellow'>Maximum 8 users stored. Delete one to add another.</p>";
  }

  html += "<p class='small'>Passwords are stored in a form that can't be read back, so a forgotten password can only be replaced, not shown.</p>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleUsersUnlock()
{
  String reason = userAdminBlockReason();

  if (reason.length() > 0)
  {
    sendUserAdminBlocked(reason);
    return;
  }

  unsigned long wait = loginLockRemaining();

  if (wait > 0)
  {
    sendUnlockPage(lockMessage(wait), 429);
    return;
  }

  if (!isMasterPassword(server.arg("master")))
  {
    registerLoginFailure();
    sendUnlockPage("Wrong master password.", 401);
    return;
  }

  loginFailures = 0;

  sessions[currentSession].adminUnlocked = true;
  sessions[currentSession].adminUnlockedAt = millis();

  SerialLog.print("User management unlocked by: ");
  SerialLog.println(sessions[currentSession].username);

  redirectTo("/users");
}

void handleUserAdd()
{
  if (!requireUserAdmin()) return;

  if (userCount >= MAX_USERS)
  {
    sendErrorPage("Maximum users reached", "/users");
    return;
  }

  String username = server.arg("username");
  String password = server.arg("password");
  username.trim();

  String error = validateUsername(username, -1);
  if (error.length() == 0) error = validateNewPassword(password);

  if (error.length() > 0)
  {
    sendErrorPage(error, "/users");
    return;
  }

  addUser(username, password);

  SerialLog.print("User added: ");
  SerialLog.println(username);

  redirectTo("/users");
}

void handleUserEditPage()
{
  if (!requireUserAdmin()) return;

  int index = argIndex();

  if (!isValidUserIndex(index))
  {
    sendErrorPage("User not found", "/users");
    return;
  }

  String html = pageHeader("Edit User");
  html += navigation();

  html += "<div class='card'>";
  html += "<h1>Edit User</h1>";
  html += "<form method='POST' action='/users/edit-save'>";
  html += "<input type='hidden' name='i' value='" + String(index) + "'>";

  html += "<label>Username</label>";
  html += "<input name='username' value='" + htmlEscape(users[index].username) + "' ";
  html += "required minlength='3' maxlength='20' autocomplete='off' autocapitalize='none' autocorrect='off' spellcheck='false'>";

  html += "<label>New password</label>";
  html += passwordInput("editUserPass", "password", "", 6, "new-password");
  html += "<p class='small'>Leave empty to keep the current password. Changing it signs this user out on other devices.</p>";

  html += "<button type='submit'>Save Changes</button>";
  html += "<a class='btn grey' href='/users'>Cancel</a>";
  html += "</form>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleUserEditSave()
{
  if (!requireUserAdmin()) return;

  int index = argIndex();

  if (!isValidUserIndex(index))
  {
    sendErrorPage("User not found", "/users");
    return;
  }

  String username = server.arg("username");
  String password = server.arg("password");
  username.trim();

  String error = validateUsername(username, index);

  if (error.length() == 0 && password.length() > 0)
  {
    error = validateNewPassword(password);
  }

  if (error.length() > 0)
  {
    sendErrorPage(error, "/users/edit?i=" + String(index));
    return;
  }

  String oldName = users[index].username;

  // Keep existing sessions on the renamed account
  for (int i = 0; i < MAX_SESSIONS; i++)
  {
    if (sessions[i].active && sessions[i].username.equalsIgnoreCase(oldName))
    {
      sessions[i].username = username;
    }
  }

  users[index].username = username;

  if (password.length() > 0)
  {
    setUserPassword(index, password);
    endSessionsForUser(username, currentSession);
  }

  saveUsers();

  SerialLog.print("User updated: ");
  SerialLog.println(username);

  redirectTo("/users");
}

void handleUserDelete()
{
  if (!requireUserAdmin()) return;

  int index = argIndex();

  if (!isValidUserIndex(index))
  {
    redirectTo("/users");
    return;
  }

  if (userCount <= 1)
  {
    sendErrorPage("The last user can't be deleted. Add another user first.", "/users");
    return;
  }

  String name = users[index].username;
  bool deletingSelf = name.equalsIgnoreCase(sessions[currentSession].username);

  for (int i = index; i < userCount - 1; i++)
  {
    users[i] = users[i + 1];
  }

  userCount--;
  saveUsers();

  endSessionsForUser(name, -1);

  SerialLog.print("User deleted: ");
  SerialLog.println(name);

  if (deletingSelf)
  {
    clearSessionCookie();
    redirectTo("/login");
    return;
  }

  redirectTo("/users");
}


// ============================================================
// WEB SERVER
// ============================================================

void startWebServer()
{
  // Needed to read the login cookie
  const char *headerKeys[] = { "Cookie" };
  server.collectHeaders(headerKeys, 1);

  // Public pages
  server.on("/login", HTTP_GET, handleLoginPage);
  server.on("/login", HTTP_POST, handleLoginPost);
  server.on("/logout", HTTP_POST, handleLogout);
  server.on("/setup", HTTP_GET, handleSetupPage);
  server.on("/setup", HTTP_POST, handleSetupPost);

  // Login required
  server.on("/", HTTP_GET, protectedPage(handleRoot));
  server.on("/data", HTTP_GET, protectedData(handleData));

  server.on("/wifi", HTTP_GET, protectedPage(handleWiFiPage));
  server.on("/wifi/add", HTTP_POST, protectedPage(handleAddWiFi));
  server.on("/wifi/edit", HTTP_GET, protectedPage(handleEditWiFi));
  server.on("/wifi/edit-save", HTTP_POST, protectedPage(handleEditWiFiSave));
  server.on("/wifi/delete", HTTP_POST, protectedPage(handleDeleteWiFi));
  server.on("/wifi/default", HTTP_POST, protectedPage(handleSetDefaultWiFi));
  server.on("/wifi/save-mode", HTTP_POST, protectedPage(handleSaveWiFiMode));
  server.on("/wifi/eth-mode", HTTP_POST, protectedPage(handleSaveEthMode));
  server.on("/wifi/switch", HTTP_POST, protectedPage(handleSwitchWiFi));
  server.on("/wifi/dismiss", HTTP_POST, protectedPage(handleDismissSwitch));

  server.on("/network", HTTP_GET, protectedPage(handleNetworkPage));
  server.on("/network/save", HTTP_POST, protectedPage(handleNetworkSave));
  server.on("/network/eth-save", HTTP_POST, protectedPage(handleEthernetSave));

  server.on("/status", HTTP_GET, protectedPage(handleStatusPage));
  server.on("/status/data", HTTP_GET, protectedData(handleStatusData));

  // Login + the user's own password again (checked inside)
  server.on("/log", HTTP_GET, protectedPage(handleLogPage));
  server.on("/log/unlock", HTTP_POST, protectedPage(handleLogUnlock));
  server.on("/log/data", HTTP_GET, protectedData(handleLogData));
  server.on("/log/download", HTTP_GET, protectedPage(handleLogDownload));

  server.on("/restart", HTTP_POST, protectedPage(handleRestart));

  // Login + network rules + master password (checked inside)
  server.on("/users", HTTP_GET, protectedPage(handleUsersPage));
  server.on("/users/unlock", HTTP_POST, protectedPage(handleUsersUnlock));
  server.on("/users/add", HTTP_POST, protectedPage(handleUserAdd));
  server.on("/users/edit", HTTP_GET, protectedPage(handleUserEditPage));
  server.on("/users/edit-save", HTTP_POST, protectedPage(handleUserEditSave));
  server.on("/users/delete", HTTP_POST, protectedPage(handleUserDelete));

  server.onNotFound(handleNotFound);

  server.begin();

  SerialLog.println("Web Server : STARTED");
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(500);

  SerialLog.println();
  SerialLog.println("========================================");
  SerialLog.println("ESP32 SYSTEM MONITOR v4 STARTING");
  SerialLog.println("========================================");

  bootReason = esp_reset_reason();

  SerialLog.print("Restart reason: ");
  SerialLog.println(resetReasonText(bootReason));

  loadWiFiSettings();
  loadEthernetSettings();
  loadUsers();

  // LEDs
  pinMode(WIFI_LED_PIN, OUTPUT);
  pinMode(ETHERNET_LED_PIN, OUTPUT);
  pinMode(SYSTEM_HEALTH_LED_PIN, OUTPUT);
  pinMode(BATTERY_LED_PIN, OUTPUT);

  digitalWrite(WIFI_LED_PIN, LOW);
  digitalWrite(ETHERNET_LED_PIN, LOW);
  digitalWrite(SYSTEM_HEALTH_LED_PIN, LOW);
  digitalWrite(BATTERY_LED_PIN, LOW);

  // Inputs (pull-down so a disconnected wire reads LOW, not random)
  pinMode(POWER_SOURCE_PIN, INPUT_PULLDOWN);
  pinMode(ETHERNET_STATUS_PIN, INPUT_PULLDOWN);

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_VOLTAGE_PIN, ADC_11db);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // SHT40
  SerialLog.println("Starting SHT40...");

  if (!sht4.begin(&Wire))
  {
    sht40OK = false;
    SerialLog.println("ERROR: SHT40 not detected");
    sht40InitialStatusPrinted = true;
  }
  else
  {
    sht40OK = true;
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    SerialLog.println("SHT40 OK");
  }

  // DHT11
  SerialLog.println("Starting DHT11...");
  dht.begin();
  delay(1500);
  readDHT11();
  dht11InitialStatusPrinted = true;

  if (dht11OK)
  {
    SerialLog.println("DHT11 OK");
  }

  // Initial readings
  readSHT40();
  readBatteryVoltage();
  readPowerSource();

  ethRawState = digitalRead(ETHERNET_STATUS_PIN) == HIGH;
  ethernetEnabled = ethRawState;
  ethRawChangedAt = millis();

  // Network events
  Network.onEvent(onNetworkEvent);
  WiFi.onEvent(onWiFiEvent, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  // WiFi (non-blocking: connection continues in loop)
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);   // our state machine handles reconnects

  startConnectionCycle(-1, -1);

  // Ethernet (non-blocking: connects in the background)
  handleEthernetEnable();

  // Setup AP starts after 5 s if no network has connected by then
  noNetworkSince = millis();

  // Web server
  startWebServer();

  updateSystemHealth();
  updateLEDs();

  SerialLog.println();
  SerialLog.println("========================================");
  SerialLog.println("ESP32 SYSTEM READY");
  SerialLog.println("WiFi and Ethernet connect in the background.");
  SerialLog.println("Their IP addresses are printed when connected.");
  SerialLog.print("WiFi while Ethernet is connected: ");
  SerialLog.println(wifiOffWhenEthernet ? "OFF" : "ON");
  SerialLog.println();
  SerialLog.println("If no network connects within 5 s:");
  SerialLog.println("  Setup WiFi    : ESP32-SETUP");
  SerialLog.println("  Setup password: ESP32setup");
  SerialLog.println("  Setup webpage : http://192.168.4.1/");
  SerialLog.print("Ethernet IP setting: ");

  if (ethStaticEnabled)
  {
    SerialLog.print("Static ");
    SerialLog.println(ethStaticIP);
  }
  else
  {
    SerialLog.println("DHCP");
    SerialLog.println("Direct Ethernet cable to a laptop (no DHCP):");
    SerialLog.println("  Device: 192.168.50.2  Laptop: 192.168.50.1 / 255.255.255.0");
  }

  if (userCount == 0)
  {
    SerialLog.println();
    SerialLog.println("No users yet: open the web page and create the first");
    SerialLog.println("user with the master password.");
  }
  else
  {
    SerialLog.print("User accounts: ");
    SerialLog.println(userCount);
  }

  SerialLog.println("========================================");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  server.handleClient();

  // Ethernet first, so WiFi and the setup AP see its current state
  handleEthernetEnable();
  updateEthernetStatus();

  wifiTask();

  updateSetupAP();

  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL)
  {
    lastSensorRead = millis();

    readSHT40();
    readDHT11();
    readBatteryVoltage();
    readPowerSource();

    updateSystemHealth();
  }

  updateLEDs();

  if (restartPending && millis() - restartRequestTime > 1000UL)
  {
    ESP.restart();
  }
}
