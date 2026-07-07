/*
  Rigol Speaking Multimeter - ESP32-C3 foot pedal trigger
  =========================================================
  Wakes on a foot-pedal press, joins Wi-Fi, and sends a UDP "trigger"
  packet to the PC running rigol_speaking_multimeter.py, which speaks the
  current multimeter reading (same effect as pressing F8 on the PC). The
  PC-side UDP listener ignores packet contents - any datagram on the
  configured port triggers a speak - so this firmware just needs to get
  one empty packet there per press.

  ------------------------------------------------------------------------
  HARDWARE
  ------------------------------------------------------------------------
  Board: "ESP32-C3 Super Mini" - small USB-C dev board with a BOOT button
  wired to GPIO9 and a RESET button. Pedal switch wiring: normally-open,
  one side to BUTTON_PIN (below), the other side to GND. The pin uses the
  internal pull-up, so it reads LOW while the pedal is pressed and HIGH
  when released.

  BUTTON_PIN defaults to GPIO4: it's one of the ESP32-C3's RTC-capable
  pins (GPIO0-5), which is required for it to work as a deep-sleep wake
  source, and it is *not* one of the chip's three strapping pins
  (GPIO2, GPIO8, GPIO9) that affect boot mode - so it's safe to use
  without any chance of interfering with flashing/boot.

  ------------------------------------------------------------------------
  FIRST-TIME SETUP (Wi-Fi only)
  ------------------------------------------------------------------------
  On first power-up (or any time it can't connect with previously saved
  Wi-Fi credentials), this firmware opens its own temporary Wi-Fi access
  point called "RigolPedal-Setup". Connect to that from a phone or laptop;
  a setup page should open automatically (or browse to 192.168.4.1). Pick
  your Wi-Fi network, enter its password, and (optionally) check the
  "Trigger UDP port" field - it defaults to 47269, matching the Python
  app's own default --trigger-port, so you only need to touch it if you
  changed that on the PC side. Save, and the pedal reboots onto your real
  network.

  There's deliberately no "target PC IP address" to configure: the pedal
  broadcasts its trigger to the whole local network (see sendTrigger()
  below) instead of addressing one specific PC, so nothing needs
  re-entering if the PC's IP ever changes via DHCP, and there's nothing to
  look up during setup.

  To force re-entering setup mode later (new Wi-Fi network, different
  trigger port), hold the foot pedal itself down for 3+ seconds right as
  it wakes (instead of a quick tap). This deliberately does NOT use the
  onboard BOOT button (GPIO9): that pin is one of the ESP32-C3's boot-mode
  strapping pins, and holding it low during a reset/power-up diverts the
  chip straight into the ROM's serial download bootloader instead of
  running this firmware at all - confirmed by real hardware testing (the
  setup AP never appeared because the app never started). GPIO4 has no
  such conflict, so reusing the pedal switch for the long-press is the
  safe way to do this.

  ------------------------------------------------------------------------
  POWER / SLEEP DESIGN
  ------------------------------------------------------------------------
  The pedal spends almost all of its time in deep sleep (microamps),
  woken only by the pedal switch pulling BUTTON_PIN low. On wake it
  reconnects to Wi-Fi, sends a trigger packet immediately, then stays
  awake ("lingers") for lingerSeconds (default 5 minutes, changeable on
  the live status page below) so a burst of presses during one probing
  session doesn't pay the Wi-Fi-reconnect cost every time - each press
  resets the countdown. Once that time passes with no further presses (or
  status-page activity), it goes back to deep sleep.

  ------------------------------------------------------------------------
  LIVE STATUS / SETTINGS PAGE (while awake only)
  ------------------------------------------------------------------------
  While the pedal is awake and joined to your Wi-Fi, it runs a small web
  server at its normal network IP (check your router's client list, or
  the serial monitor) showing current status - IP, Wi-Fi network, time
  since last trigger, seconds until it sleeps, and a near-live view of the
  raw pedal GPIO state (polled every 250ms via a small JS/fetch loop
  against /pin) so you can watch it flip PRESSED/RELEASED in real time to
  verify wiring and contact bounce - plus a "send test trigger" button and
  a form to change the trigger UDP port and the stay-awake duration,
  applied immediately and saved to flash. Loading the page or using either
  button resets the sleep countdown; the /pin polling deliberately does
  NOT, so just leaving the page open to watch the pin doesn't itself keep
  the pedal permanently awake.
  This page is only reachable while awake; once it deep-sleeps it drops
  off the network entirely until the next press. There's no way to wake
  it remotely - a physical press always comes first.

  Not yet implemented (possible future optimization): caching the Wi-Fi
  AP's channel/BSSID in RTC memory across sleep cycles to speed up
  reconnection after deep sleep. Left out of this first version to keep
  it simple and easier to verify; add it later if reconnect time after a
  long sleep proves too slow in practice.

  ------------------------------------------------------------------------
  STATUS
  ------------------------------------------------------------------------
  Confirmed working end-to-end on real hardware (an "ESP32-C3 Super Mini"
  board): deep sleep, GPIO4 wake, the Wi-Fi setup portal, the subnet-
  broadcast UDP trigger, and a real pedal press making
  rigol_speaking_multimeter.py speak on the PC. The one gotcha that took a
  while to track down wasn't in this firmware at all - it was Windows
  Firewall silently dropping the incoming UDP packet on the PC side (see
  that project's ensure_firewall_rule() / README for the fix).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <Preferences.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

// ---- Pins ----
#define BUTTON_PIN 4        // pedal switch: this pin -> GND, internal pull-up
// NOTE: deliberately NOT using GPIO9 (onboard BOOT button) for any in-app
// logic - it's a boot-mode strapping pin on the ESP32-C3, and holding it
// low during reset/power-up sends the chip into the ROM download
// bootloader instead of running this firmware (confirmed on real
// hardware: the setup AP never came up because the app never started).
// The long-press-to-force-setup gesture below uses BUTTON_PIN instead,
// which has no such conflict.
#define LED_PIN 8            // onboard plain blue LED (single color, no RGB)
// GPIO8 is ALSO one of the ESP32-C3's strapping pins, same family as GPIO9
// above - but that only matters for a pin's level at the instant of
// reset/power-up. This code only ever *writes* to GPIO8 well after boot
// has finished, which doesn't touch strapping at all, so there's no
// conflict here (unlike reading GPIO9 as an input during boot, which
// would be a problem).
//
// Polarity: unconfirmed which way this specific board wires it (LED to
// 3.3V + GPIO sinks it low = active-low, or LED to GND + GPIO sources it
// high = active-high - both designs are common on cheap dev boards). If
// the LED behaves backwards from what's described below (on when it
// should be off, etc.), just flip this one constant.
#define LED_ACTIVE_LOW true

// ---- Timing ----
#define FORCE_SETUP_HOLD_MS 3000UL       // how long to hold the pedal to force re-setup
#define DEFAULT_LINGER_S 300             // default: stay awake 5 min after a press
#define MIN_LINGER_S 10                  // clamp for the live-settings page
#define MAX_LINGER_S 3600
#define DEBOUNCE_MS 250UL                // minimum time between repeat triggers
#define CONFIG_PORTAL_TIMEOUT_S 180      // give up on an unconfigured portal after this
#define STATUS_SERVER_PORT 80

Preferences prefs;
WiFiUDP udp;
WiFiManager wm;
WebServer statusServer(STATUS_SERVER_PORT);

// Computed once after connecting (see setup()) as the actual subnet
// broadcast address (e.g. 10.0.0.255) rather than hardcoded 255.255.255.255.
// Both are technically "broadcast", but 255.255.255.255 is the "limited
// broadcast" address, and some Wi-Fi routers/APs are known to not flood
// that one out to all wireless clients as reliably as a proper
// subnet-directed broadcast - a common real-world cause of "the ESP32
// sends it but nothing ever receives it" reports. Falls back to
// 255.255.255.255 if the subnet mask isn't available yet.
IPAddress broadcastAddr(255, 255, 255, 255);

// Must match the Python app's own default --trigger-port unless you've
// changed that there, in which case set the same value in the setup
// portal's "Trigger UDP port" field.
#define DEFAULT_TRIGGER_PORT 47269

// Overrides WiFiManager's default button colors (blue background, white
// text) with yellow/black. WiFiManager's own default button CSS is:
//   button,input[type='button'],input[type='submit']{...background-color:
//   #1fa3ec;color:#fff...}
// - matching that same selector here, with !important to guarantee this
// wins regardless of the two stylesheets' exact order in <head>, is what
// overrides it. The red "Erase" button (class .D) is deliberately left
// alone - it's a destructive action and keeping it visually distinct from
// the ordinary yellow buttons seems worth keeping.
// setCustomBodyHeader() isn't available in the WiFiManager release
// PlatformIO resolves here (confirmed by an actual build error: "no member
// named setCustomBodyHeader"), only the older setCustomHeadElement(). So
// both the button CSS and the logo <img> are injected there instead, as
// one combined string. That still displays the image correctly: an <img>
// tag is flow content, and per the HTML5 parsing spec, a browser
// encountering it while still "in head" pops out to "in body" and
// re-processes it there - a well-known, reliable trick for adding a logo
// to WiFiManager pages without needing a newer library version.
const char CUSTOM_PORTAL_CSS[] =
    "<style>"
    "button,input[type='button'],input[type='submit']{"
    "background-color:#ffdd00 !important;"
    "color:#000 !important;"
    "}"
    "</style>"
    "<div style='text-align:center;padding:6px 0 2px;'>"
    "<img src='data:image/png;base64,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'"
    " style='max-width:180px;height:auto;display:block;margin:0 auto;'>"
    "</div>";

uint16_t targetPort = DEFAULT_TRIGGER_PORT;
uint16_t lingerSeconds = DEFAULT_LINGER_S;
bool shouldSaveParams = false;

// Tracks the awake window: reset by every pedal press AND every request to
// the status page below, so either one keeps the pedal awake. Also used to
// show "last trigger Ns ago" / "sleeping in Ns" on the status page.
unsigned long lastActivityMs = 0;
unsigned long lastTriggerMs = 0;
bool hasTriggeredThisWake = false;

// WiFiManagerParameter objects must outlive the wm.autoConnect() /
// wm.startConfigPortal() call that uses them; allocating them with `new`
// in setup() (after loadConfig() has populated the defaults it should
// show) and never freeing them is the simplest way to guarantee that on a
// microcontroller with no dynamic teardown to worry about.
WiFiManagerParameter *customPortParam = nullptr;

void loadConfig() {
  prefs.begin("pedal", true);
  targetPort = prefs.getUShort("target_port", DEFAULT_TRIGGER_PORT);
  lingerSeconds = prefs.getUShort("linger_s", DEFAULT_LINGER_S);
  prefs.end();
}

void saveConfig(uint16_t port, uint16_t lingerS) {
  prefs.begin("pedal", false);
  prefs.putUShort("target_port", port);
  prefs.putUShort("linger_s", lingerS);
  prefs.end();
}

// Plain free function (not a lambda) so it converts cleanly to whatever
// callback type WiFiManager's setSaveParamsCallback expects.
void saveParamsCallback() {
  shouldSaveParams = true;
}

unsigned long lingerMs() {
  return (unsigned long)lingerSeconds * 1000UL;
}

// ---- Status LED ----
// Off during Wi-Fi setup/connect. Solid on once connected = "Wi-Fi link
// up". Blinks off briefly on each subsequent pedal press during the awake
// window (not the initial wake-up press itself, since that's implicit in
// the LED turning on), then returns to solid on. Off again before deep
// sleep.
void ledOn() {
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW : HIGH);
}

void ledOff() {
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW);
}

void ledBlinkForPress() {
  ledOff();
  delay(200);
  ledOn();
}

void sendTrigger() {
  // Broadcasts to the whole local network rather than a specific PC
  // address - no "target IP" needs configuring, and it keeps working if
  // the PC's DHCP-assigned IP ever changes. The Python app's UDP listener
  // binds 0.0.0.0 and ignores packet contents, so a broadcast arrives and
  // triggers a speak exactly like a directed packet would.
  udp.beginPacket(broadcastAddr, targetPort);
  udp.write((uint8_t)0);
  int ok = udp.endPacket();
  // endPacket() only confirms the ESP32 handed the packet to the network -
  // it says nothing about whether the router forwarded it or the PC
  // actually received it. Still, ok==0 here means it failed at the source
  // and never had a chance, which narrows things down a lot if that's
  // ever what's happening.
  Serial.println("[pedal] sendTrigger to " + broadcastAddr.toString() + ":" +
                  String(targetPort) + " -> udp.endPacket()=" + String(ok));
}

// ---- Live status/settings page ----
// Only reachable while the pedal is awake (joined Wi-Fi, in the linger
// window after a press) - once it goes back to deep sleep it drops off
// the network entirely, same as unplugging it. There's deliberately no
// way to wake it remotely over Wi-Fi; you always need one physical press
// first. Reuses the same yellow/black styling as the setup portal.
const char STATUS_PAGE_CSS[] =
    "<style>body{font-family:sans-serif;max-width:420px;margin:20px auto;"
    "padding:0 12px;}button,input[type=submit]{background-color:#ffdd00;"
    "color:#000;border:1px solid #333;padding:8px 14px;font-size:1em;"
    "border-radius:4px;}label{display:block;margin-top:10px;}"
    "input[type=number]{width:100%;padding:6px;box-sizing:border-box;}"
    "hr{margin:20px 0;}"
    "#pinstate{display:inline-block;min-width:90px;font-weight:bold;}"
    "</style>";

// Same logo image used on the WiFiManager setup portal - duplicated here
// as its own literal (rather than shared/reused) for simplicity; the
// extra ~4.8KB of flash is not worth the added complexity of sharing one
// copy across two otherwise-unrelated page builders.
const char STATUS_PAGE_LOGO[] =
    "<div style='text-align:center;padding:6px 0 2px;'>"
    "<img src='data:image/png;base64,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'"
    " style='max-width:150px;height:auto;display:block;margin:0 auto;'>"
    "</div>";

void handleStatusRoot() {
  lastActivityMs = millis(); // just loading the page counts as activity,
                              // so it always shows the full window remaining
  long remainingS = (long)(lingerMs() / 1000);

  String lastTrigger = hasTriggeredThisWake
      ? String((millis() - lastTriggerMs) / 1000) + "s ago"
      : "not yet this session";

  String page = "<!DOCTYPE html><html><head><meta name='viewport' "
      "content='width=device-width,initial-scale=1'>";
  page += STATUS_PAGE_CSS;
  page += "</head><body>";
  page += STATUS_PAGE_LOGO;
  page += "<h2>Rigol foot pedal</h2>";
  page += "<p><b>IP:</b> " + WiFi.localIP().toString() + "<br>";
  page += "<b>Wi-Fi network:</b> " + WiFi.SSID() + "<br>";
  page += "<b>Last trigger:</b> " + lastTrigger + "<br>";
  page += "<b>Sleeping in:</b> " + String(remainingS) + "s (any activity here resets this)</p>";
  page += "<p><b>Pedal input right now:</b> <span id='pinstate'>"
      + String(digitalRead(BUTTON_PIN) == LOW ? "PRESSED" : "RELEASED")
      + "</span><br><span style='color:#666;font-size:0.85em'>(updates live - "
      "press/release the pedal to check wiring and contact bounce)</span></p>";
  page += "<script>setInterval(function(){"
      "fetch('/pin').then(function(r){return r.text();}).then(function(t){"
      "document.getElementById('pinstate').textContent = t;"
      "});"
      "}, 250);</script>";
  page += "<form method='POST' action='/trigger'>"
      "<button type='submit'>Send test trigger now</button></form>";
  page += "<hr><h3>Settings</h3>";
  page += "<form method='POST' action='/save'>";
  page += "<label>Trigger UDP port<input type='number' min='1' max='65535' "
      "name='port' value='" + String(targetPort) + "'></label>";
  page += "<label>Stay-awake time after a press, seconds (" +
      String(MIN_LINGER_S) + "-" + String(MAX_LINGER_S) + ")"
      "<input type='number' min='" + String(MIN_LINGER_S) + "' max='" +
      String(MAX_LINGER_S) + "' name='linger' value='" + String(lingerSeconds) + "'></label>";
  page += "<p><input type='submit' value='Save settings'></p>";
  page += "</form>";
  page += "<p style='color:#666;font-size:0.85em'>To change Wi-Fi network, "
      "hold the pedal down for 3+ seconds right as it wakes.</p>";
  page += "</body></html>";

  statusServer.send(200, "text/html", page);
}

void handleStatusSave() {
  lastActivityMs = millis();
  if (statusServer.hasArg("port")) {
    int p = statusServer.arg("port").toInt();
    if (p > 0 && p <= 65535) targetPort = (uint16_t)p;
  }
  if (statusServer.hasArg("linger")) {
    int s = statusServer.arg("linger").toInt();
    if (s >= MIN_LINGER_S && s <= MAX_LINGER_S) lingerSeconds = (uint16_t)s;
  }
  saveConfig(targetPort, lingerSeconds);
  statusServer.sendHeader("Location", "/");
  statusServer.send(302, "text/plain", "");
}

// Polled by the status page's JS every 250ms to show a near-live pedal
// state. Deliberately does NOT reset lastActivityMs - leaving the page
// open just to watch this shouldn't itself prevent the pedal from ever
// sleeping; only loading the page fresh or using the buttons/form does.
void handleStatusPin() {
  statusServer.send(200, "text/plain", digitalRead(BUTTON_PIN) == LOW ? "PRESSED" : "RELEASED");
}

void handleStatusTrigger() {
  sendTrigger();
  lastActivityMs = millis();
  lastTriggerMs = millis();
  hasTriggeredThisWake = true;
  statusServer.sendHeader("Location", "/");
  statusServer.send(302, "text/plain", "");
}

void goToSleep() {
  ledOff();
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // pinMode()'s pull-up alone isn't guaranteed to survive deep sleep, so
  // this explicitly (re-)asserts it via the IDF GPIO driver right before
  // sleeping. On the original ESP32 this would need the separate
  // rtc_gpio_* driver (a distinct RTC IO peripheral) - the ESP32-C3's
  // simpler GPIO/RTC architecture doesn't expose that API (confirmed by
  // build error: 'rtc_gpio_pullup_en' not declared, compiler suggested
  // gpio_pullup_en instead), so plain gpio_pullup_en/gpio_pulldown_dis
  // is the correct call here.
  gpio_pullup_en((gpio_num_t)BUTTON_PIN);
  gpio_pulldown_dis((gpio_num_t)BUTTON_PIN);

  // ESP32-C3 uses esp_deep_sleep_enable_gpio_wakeup() (not the classic
  // ext0/ext1 API, which is specific to the original ESP32's RTC
  // controller hardware). Wake when BUTTON_PIN reads low, i.e. pedal
  // pressed. Confirmed (2026-07) that GPIO4 is a valid wake-capable pin
  // for this - GPIO0/GPIO1 are RTC-capable but do NOT reliably work as
  // ESP32-C3 deep-sleep GPIO wake sources (known Arduino-ESP32 core
  // limitation); only GPIO2-5 do.
  //
  // NOTE - unverified against the exact ESP-IDF version PlatformIO
  // resolves at build time: this call and the ESP_GPIO_WAKEUP_GPIO_LOW
  // enum were current as of ESP-IDF v5.x. ESP-IDF v6.0 renamed the enum
  // type to esp_sleep_gpio_wake_up_mode_t and, per Espressif's migration
  // notes, may replace this whole function with
  // esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(...) instead. If
  // the build fails here, check which of those two applies to the
  // resolved core version and swap accordingly - it's a rename/replace,
  // not a redesign.
  esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
  // esp_deep_sleep_start() never returns - the chip resets on wake.
}

void setup() {
  // Absolute first thing, before Wi-Fi/Serial/anything else: prove the
  // firmware is actually running at all by blinking the LED a few times.
  // If this never happens, the problem is upstream of all our app logic -
  // wrong flash, chip not actually running our code, wrong LED_PIN/wiring,
  // dead board, etc. - not a Wi-Fi or GPIO4 bug specifically.
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW : HIGH);
    delay(150);
    digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW);
    delay(150);
  }

  Serial.begin(115200);
  delay(200); // let the USB-serial side settle before the first print
  Serial.println("\n[pedal] boot, wake reason=" + String((int)esp_sleep_get_wakeup_cause()));

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  ledOff(); // stays off through Wi-Fi setup/connect; solid on once connected

  loadConfig();
  Serial.println("[pedal] loaded config: trigger_port=" + String(targetPort) +
                  " linger_s=" + String(lingerSeconds));

  char portBuf[8];
  snprintf(portBuf, sizeof(portBuf), "%u", targetPort);

  customPortParam = new WiFiManagerParameter("target_port", "Trigger UDP port", portBuf, sizeof(portBuf) - 1);
  wm.addParameter(customPortParam);
  wm.setSaveParamsCallback(saveParamsCallback);
  wm.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT_S);
  wm.setCustomHeadElement(CUSTOM_PORTAL_CSS);

  // Holding the pedal itself down for 3+ seconds (instead of a quick tap)
  // forces re-provisioning (new Wi-Fi network, new trigger port) instead
  // of trying saved settings first. Every wake starts with the pedal
  // already down (that's what triggers the wake), so a normal quick tap
  // releases well before FORCE_SETUP_HOLD_MS and this has no effect on
  // the ordinary press-and-trigger behavior.
  bool forceSetup = false;
  if (digitalRead(BUTTON_PIN) == LOW) {
    unsigned long start = millis();
    while (digitalRead(BUTTON_PIN) == LOW && millis() - start < FORCE_SETUP_HOLD_MS) {
      delay(10);
    }
    if (millis() - start >= FORCE_SETUP_HOLD_MS) {
      forceSetup = true;
      wm.resetSettings();
    }
  }

  Serial.println(forceSetup
      ? "[pedal] pedal held 3+s -> forcing setup portal"
      : "[pedal] trying autoConnect (saved Wi-Fi, or setup portal on failure)");

  bool connected;
  if (forceSetup) {
    connected = wm.startConfigPortal("RigolPedal-Setup");
  } else {
    connected = wm.autoConnect("RigolPedal-Setup");
  }

  if (shouldSaveParams) {
    targetPort = (uint16_t) atoi(customPortParam->getValue());
    saveConfig(targetPort, lingerSeconds);
    Serial.println("[pedal] saved new settings from setup portal");
  }

  if (!connected) {
    // Could not connect and nobody configured the portal in time - rather
    // than loop/retry forever and drain the battery, go back to sleep and
    // try again on the next press.
    Serial.println("[pedal] not connected (portal timed out or connect failed) -> sleeping");
    goToSleep();
    return;
  }

  Serial.println("[pedal] connected, IP=" + WiFi.localIP().toString());

  // Compute the real subnet broadcast address (e.g. 10.0.0.255) now that
  // we know our IP and mask, instead of relying on 255.255.255.255 - see
  // the comment on the broadcastAddr declaration above for why.
  {
    IPAddress ip = WiFi.localIP();
    IPAddress mask = WiFi.subnetMask();
    for (int i = 0; i < 4; i++) {
      broadcastAddr[i] = ip[i] | (~mask[i] & 0xFF);
    }
  }
  Serial.println("[pedal] subnet mask=" + WiFi.subnetMask().toString() +
                  " -> broadcast target=" + broadcastAddr.toString());

  WiFi.setSleep(true); // Wi-Fi modem power-save during the linger window below
  ledOn(); // solid on = Wi-Fi link up

  sendTrigger();
  lastTriggerMs = millis();
  hasTriggeredThisWake = true;
  Serial.println("[pedal] sent initial trigger, awake for " + String(lingerSeconds) + "s");

  // Live status/settings page, reachable at this device's IP for as long
  // as it stays awake below - see the handlers above for what it shows.
  statusServer.on("/", HTTP_GET, handleStatusRoot);
  statusServer.on("/pin", HTTP_GET, handleStatusPin);
  statusServer.on("/save", HTTP_POST, handleStatusSave);
  statusServer.on("/trigger", HTTP_POST, handleStatusTrigger);
  statusServer.begin();

  lastActivityMs = millis();
  bool wasDown = (digitalRead(BUTTON_PIN) == LOW);
  unsigned long lastHeartbeatMs = millis();
  while (millis() - lastActivityMs < lingerMs()) {
    bool isDown = (digitalRead(BUTTON_PIN) == LOW);
    if (isDown && !wasDown) {
      sendTrigger();
      lastTriggerMs = millis();
      lastActivityMs = millis();
      Serial.println("[pedal] pedal pressed -> trigger sent, sleep countdown reset");
      ledBlinkForPress(); // off 200ms, back to solid blue - visual press feedback
      if (DEBOUNCE_MS > 200) delay(DEBOUNCE_MS - 200);
    }
    wasDown = isDown;
    statusServer.handleClient();

    // Repeats every 2s for as long as it's awake, on purpose: the ESP32-C3's
    // native USB connection drops and re-enumerates on every reset
    // (including the one deep-sleep wake causes), so the very first boot
    // prints are often lost while Windows/the monitor notice the port came
    // back. A repeating heartbeat means you'll still catch a line even if
    // you missed the boot messages, without needing to watch the exact
    // instant you press the pedal.
    if (millis() - lastHeartbeatMs >= 2000) {
      lastHeartbeatMs = millis();
      long remain = (long)(lingerMs() - (millis() - lastActivityMs)) / 1000;
      Serial.println("[pedal] awake, IP=" + WiFi.localIP().toString() +
                      ", sleeping in " + String(remain) + "s");
    }
    delay(20);
  }

  Serial.println("[pedal] awake window elapsed with no activity -> sleeping");

  goToSleep();
}

void loop() {
  // Not used - setup() handles one full wake/connect/linger/sleep cycle.
}
