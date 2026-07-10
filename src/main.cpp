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

  GPIO pinout summary (see each #define's own comment further down for the
  full reasoning behind the choice):
    GPIO4  BUTTON_PIN            foot pedal switch -> GND (internal pull-up)
    GPIO8  LED_PIN               onboard blue status LED (built into the board)
    GPIO6  EXT_STATUS_LED_PIN    external mirror of the status LED above -
                                 same on/off/blink state, wire your own LED
                                 here + a resistor to GND if you want the
                                 status visible from outside the enclosure
    GPIO5  ACK_LED_PIN           external LED - blinks once the PC app
                                 acknowledges a trigger (see sendTrigger()/
                                 the udp.parsePacket() check in setup())
  Only BUTTON_PIN is required for basic operation; EXT_STATUS_LED_PIN and
  ACK_LED_PIN are both optional - leave them unconnected and the firmware
  just drives an LED that isn't there, no harm done.

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

// A plain mirror of the onboard LED above, on its own external LED - so the
// same status (blinking at boot, solid once Wi-Fi's up, blink on press, off
// before sleep) is also visible from outside the pedal enclosure, not just
// on the tiny onboard LED buried inside the box. Wire an external LED (with
// a current-limiting resistor) between this pin and GND, same as the ack
// LED below - deliberately its OWN pin rather than paralleling the onboard
// LED_PIN directly, since that pin's already at/near its safe drive-current
// budget with just the one onboard LED, and its exact polarity/physical
// pad access is less certain than a plain header pin. GPIO6 is a spare
// general-purpose pin (not a strapping pin, not used for flash/USB) -
// rewire to a different free pin here if 6 is inconvenient on your layout.
#define EXT_STATUS_LED_PIN 6
// Independent of LED_ACTIVE_LOW - this is a different physical LED you're
// wiring yourself, so its wiring orientation (and therefore polarity) is
// entirely up to you. Flip this if it lights up backwards.
#define EXT_STATUS_LED_ACTIVE_LOW false

// A second, separate LED - not the onboard one above - dedicated purely to
// showing that the PC app actually received and is processing the last
// trigger, not just that the pedal thinks it sent one. Wire an external LED
// (with a current-limiting resistor, a few hundred ohms) between this pin
// and GND. GPIO5 is a plain general-purpose pin on the ESP32-C3 (not a
// strapping pin like GPIO2/8/9, not used for flash/USB), so it's safe to
// drive freely - rewire to a different free pin here if 5 is inconvenient
// on your board layout.
#define ACK_LED_PIN 5
// Same reasoning as LED_ACTIVE_LOW above - flip this if the LED lights up
// backwards from what's described where it's used.
#define ACK_LED_ACTIVE_LOW false
// How long the ack LED stays lit per blink - long enough to actually see,
// short enough that back-to-back presses still read as distinct blinks
// rather than one long glow.
#define ACK_LED_BLINK_MS 150UL
// Fixed local UDP port the pedal listens on for the PC app's one-byte
// acknowledgement reply (see udp_trigger_listener's sock.sendto(b"A", addr)
// on the PC side). Without explicitly binding here, the local port used by
// beginPacket()/endPacket() for outgoing sends is whatever the WiFiUDP
// library happens to pick internally - binding it ourselves up front makes
// it a known, fixed target the PC can actually reply to. Deliberately a
// different number from DEFAULT_TRIGGER_PORT below (that one's the PC's
// listening port, this one's the pedal's own).
#define PEDAL_UDP_LISTEN_PORT 47270

// ---- Timing ----
#define FORCE_SETUP_HOLD_MS 3000UL       // how long to hold the pedal to force re-setup
#define DEFAULT_LINGER_S 300             // default: stay awake 5 min after a press
// Floor for the live-settings page. Not just a sanity clamp: this is also
// the least amount of time you're guaranteed to have to notice you set it
// too low and fix it again before the pedal drops off Wi-Fi entirely -
// loading the page, typing a new number, and hitting Save all eat into
// this window too, so it needs real margin, not just "more than zero".
#define MIN_LINGER_S 15
#define MAX_LINGER_S 3600
#define DEBOUNCE_MS 250UL                // minimum time between repeat triggers
// A single digitalRead() sampling LOW does not by itself mean a real
// press happened - a wire run from the pin to a physical footswitch can
// pick up momentary electrical noise/EMI that pulls it low for a few
// milliseconds with no press involved at all. The pin must read
// continuously LOW for at least this long before it's trusted as a real
// press - see the confirm-window logic in setup()'s awake loop. A real
// footswitch press is comfortably longer than this; noise pickup usually
// isn't.
#define PRESS_CONFIRM_MS 30UL
// How often to broadcast a "still awake" UDP heartbeat. 1s (not the
// original 5s) specifically so the PC app is never more than ~1s behind
// the pedal's own real countdown - at 5s, the last heartbeat before
// actually sleeping could be reporting a figure already up to 5s stale,
// which is what made the app's "gone to sleep" reaction lag noticeably
// behind its own on-screen countdown reaching zero. The pedal is already
// awake and on Wi-Fi during this whole window regardless, so one more
// tiny broadcast per second here costs nothing extra worth worrying
// about power-wise.
#define UDP_HEARTBEAT_INTERVAL_MS 1000UL
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

// Bump this by hand with each new firmware build/reflash. Not read by
// anything on the pedal itself - it only exists so the PC app can ask
// "/version" and show which firmware a given pedal is running (handy once
// more than one pedal is in use, or when troubleshooting "did I actually
// reflash the new build onto this one?").
#define FIRMWARE_VERSION "1.5.6"

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
//
// The embedded logo below is a plain, procedurally-drawn placeholder
// badge (verified byte-for-byte: CRC + zlib decompress + per-scanline
// filter-byte checks, all passing) - not the earlier chroma-keyed RIGOL
// artwork, which turned out to have corrupted PNG data that crashed the
// PC app's Tk-based UI on startup (`invalid filter type`). See
// RIGOL_LOGO_PNG_B64's comment in rigol_speaking_multimeter.py for the
// full story and how to get the real artwork back if wanted.
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
    "<div style='color:#999;font-size:0.75em;margin-top:2px;'>Firmware v" FIRMWARE_VERSION "</div>"
    "</div>";

uint16_t targetPort = DEFAULT_TRIGGER_PORT;
uint16_t lingerSeconds = DEFAULT_LINGER_S;
bool shouldSaveParams = false;

// Tracks the awake window: reset by every pedal press AND every request to
// the status page below, so either one keeps the pedal awake. lastActivityMs
// (real millis(), sub-second precision) is what actually decides when to
// sleep - see the while() loop in setup(). Every reported figure - the UDP
// heartbeat, /remaining, the Serial debug line, sendConfigUpdate() - calls
// remainingSecondsNow() (defined below, right after lingerMs()) which
// computes the seconds left fresh, on demand, from this same
// lastActivityMs/millis() source, rather than tracking a separately
// maintained counter.
//
// An earlier version of this file kept a plain integer ("master clock")
// that decremented by exactly 1 every time the UDP heartbeat tick fired,
// on the theory that a single shared counter could never disagree with
// itself the way independently-computed elapsed-time figures on each side
// once did. That was true, but it traded away something more important:
// decrementing by exactly 1 assumes the heartbeat tick actually fires
// once a second. If the loop stalls for any reason - WiFi modem-sleep
// pauses, the DEBOUNCE_MS delay() after a press, or anything else
// blocking statusServer.handleClient() for a few seconds - the counter
// still only ticks down by 1, so it drifts and reports MORE time
// remaining than is truly left relative to the real, millis()-based sleep
// decision. Computing on demand from elapsed time (with rounding, not
// truncation) is immune to that: it's always correct regardless of how
// many ticks were actually serviced, while still being the one shared
// function every read site calls, so they still can't disagree with each
// other.
unsigned long lastActivityMs = 0;
unsigned long lastTriggerMs = 0;
bool hasTriggeredThisWake = false;

// Whether the PC app has acknowledged the *current* trigger yet - reset to
// false every time a fresh trigger goes out (all three send sites below set
// this alongside their existing lastTriggerMs/hasTriggeredThisWake updates),
// set true the instant the ack reply arrives (see the udp.parsePacket()
// check in setup()'s awake loop). lastTriggerAckMs is only meaningful once
// lastTriggerAcked is true - it's what round-trip time and the /trigger_ack
// endpoint below are computed from.
bool lastTriggerAcked = false;
unsigned long lastTriggerAckMs = 0;
// 0 = ack LED is idle/off. Non-zero = the millis() timestamp it was turned
// on, so the main loop can turn it back off after ACK_LED_BLINK_MS without
// ever calling delay() - a blocking blink here would reintroduce exactly
// the kind of loop stall today's other fixes were about removing.
unsigned long ackLedOnAtMs = 0;

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

// The single shared source of truth for "how many seconds until this
// pedal goes back to sleep", computed fresh every time it's called from
// lastActivityMs/millis() - the same clock that actually decides when to
// sleep (see the while() loop in setup()). Rounds to the nearest second
// (not truncating) so it agrees with the same figure computed independently
// on the PC app side. See the comment above lastActivityMs's declaration
// for why this replaced an earlier decrementing-counter design.
long remainingSecondsNow() {
  long elapsedMs = (long)(millis() - lastActivityMs);
  long remainingMs = (long)lingerMs() - elapsedMs;
  if (remainingMs < 0) remainingMs = 0;
  return (remainingMs + 500) / 1000;
}

// ---- Status LED ----
// Off during Wi-Fi setup/connect. Solid on once connected = "Wi-Fi link
// up". Blinks off briefly on each subsequent pedal press during the awake
// window (not the initial wake-up press itself, since that's implicit in
// the LED turning on), then returns to solid on. Off again before deep
// sleep. Drives both the onboard LED and its external mirror (see
// EXT_STATUS_LED_PIN's comment) together, so every call site below stays a
// single ledOn()/ledOff() - nothing had to change anywhere else in the file
// to pick up the second LED.
void ledOn() {
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW : HIGH);
  digitalWrite(EXT_STATUS_LED_PIN, EXT_STATUS_LED_ACTIVE_LOW ? LOW : HIGH);
}

void ledOff() {
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(EXT_STATUS_LED_PIN, EXT_STATUS_LED_ACTIVE_LOW ? HIGH : LOW);
}

void ledBlinkForPress() {
  ledOff();
  delay(200);
  ledOn();
}

// ---- Ack LED ----
// Separate from the status LED above - see ACK_LED_PIN's comment for why.
// Deliberately no blocking blink() helper here like ledBlinkForPress()'s:
// turning this LED off again is handled non-blockingly in the main loop
// (see ackLedOnAtMs), since the ack can arrive at any arbitrary moment
// mid-loop, not just right after a press when a brief delay() is tolerable.
void ackLedOn() {
  digitalWrite(ACK_LED_PIN, ACK_LED_ACTIVE_LOW ? LOW : HIGH);
}

void ackLedOff() {
  digitalWrite(ACK_LED_PIN, ACK_LED_ACTIVE_LOW ? HIGH : LOW);
}

// Payload format for both packet kinds below: "<K>:<secondsUntilSleep>",
// e.g. "T:300" or "H:187". K is 'T' for an actual pedal-press trigger, 'H'
// for a heartbeat - the PC app tells them apart by this prefix and only
// ever speaks the reading for a 'T', never for a plain 'H'. Any packet
// that *doesn't* parse as this format (e.g. an older pedal's bare single
// zero byte) is still treated as a trigger with unknown remaining time by
// the PC app, for backward compatibility with already-flashed pedals.
void sendUdpPacket(char kind, long remainingS) {
  String payload = String(kind) + ":" + String(remainingS);
  udp.beginPacket(broadcastAddr, targetPort);
  udp.write((const uint8_t *)payload.c_str(), payload.length());
  int ok = udp.endPacket();
  // endPacket() only confirms the ESP32 handed the packet to the network -
  // it says nothing about whether the router forwarded it or the PC
  // actually received it. Still, ok==0 here means it failed at the source
  // and never had a chance, which narrows things down a lot if that's
  // ever what's happening.
  Serial.println("[pedal] sendUdpPacket " + payload + " to " + broadcastAddr.toString() +
                  ":" + String(targetPort) + " -> udp.endPacket()=" + String(ok));
}

void sendTrigger() {
  // Broadcasts to the whole local network rather than a specific PC
  // address - no "target IP" needs configuring, and it keeps working if
  // the PC's DHCP-assigned IP ever changes. Every call site resets
  // lastActivityMs right before calling this (a press always means "full
  // window again"), so remainingSecondsNow() here is the same thing as
  // lingerSeconds at this exact moment.
  sendUdpPacket('T', remainingSecondsNow());
}

// A separate, lightweight "still awake" broadcast - lets the PC app
// confirm the pedal is alive/connected (and how long it has left before
// sleeping, per the pedal's own actual countdown) without it counting as
// a button press. Sent once a second during the awake window (see the
// loop in setup()).
void sendHeartbeat() {
  sendUdpPacket('H', remainingSecondsNow());
}

// Sent once, immediately, whenever the stay-awake time is actually
// changed on the settings page below - so the PC app finds out its new
// total right away instead of waiting on its own periodic re-check (or a
// fresh physical press) to notice. handleStatusSave() already resets
// lastActivityMs before calling this, same as loading the page does, so
// remainingSecondsNow() *is* the true remaining time at this exact
// moment - same shape as a trigger packet, just a distinct 'C' (config)
// kind so the app knows not to treat it as a button press.
void sendConfigUpdate() {
  sendUdpPacket('C', remainingSecondsNow());
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
    "#sleeptimer{font-weight:bold;}"
    // Fixed width + a min-height sized for 2 lines of text, regardless of
    // which of the ack states below is showing - the shortest ("waiting...")
    // and longest ("no response yet (X.Xs) - is the app running?") messages
    // would otherwise wrap to a different number of lines and make the
    // whole page's height jump around as this text changes on its own
    // (every ~500ms, via updateAckState() below) without any user action -
    // reserving the same box size for every possible message avoids that.
    "#ackstate{display:inline-block;width:230px;min-height:2.6em;vertical-align:top;}"
    "button:disabled,input:disabled{opacity:0.4;cursor:not-allowed;}"
    "</style>";

// Same logo image used on the WiFiManager setup portal - duplicated here
// as its own literal (rather than shared/reused) for simplicity; the
// extra ~3.7KB of flash is not worth the added complexity of sharing one
// copy across two otherwise-unrelated page builders. See the matching
// comment on CUSTOM_PORTAL_CSS above (and RIGOL_LOGO_PNG_B64 in the PC
// app) for why this is a plain verified placeholder badge rather than the
// earlier chroma-keyed artwork, which had corrupted PNG data that crashed
// the PC app on startup and would very likely have failed to render here
// too.
const char STATUS_PAGE_LOGO[] =
    "<div style='text-align:center;padding:6px 0 2px;'>"
    "<img src='data:image/png;base64,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'"
    " style='max-width:150px;height:auto;display:block;margin:0 auto;'>"
    "</div>";

void handleStatusRoot() {
  lastActivityMs = millis(); // just loading the page counts as activity,
                              // so it always shows the full window remaining
  long remainingS = remainingSecondsNow();

  String lastTrigger = hasTriggeredThisWake
      ? String((millis() - lastTriggerMs) / 1000) + " s ago"
      : "not yet this session";

  String page = "<!DOCTYPE html><html><head><meta name='viewport' "
      "content='width=device-width,initial-scale=1'>";
  page += STATUS_PAGE_CSS;
  page += "</head><body>";
  page += STATUS_PAGE_LOGO;
  page += "<h2>Rigol foot pedal</h2>";
  page += "<p><b>IP:</b> " + WiFi.localIP().toString() + "<br>";
  page += "<b>Wi-Fi network:</b> " + WiFi.SSID() + "<br>";
  page += "<b>Last trigger:</b> <span id='lasttrigger'>" + lastTrigger + "</span><br>";
  page += "<b>PC acknowledged:</b> <span id='ackstate'>-</span><br>";
  page += "<b>Sleeping in:</b> <span id='sleeptimer'>" + String(remainingS) +
      "</span> s (any activity here resets this)</p>";
  // Full-width bar right after a fresh press, shrinking from the right as
  // the countdown above runs down - same numbers as #sleeptimer, just
  // also visible at a glance without reading the seconds.
  page += "<div id='sleepbar-outer' style='background:#e0e0e0;border:1px solid #999;"
      "border-radius:4px;height:10px;overflow:hidden;margin:4px 0 10px;'>"
      // width's transition duration matches the 1s poll interval below
      // (updateSleepTimer's setInterval) exactly - a shorter one (this
      // used to be 0.5s) finishes the animation early and then sits
      // frozen for the rest of the second before snapping to the next
      // step, which is what made it look jerky/uneven rather than a
      // smooth, continuous countdown. background-color stays quick since
      // a threshold color change is meant to be a snap, not a glide.
      "<div id='sleepbar-inner' style='background:#2ecc71;height:100%;width:100%;"
      "transition:width 1s linear,background-color 0.3s linear;'></div></div>";
  page += "<p id='sleepwarning' style='color:#b00;font-weight:bold;display:none'>"
      "Pedal is asleep - the buttons below won't do anything until you press "
      "the pedal to wake it up and reload this page.</p>";
  page += "<p><b>Pedal input right now:</b> <span id='pinstate'>"
      + String(digitalRead(BUTTON_PIN) == LOW ? "PRESSED" : "RELEASED")
      + "</span><br><span style='color:#666;font-size:0.85em'>(updates live - "
      "press/release the pedal to check wiring and contact bounce)</span></p>";
  page += "<script>"
      // totalLingerS: the pedal's own configured stay-awake window at the
      // moment this page was rendered - the fixed denominator the bar's
      // percentage is scaled against. (Loading this page just reset the
      // countdown to the full window - see lastActivityMs above - so this
      // is also exactly what remainingS starts at.)
      "var totalLingerS = " + String(lingerSeconds) + ";"
      // displayRemaining: what's actually shown on screen right now.
      // Ticked down once a second by the browser's own local timer below,
      // independent of the network - and separately corrected/resynced
      // to the pedal's real, elapsed-time-accurate figure whenever a
      // /remaining poll actually succeeds. Splitting these two apart is
      // what fixes the display freezing whenever a poll is briefly
      // delayed (Wi-Fi housekeeping, the ESP32 loop busy elsewhere) and
      // then jumping/"catching up" several seconds at once the moment a
      // late response finally lands - the local tick keeps it gliding at
      // a steady 1/s throughout, and the poll's correction is then small
      // (usually invisible) instead of one big visible snap.
      "var displayRemaining = " + String(remainingS) + ";"
      // Counts consecutive failed/timed-out polls in a row - see the
      // .catch() below for why this exists (a single blip shouldn't be
      // enough to declare the pedal asleep).
      "var missedPolls = 0;"
      // Authoritative "is it actually asleep" state - only ever set by
      // the network poll below (a real reply of 0, or enough misses in a
      // row), never guessed locally. The local tick keeps counting down
      // for display purposes but is not what decides this.
      "var isAsleep = false;"
      "function renderRemaining(n, asleep){"
        "document.getElementById('sleeptimer').textContent = asleep ? '0' : n;"
        "document.getElementById('triggerbtn').disabled = asleep;"
        "document.getElementById('savebtn').disabled = asleep;"
        "document.getElementById('sleepwarning').style.display = asleep ? 'block' : 'none';"
        "var pct = asleep ? 0 : Math.max(0, Math.min(100, (n / totalLingerS) * 100));"
        "var bar = document.getElementById('sleepbar-inner');"
        "bar.style.width = pct + '%';"
        // Green while there's a comfortable margin left, amber under a
        // quarter of the window, red for the last 10% - the same
        // countdown, just harder to miss out of the corner of your eye
        // as it gets close to timing out.
        "bar.style.backgroundColor = pct <= 10 ? '#e74c3c' : (pct <= 25 ? '#f39c12' : '#2ecc71');"
      "}"
      // The actual once-a-second glide - runs on the browser's own clock,
      // completely independent of whether the network poll below happens
      // to be on time. This is the piece that was missing before: without
      // it, the *only* thing driving the display was however often the
      // network happened to answer, so any delay showed up directly as a
      // frozen number.
      "setInterval(function(){"
        "if (isAsleep) return;"
        "if (displayRemaining > 0) displayRemaining--;"
        "renderRemaining(displayRemaining, false);"
      "}, 1000);"
      // Live network poll, also every second - lets you see it's about to
      // sleep *before* you try to click something and hit a dead
      // connection, and is what actually corrects displayRemaining to the
      // pedal's real figure and decides asleep/awake. Deliberately a
      // separate, slower interval from the /pin poll below (that one's
      // for watching contact bounce, this one just needs to be roughly
      // right, not instant).
      "function updateSleepTimer(){"
        // Without an explicit timeout, fetch() to a pedal that has
        // actually gone to sleep (dropped off Wi-Fi entirely) just hangs
        // until the browser's own generic connection timeout gives up -
        // which can be tens of seconds, far longer than this 1s poll
        // interval - leaving the countdown frozen on the last good number
        // and the buttons never greying out. Aborting after 3s makes the
        // .catch() below fire promptly and reliably instead of hanging.
        "var ctrl = new AbortController();"
        "var abortTimer = setTimeout(function(){ ctrl.abort(); }, 3000);"
        "fetch('/remaining', {signal: ctrl.signal}).then(function(r){"
          "clearTimeout(abortTimer);"
          "return r.text();"
        "}).then(function(t){"
          "missedPolls = 0;"
          "var n = parseInt(t, 10);"
          "isAsleep = isNaN(n) || n <= 0;"
          "displayRemaining = isAsleep ? 0 : n;" // resync to the pedal's real figure
          "renderRemaining(displayRemaining, isAsleep);"
        "}).catch(function(){"
          // A single failed/timed-out poll is far more often just the
          // ESP32's single-threaded loop being briefly busy elsewhere
          // (debounce delay, Wi-Fi housekeeping) than the pedal actually
          // having gone to sleep - treating one miss as "asleep for real"
          // was flashing the countdown/buttons down to zero and straight
          // back up moments later on a perfectly awake pedal, which looks
          // like the counter "lost its value". Requiring 2 misses in a
          // row (~3s of real silence either way) before declaring it
          // asleep filters that out, while still reacting quickly to an
          // actual sleep - which won't have anything answering ever again
          // until the next physical press."
          "missedPolls++;"
          "if (missedPolls < 2) return;"
          "isAsleep = true;"
          "displayRemaining = 0;"
          "renderRemaining(0, true);"
        "});"
      "}"
      "setInterval(updateSleepTimer, 1000);"
      "setInterval(function(){"
        "fetch('/pin').then(function(r){return r.text();}).then(function(t){"
        "document.getElementById('pinstate').textContent = t;"
        "});"
      "}, 250);"
      // Turns an age in ms into a short "Ns ago" / "Nm Ns ago" string - used
      // so both the ack line and the "Last trigger" line always say plainly
      // how long ago the underlying press happened, not just a number that
      // could be mistaken for "just now" (see formatAge() call sites below).
      "function formatAge(ms){"
        "var s = Math.floor(ms / 1000);"
        "if (s < 60) return s + ' s ago';"
        "var m = Math.floor(s / 60);"
        "return m + ' m ' + (s % 60) + ' s ago';"
      "}"
      // Shows whether the PC app actually received the last trigger, not
      // just that the pedal thinks it sent one - see handleStatusTriggerAck()
      // and the physical second LED (ACK_LED_PIN) that lights up at the same
      // moment this text updates. Also re-states how long ago that trigger
      // happened alongside the round-trip time, and keeps the "Last trigger"
      // line ticking live off the same poll response - a bare "yes (266ms)"
      // used to look like fresh news no matter how long ago the press
      // actually was, since 266ms always reads as "just now".
      "function updateAckState(){"
        "fetch('/trigger_ack').then(function(r){return r.text();}).then(function(t){"
          "var el = document.getElementById('ackstate');"
          "var lastEl = document.getElementById('lasttrigger');"
          "if (t === 'NONE') {"
            "el.textContent = 'no trigger sent yet this session';"
            "el.style.color = '#666';"
            "return;"
          "}"
          "var parts = t.split(':');"
          "if (parts[0] === 'ACKED') {"
            "var rtt = parseInt(parts[1], 10);"
            "var age = parseInt(parts[2], 10);"
            "el.textContent = 'yes, ' + rtt + ' ms round-trip (press was ' + formatAge(age) + ')';"
            "el.style.color = '#2ecc71';"
            "if (lastEl) lastEl.textContent = formatAge(age);"
          "} else if (parts[0] === 'PENDING') {"
            "var age2 = parseInt(parts[1], 10);"
            "if (age2 > 3000) {"
              "el.textContent = 'no response yet (' + (age2/1000).toFixed(1) + ' s) - is the app running?';"
              "el.style.color = '#e74c3c';"
            "} else {"
              "el.textContent = 'waiting for PC ack...';"
              "el.style.color = '#f39c12';"
            "}"
            "if (lastEl) lastEl.textContent = formatAge(age2);"
          "}"
        "}).catch(function(){});"
      "}"
      "setInterval(updateAckState, 500);"
      "</script>";
  page += "<form method='POST' action='/trigger'>"
      "<button type='submit' id='triggerbtn'>Send test trigger now</button></form>";
  page += "<hr><h3>Settings</h3>";
  page += "<form method='POST' action='/save'>";
  page += "<label>Trigger UDP port<input type='number' min='1' max='65535' "
      "name='port' value='" + String(targetPort) + "'></label>";
  page += "<label>Stay-awake time after a press, seconds (" +
      String(MIN_LINGER_S) + "-" + String(MAX_LINGER_S) + ")"
      "<input type='number' min='" + String(MIN_LINGER_S) + "' max='" +
      String(MAX_LINGER_S) + "' name='linger' value='" + String(lingerSeconds) + "'></label>";
  page += "<p><input type='submit' id='savebtn' value='Save settings'></p>";
  page += "</form>";
  page += "<p style='color:#666;font-size:0.85em'>To change Wi-Fi network, "
      "hold the pedal down for 3+ seconds right as it wakes.</p>";
  page += "<p style='color:#999;font-size:0.75em'>Firmware v" FIRMWARE_VERSION "</p>";
  page += "<p style='color:#999;font-size:0.75em'>KOCH Engineering by Henrik J. Koch</p>";
  page += "</body></html>";

  // Connection: close - see handleStatusRemaining()'s comment. Applies here
  // too so even a manual F5/reload after a long sleep opens a fresh
  // connection immediately rather than possibly hanging on a stale one.
  statusServer.sendHeader("Connection", "close");
  statusServer.sendHeader("Cache-Control", "no-store");
  statusServer.send(200, "text/html", page);
}

void handleStatusSave() {
  lastActivityMs = millis();
  if (statusServer.hasArg("port")) {
    int p = statusServer.arg("port").toInt();
    if (p > 0 && p <= 65535) targetPort = (uint16_t)p;
  }
  uint16_t oldLingerSeconds = lingerSeconds;
  if (statusServer.hasArg("linger")) {
    int s = statusServer.arg("linger").toInt();
    if (s >= MIN_LINGER_S && s <= MAX_LINGER_S) lingerSeconds = (uint16_t)s;
  }
  if (lingerSeconds != oldLingerSeconds) {
    // Tell the PC app right away rather than leaving it to notice on its
    // own - see sendConfigUpdate()'s comment.
    sendConfigUpdate();
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
  // Connection: close (see handleStatusRemaining()'s comment for the full
  // story) - forces the browser to open a brand-new TCP connection for
  // every one of these polls rather than trying to reuse a stale one.
  statusServer.sendHeader("Connection", "close");
  statusServer.sendHeader("Cache-Control", "no-store");
  statusServer.send(200, "text/plain", digitalRead(BUTTON_PIN) == LOW ? "PRESSED" : "RELEASED");
}

// Polled by the status page's JS every second to drive the live
// "Sleeping in: Ns" countdown and grey out the trigger/save buttons once
// time's up - so the page tells you *before* you click into a dead
// connection, rather than after. Deliberately does NOT reset
// lastActivityMs / count as activity - same reasoning as /pin: just
// having the page open polling this shouldn't itself keep the pedal
// permanently awake.
void handleStatusRemaining() {
  // Calls the same shared function the UDP heartbeat uses - never
  // computed independently, so the two can never disagree with each
  // other or drift apart over a long awake session.
  //
  // Connection: close is the actual fix for "the page shows updates again,
  // but only a long time after the pedal wakes back up, not within a few
  // seconds": when the pedal deep-sleeps, its Wi-Fi radio just switches
  // off - there's no graceful TCP close/FIN sent to any browser tab that's
  // still open and polling. If that tab's fetch() calls were using a
  // persistent (keep-alive) HTTP connection, the browser has no way to
  // know that connection died; it can sit "half-open" from the browser's
  // point of view. Our own AbortController (see the page's JS) only bounds
  // how long *that one fetch's promise* waits before giving up - it does
  // NOT force the browser to discard the underlying dead socket. New
  // requests to the same origin can then get funneled through (or queued
  // behind) that same broken connection instead of opening a fresh one,
  // so the whole polling loop can end up stuck behind it until the
  // browser's own, much longer, low-level TCP timeout finally gives up -
  // easily tens of seconds to minutes, not the ~1-3s these polls are
  // designed around. Sending "Connection: close" on every response tells
  // the browser to never keep this connection alive in the first place, so
  // every single poll always opens a brand-new TCP connection: while the
  // pedal is truly asleep that fails fast (nothing listening - immediate
  // refusal, not a hang), and the moment the pedal is back up and
  // listening again, the very next poll's fresh connection just works.
  statusServer.sendHeader("Connection", "close");
  statusServer.sendHeader("Cache-Control", "no-store");
  statusServer.send(200, "text/plain", String(remainingSecondsNow()));
}

void handleStatusTrigger() {
  lastActivityMs = millis(); // reset *before* sendTrigger() reads remainingSecondsNow()
  sendTrigger();
  lastTriggerMs = millis();
  hasTriggeredThisWake = true;
  lastTriggerAcked = false; // waiting on the PC's ack for this new trigger
  statusServer.sendHeader("Location", "/");
  statusServer.send(302, "text/plain", "");
}

// Machine-readable version info, so the PC app can ask "which firmware is
// this pedal running?" - it already learns this pedal's IP from the
// source address of every trigger packet, so it can fetch this right
// after noting a trigger while the pedal is still awake. Deliberately
// does NOT reset lastActivityMs / count as "activity" - same reasoning as
// /pin: a background version check shouldn't itself keep the pedal awake.
void handleStatusVersion() {
  // Includes the pedal's own configured stay-awake window (linger_s) so
  // the PC app can show an accurate "probably still awake for Ns" /
  // "probably asleep by now" hint instead of guessing a hardcoded default -
  // this value is user-changeable per pedal on its settings page above.
  String json = "{\"version\":\"" + String(FIRMWARE_VERSION) +
      "\",\"board\":\"esp32-c3\"," +
      "\"linger_s\":" + String(lingerSeconds) + "}";
  statusServer.send(200, "application/json", json);
}

// Polled by the status page's JS every ~500ms to show whether the PC app
// actually received the last trigger - not just that the pedal thinks it
// sent one. Also carries how long ago that trigger happened (age_ms) so the
// page can make clear whether "yes" means "just now" or "acknowledged a
// while ago, nothing has happened since" - showing only the round-trip time
// (e.g. "yes (266ms)") looked like fresh news even minutes after the actual
// press, since 266ms reads as "just now" regardless of when it really
// occurred. Four fields, colon-separated:
//   "NONE"              - no trigger sent yet this wake session
//   "ACKED:123:4500"    - acked; 123ms round-trip, press was 4500ms ago
//   "PENDING:456"       - sent 456ms ago, no ack yet (still might arrive)
// Deliberately does NOT reset lastActivityMs - same reasoning as /pin and
// /remaining: watching this page shouldn't itself keep the pedal awake.
void handleStatusTriggerAck() {
  // Connection: close - see handleStatusRemaining()'s comment for why.
  statusServer.sendHeader("Connection", "close");
  statusServer.sendHeader("Cache-Control", "no-store");
  if (!hasTriggeredThisWake) {
    statusServer.send(200, "text/plain", "NONE");
  } else if (lastTriggerAcked) {
    statusServer.send(200, "text/plain",
        "ACKED:" + String(lastTriggerAckMs - lastTriggerMs) + ":" +
        String(millis() - lastTriggerMs));
  } else {
    statusServer.send(200, "text/plain", "PENDING:" + String(millis() - lastTriggerMs));
  }
}

void goToSleep() {
  ledOff();
  ackLedOff();
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
  pinMode(EXT_STATUS_LED_PIN, OUTPUT);
  pinMode(ACK_LED_PIN, OUTPUT);
  ackLedOff(); // stays off until the PC actually acknowledges a trigger
  for (int i = 0; i < 8; i++) {
    digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW : HIGH);
    digitalWrite(EXT_STATUS_LED_PIN, EXT_STATUS_LED_ACTIVE_LOW ? LOW : HIGH);
    delay(150);
    digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW);
    digitalWrite(EXT_STATUS_LED_PIN, EXT_STATUS_LED_ACTIVE_LOW ? HIGH : LOW);
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

  // Modem power-save (WiFi.setSleep(true)) periodically parks the radio
  // between beacon intervals to save power - which is also exactly what
  // causes the pedal to intermittently stop answering (heartbeats,
  // /remaining, /pin) for a second or two at a time before catching up,
  // even though its own internal countdown (driven by millis(), not
  // Wi-Fi) never actually paused. The awake window is only a few minutes
  // at most, and the real power savings happen during deep sleep
  // afterward regardless - so it's not worth trading away a
  // consistently-responsive live page/heartbeat for it here.
  WiFi.setSleep(false);
  ledOn(); // solid on = Wi-Fi link up

  // Bind our own local UDP port before sending anything, so the PC app's
  // one-byte acknowledgement reply has a known, fixed address to come back
  // to - see PEDAL_UDP_LISTEN_PORT's comment above. Same WiFiUDP object is
  // still used for the outgoing broadcasts below via beginPacket()/
  // endPacket(); binding it first doesn't change that, it just also makes
  // it able to receive.
  udp.begin(PEDAL_UDP_LISTEN_PORT);

  lastActivityMs = millis(); // start the awake window at the full linger time
  sendTrigger();
  lastTriggerMs = millis();
  hasTriggeredThisWake = true;
  lastTriggerAcked = false; // waiting on the PC's ack for this new trigger
  Serial.println("[pedal] sent initial trigger, awake for " + String(lingerSeconds) + "s");

  // Live status/settings page, reachable at this device's IP for as long
  // as it stays awake below - see the handlers above for what it shows.
  statusServer.on("/", HTTP_GET, handleStatusRoot);
  statusServer.on("/pin", HTTP_GET, handleStatusPin);
  statusServer.on("/remaining", HTTP_GET, handleStatusRemaining);
  statusServer.on("/save", HTTP_POST, handleStatusSave);
  statusServer.on("/trigger", HTTP_POST, handleStatusTrigger);
  statusServer.on("/version", HTTP_GET, handleStatusVersion);
  statusServer.on("/trigger_ack", HTTP_GET, handleStatusTriggerAck);
  statusServer.begin();

  lastActivityMs = millis();
  bool wasDown = (digitalRead(BUTTON_PIN) == LOW);
  // downSinceMs: when the pin most recently transitioned to LOW - 0 means
  // "not currently down". confirmed: whether *this* low stretch has
  // already been accepted as a real press, so holding the pedal down
  // doesn't repeatedly re-trigger. Neither is trusted until the pin has
  // read continuously LOW for PRESS_CONFIRM_MS - see that constant's
  // comment for why (a bare instantaneous LOW reading can be noise
  // picked up on the wire, not an actual press).
  unsigned long downSinceMs = wasDown ? millis() : 0;
  bool confirmed = false;
  unsigned long lastHeartbeatMs = millis();
  unsigned long lastUdpHeartbeatMs = millis();
  while (millis() - lastActivityMs < lingerMs()) {
    bool isDown = (digitalRead(BUTTON_PIN) == LOW);
    if (isDown && !wasDown) {
      downSinceMs = millis(); // just went low - not trusted yet
    }
    if (!isDown) {
      // Released (or never was a real press) - reset for the next one.
      downSinceMs = 0;
      confirmed = false;
    } else if (!confirmed && downSinceMs != 0 && millis() - downSinceMs >= PRESS_CONFIRM_MS) {
      // Stayed low long enough to trust it - this is a real press.
      confirmed = true;
      lastActivityMs = millis(); // reset *before* sendTrigger() reads remainingSecondsNow()
      sendTrigger();
      lastTriggerMs = millis();
      lastTriggerAcked = false; // waiting on the PC's ack for this new trigger
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
      Serial.println("[pedal] awake, IP=" + WiFi.localIP().toString() +
                      ", sleeping in " + String(remainingSecondsNow()) + "s");
    }

    // Separate UDP heartbeat, on its own (longer) interval - lets the PC
    // app confirm this pedal is alive/connected, and exactly how long it
    // has left before sleeping, without a physical press. See
    // sendHeartbeat()/sendUdpPacket() above for the payload format the PC
    // app tells apart from an actual trigger.
    //
    // UDP_HEARTBEAT_INTERVAL_MS is 1000ms, so under normal conditions this
    // fires once a second - but sendHeartbeat() reads remainingSecondsNow(),
    // which is computed fresh from elapsed real time every call, not a
    // counter ticked down once per fire. So even if this loop stalls for a
    // few seconds (WiFi modem-sleep pause, a blocking delay() elsewhere)
    // and this block ends up firing late, the very next heartbeat still
    // reports the true remaining time, not a drifted estimate. The PC app
    // mirrors the same approach on its side (recomputing from its own
    // elapsed-time source each tick, resynced whenever a packet arrives)
    // instead of maintaining an independently-ticking mirror.
    if (millis() - lastUdpHeartbeatMs >= UDP_HEARTBEAT_INTERVAL_MS) {
      lastUdpHeartbeatMs = millis();
      sendHeartbeat();
    }

    // Turn the ack LED back off once it's been lit for ACK_LED_BLINK_MS -
    // scheduled here rather than a blocking delay(), so a blink never stalls
    // the rest of this loop (same reasoning as everywhere else this session).
    if (ackLedOnAtMs != 0 && millis() - ackLedOnAtMs >= ACK_LED_BLINK_MS) {
      ackLedOff();
      ackLedOnAtMs = 0;
    }

    // Check for the PC app's one-byte acknowledgement of our last trigger -
    // sent the instant it receives one (see udp_trigger_listener's
    // sock.sendto(b"A", addr) on the PC side). Requires udp.begin() above so
    // this WiFiUDP object is bound to a fixed local port the PC can actually
    // reply to. Only the *first* ack for a given trigger matters - ignore
    // any further replies until the next trigger resets lastTriggerAcked.
    int ackPacketSize = udp.parsePacket();
    if (ackPacketSize > 0) {
      char ackBuf[8];
      int ackLen = udp.read(ackBuf, sizeof(ackBuf) - 1);
      if (ackLen > 0) {
        ackBuf[ackLen] = '\0';
        if (ackBuf[0] == 'A' && !lastTriggerAcked) {
          lastTriggerAcked = true;
          lastTriggerAckMs = millis();
          ackLedOn();
          ackLedOnAtMs = millis();
          Serial.println("[pedal] PC acknowledged trigger (" +
                          String(lastTriggerAckMs - lastTriggerMs) + "ms round-trip)");
        }
      }
    }
    delay(20);
  }

  Serial.println("[pedal] awake window elapsed with no activity -> sleeping");

  goToSleep();
}

void loop() {
  // Not used - setup() handles one full wake/connect/linger/sleep cycle.
}
