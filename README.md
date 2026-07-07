# Rigol Speaking Multimeter - ESP32-C3 Foot Pedal

A Wi-Fi foot pedal trigger for [rigol-speaking-multimeter](../rigol-speaking-multimeter):
press it, and the PC running that app speaks the current multimeter
reading - same effect as pressing F8, but hands-free, for when both hands
are busy with test leads.

## Hardware

- An **ESP32-C3 "Super Mini"** board (or any ESP32-C3 board - pin numbers
  may need adjusting for a different board's silkscreen/layout).
- A normally-open momentary foot switch/pedal, wired between **GPIO4** and
  a nearby **GND** pad.

No external resistor needed - the firmware enables the chip's own
built-in pull-up on GPIO4, both via `pinMode` for normal operation and,
right before sleeping, again via the IDF GPIO driver (`gpio_pullup_en`) to
make sure it's actually held through deep sleep, since `pinMode`'s setting
isn't guaranteed to survive that. (An earlier draft used `rtc_gpio_pullup_en`,
which is the right call on the original ESP32 but doesn't exist on the
ESP32-C3's simpler GPIO/RTC architecture - confirmed by an actual build
error, fixed to the plain `gpio_pullup_en`/`gpio_pulldown_dis`.)

GPIO4 was chosen deliberately: it's RTC-capable, isn't one of the three
strapping pins, and - confirmed against real-world ESP32-C3 deep-sleep
reports - GPIO0/GPIO1 are RTC-capable too but do *not* reliably work as
wake sources on this chip (a known Arduino-ESP32 core limitation); only
GPIO2-5 do.

## Building and flashing

Requires [PlatformIO](https://platformio.org/) (the CLI, or the VS Code
extension). From this folder:

```
pio run --target upload
```

Then, to watch its serial output while testing:

```
pio device monitor
```

Note: this board almost certainly uses the ESP32-C3's built-in native USB
(not a separate USB-UART bridge chip), so every reset - including waking
from deep sleep - makes the USB connection itself drop and re-enumerate.
The very first boot log lines are often lost in that brief gap before
Windows/the monitor notice the port came back. Don't worry if you miss
them; the firmware repeats an "awake, IP=..., sleeping in Ns" line every
2 seconds for as long as it stays awake, so you'll catch that even if you
missed the initial boot messages.

**Confirmed working end-to-end on real hardware** (an "ESP32-C3 Super
Mini" board): deep sleep, GPIO4 wake, Wi-Fi setup portal, subnet-broadcast
UDP trigger, and a real pedal press making
[rigol-speaking-multimeter](../rigol-speaking-multimeter) speak on the PC.
The one non-obvious extra step it took to get there: Windows Firewall was
silently dropping the incoming UDP packet on the PC side, even though the
pedal itself was sending it correctly - see that project's README for how
the PC app now handles this mostly automatically.

## First-time setup

1. Power on the pedal. It creates its own temporary Wi-Fi network called
   **"RigolPedal-Setup"** (setup page styled with yellow buttons/black
   text).
2. Connect to that network from your phone or laptop. A setup page should
   open automatically; if not, browse to `192.168.4.1`.
3. Pick your real Wi-Fi network and enter its password.
4. Check **Trigger UDP port** — pre-filled with `47269`, matching the
   Python app's own default `--trigger-port`; leave it alone unless you
   changed that on the PC side.
5. Save. The pedal reboots and joins your real network.

There's no PC IP address to enter: the pedal broadcasts its trigger to the
whole local network rather than addressing one specific PC, so nothing
needs re-entering if the PC's DHCP-assigned IP ever changes.

To redo this later (new Wi-Fi network, changed trigger port), hold the
**foot pedal itself** down for 3+ seconds right as it wakes, instead of a
quick tap. (Not the onboard BOOT button — that's a boot-mode strapping pin
on the ESP32-C3, and holding it down during power-on/reset sends the chip
into its serial bootloader instead of running the firmware at all, which
is why an earlier version of these instructions didn't work.)

## How it behaves

- Spends almost all its time in deep sleep (very low power).
- A pedal press wakes it, it joins Wi-Fi, and broadcasts the trigger packet
  to the local network on the configured port.
- It then stays awake for a configurable stay-awake window (5 minutes by
  default), resetting that countdown on every further press, so a burst of
  presses during one probing session doesn't pay the Wi-Fi-reconnect cost
  each time.
- After that window passes with no press, it goes back to deep sleep.
- The onboard blue LED (**GPIO8**) blinks a few times at boot (proof the
  firmware is actually running, independent of Wi-Fi), turns solid on once
  connected to Wi-Fi ("link up"), blinks off for 200ms and back on each
  time the pedal is pressed while awake, and turns off before going to
  deep sleep - confirmed working as described. If it ever behaves
  backwards on a different board (on when it should be off, etc.), flip
  `LED_ACTIVE_LOW` in `src/main.cpp`.

## Live status page (while awake)

While the pedal is awake — right after a press, during that stay-awake
window — it runs a small web page at its normal network IP (check your
router's connected-devices list, or `pio device monitor`). It shows the
pedal's IP/Wi-Fi network, time since the last trigger, seconds left before
it sleeps, and a **live pedal input indicator** (PRESSED/RELEASED,
updating a few times a second) so you can watch the raw switch state
change as you press/release it — handy for checking wiring and contact
bounce without a multimeter — plus:

- A **Send test trigger now** button, to confirm the PC app is receiving
  triggers without needing to walk over and press the pedal.
- A settings form to change the **trigger UDP port** and the **stay-awake
  duration** (10–3600 seconds) — applied immediately and saved to flash,
  no reflash or Wi-Fi reconfiguration needed.

Loading the page or clicking either button also resets the sleep
countdown. This page only exists while the pedal is awake — once it goes
back to deep sleep it disappears from the network entirely until the next
press; there's no way to wake it remotely.

## Known limitations / possible follow-ups

- No RTC-memory Wi-Fi channel/BSSID caching yet, which could speed up
  reconnection after a long deep sleep - left out of this first version
  for simplicity.
- No physical low-battery indicator.
- No active discovery/handshake with the PC - it just broadcasts and
  trusts the PC app to be listening. If a trigger doesn't arrive, check
  Windows Firewall first (see rigol-speaking-multimeter's README); the
  pedal side logs whether its own send succeeded (`udp.endPacket()`
  result) over serial either way.
