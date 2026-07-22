# COSMOS-2026-SOLAR

Dual-axis ESP32-WROVER solar tracker with four LDRs, two servos, and
ThingSpeak telemetry.

## Arduino setup

Install these libraries through **Arduino IDE > Library Manager**:

- `ESP32Servo`
- `ThingSpeak` by MathWorks

Select **ESP32 Wrover Module** as the board and use a 115200 baud Serial
Monitor.

## Wi-Fi and ThingSpeak credentials

Edit the local `secrets.h` file:

```cpp
#define SECRET_SSID "your-wifi-name"
#define SECRET_PASS ""
#define SECRET_CH_ID 1234567UL
#define SECRET_WRITE_APIKEY "your-channel-write-api-key"
```

Leave `SECRET_PASS` empty for an open guest network. If the guest network opens
a browser page that asks you to accept terms or sign in, the ESP32 cannot pass
that captive portal automatically; use a normal open/WPA network or a hotspot.

The channel ID is a number without quotation marks. Find the Write API key on
the ThingSpeak channel's **API Keys** tab. `secrets.h` is ignored by Git so the
password and key are not uploaded. `secrets.example.h` is the safe template.

Configure the ThingSpeak channel fields as follows:

1. Top-left LDR raw ADC
2. Top-right LDR raw ADC
3. Bottom-left LDR raw ADC
4. Bottom-right LDR raw ADC
5. Pan servo command
6. Tilt servo command

Lower LDR ADC values mean brighter light with the documented voltage-divider
wiring. The ESP32 tracks locally every 20 ms, prints readings every 100 ms,
reconnects to Wi-Fi automatically, and uploads all six fields every 20 seconds.
A successful ThingSpeak upload prints status code `200` in the Serial Monitor.

## Automatic joint-limit recovery

Normal tracking uses large command steps for a large LDR imbalance and
one-command-unit steps near alignment. The four readings also pass through a
fast first-order low-pass filter (`tau = 80 ms`), so flashlight changes appear
quickly without letting single ADC spikes shake the panel.

If an LDR error continues requesting motion after a servo reaches command `0`
or `180` for 400 ms, the tracker stops requesting that direction and enters
recovery. It does the same if pan feedback is trapped in the small straight-up
singularity:

1. At a pan limit, it first tries an over-the-top reindex: tilt to the pan axis,
   move pan to the opposite endpoint, and mirror the tilt angle.
2. It measures the real LDR result and keeps that pose only if its combined
   brightness-and-balance score improves by at least 3%.
3. If the reindex does not help, or the tilt axis is blocked, it performs a
   nonblocking serpentine 5x5 search across the configured servo command range,
   remembers the best measured pose, returns there, and resumes fine tracking.
4. An 8-second cooldown prevents an unreachable light source from causing
   continuous full-range scans.

The Serial Monitor prints `mode=TRACK`, `mode=FLIP-*`, or `mode=SCAN-*` so the
behavior is visible during testing. Wi-Fi, telemetry, sensor updates, and serial
output continue while recovery is running.

`TILT_AXIS_ALIGNMENT_COMMAND` in `lcd.ino` must be calibrated to the command
where the panel normal is parallel to the bottom pan axis. `PAN_DIRECTION` and
`TILT_DIRECTION` still select the motor wiring/mounting signs. The four LDRs
should have a small cross-shaped opaque divider between them; without it, all
four corners can receive nearly the same light even when the panel is not aimed
at the source.

### Servo command range versus physical rotation

`Servo.write(0..180)` uses normalized command units; those numbers are not a
measurement of the shaft angle. In this sketch, command `0` maps to a 1000 us
pulse and command `180` maps to 2000 us. A positional servo can physically move
120, 180, 270, or another number of degrees over that command range.

A normal three-wire servo sends no shaft-position or stall feedback to the
ESP32. Therefore, the firmware cannot safely detect the exact instant the motor
hits a physical stop. It now uses the entire configured 1000--2000 us range and
tries the opposite direction when the command endpoint cannot satisfy the LDRs.
Do not expand the pulse range just to force more movement; use the servo's data
sheet. Exact physical-stop detection requires a feedback servo, encoder, limit
switches, or a properly designed current sensor and cutoff.

Two positional servos still cannot reach a direction outside their mechanical
workspace. The search chooses the best reachable pose and avoids repeatedly
requesting motion past a command endpoint; guaranteed full-azimuth coverage
requires wider-travel positioning hardware with position feedback.
