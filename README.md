# COSMOS-2026-SOLAR

Dual-axis ESP32-WROVER solar tracker with four LDRs, two servos, and
ThingSpeak telemetry.

## Arduino setup

Install these libraries through **Arduino IDE > Library Manager**:

- `ESP32Servo`
- `ThingSpeak` by MathWorks

Select **ESP32 Wrover Module** as the board and use a 115200 baud Serial
Monitor. Wi-Fi, HTTPS, NTP time, and Preferences storage come with the ESP32
board package; they do not require additional Library Manager downloads.

## Wi-Fi and ThingSpeak credentials

Edit the local `secrets.h` file:

```cpp
#define SECRET_SSID "your-wifi-name"
#define SECRET_PASS ""
#define SECRET_CH_ID 1234567UL
#define SECRET_WRITE_APIKEY "your-channel-write-api-key"
#define SECRET_LATITUDE 999.0
#define SECRET_LONGITUDE 999.0
```

Leave `SECRET_PASS` empty for an open guest network. If the guest network opens
a browser page that asks you to accept terms or sign in, the ESP32 cannot pass
that captive portal automatically; use a normal open/WPA network or a hotspot.

The channel ID is a number without quotation marks. Find the Write API key on
the ThingSpeak channel's **API Keys** tab. `secrets.h` is ignored by Git so the
password and key are not uploaded. `secrets.example.h` is the safe template.

Leave both coordinates at `999.0` for automatic public-IP geolocation. Enter
real latitude/longitude only when a more accurate fixed location is needed
(north/east positive, south/west negative).

Configure the ThingSpeak channel fields as follows:

1. Top-left LDR raw ADC
2. Top-right LDR raw ADC
3. Bottom-left LDR raw ADC
4. Bottom-right LDR raw ADC
5. Pan servo command
6. Tilt servo command
7. Estimated sun azimuth in degrees (`0` north, `90` east)
8. Estimated sun elevation in degrees (`0` horizon, `90` overhead)

Lower LDR ADC values mean brighter light with the documented voltage-divider
wiring. The ESP32 samples the sensors every 20 ms, prints readings every 100 ms,
reconnects to Wi-Fi automatically, and uploads all eight fields every 20
seconds. Geolocation and ThingSpeak requests run in background tasks, so a slow
guest network does not pause sensor sampling or a light scan. A successful
ThingSpeak upload prints status code `200` in the Serial Monitor.

## Automatic time, location, and direction

After Wi-Fi connects, the ESP32 synchronizes UTC from NTP, so the hour, day,
month, and year require no manual entry. It requests an approximate
latitude/longitude once from `ipwho.is` using the network's public IP and caches
the result in ESP32 Preferences. A failed request is retried after 10 minutes,
not every few seconds. Public-IP coordinates can identify an ISP, VPN endpoint,
or nearby population center rather than the exact panel, so use the optional
coordinates in `secrets.h` when accuracy matters.

For prototype portability, the ESP32 HTTPS client does not pin the
geolocation service's certificate. The response is strictly limited and parsed
as a short coordinate pair, but it is still a best-effort estimate and must not
be used for safety-critical positioning.

The firmware applies NOAA's compact solar equations every 30 seconds to
calculate geometric sun azimuth and elevation from true north. This estimate is
reported to Serial and ThingSpeak, but it never overrides the strict
brightest-light hold described below.

When both LDR errors remain within `+/-30` under strong light for two seconds,
the firmware can associate the held panel pose with the calculated sun
azimuth/elevation. That creates an approximate local direction anchor for
telemetry. A coarse brightest-light pose is not guaranteed to be balanced
within `+/-30`, so Serial may continue to show `panel direction=acquiring`; this
does not prevent the panel from remaining locked at the best measured pose.

This is the strongest automatic direction estimate possible with the existing
four LDRs and three-wire SG90 servos. The ESP32 has no compass, accelerometer,
GPS, servo encoder, or shaft feedback. Therefore it cannot instantly determine
true north, arbitrary base tilt, or exact physical servo angle before seeing
the sun. Reflections, indoor lamps, clouds, or moving the base can also create
a false/stale anchor. True placement-independent 3-D direction requires a
calibrated accelerometer/magnetometer (or 9-axis IMU); exact device location
requires GNSS.

References: [NOAA solar-position equations](https://gml.noaa.gov/grad/solcalc/solareqns.PDF),
[Espressif system-time/SNTP documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/system_time.html),
and [ipwho.is geolocation API documentation](https://ipwhois.io/documentation).

## Brightest-light scan and hold

One second after startup, the tracker performs one bounded serpentine 5x5 scan
covering the complete configured pan/tilt command range. At every grid point it
waits 400 ms for motion and the first-order LDR low-pass filter (`tau = 80 ms`)
to settle, then averages four sensor samples. The winning point is the one with
the greatest total measured light across all four LDRs. The tracker returns to
that point and changes to `mode=LIGHT-LOCK`.

`LIGHT-LOCK` is a strict hold: normal 20 ms sensor updates, Wi-Fi, sun-position
updates, and ThingSpeak uploads do not write new servo commands. The firmware
holds the chosen position for at least 30 seconds. After that, it permits one
new full scan only when either filtered horizontal or vertical LDR difference
has changed by at least 30 ADC counts from the locked pattern continuously for
five seconds. A brief flashlight pass or ADC spike therefore cannot start a
scan. Every completed rescan returns to its newly measured brightest point and
locks again.

Serial shows `mode=SCAN-*` during the bounded search, `mode=RETURN-BEST` while
returning, and `mode=LIGHT-LOCK` when no new servo commands are being sent.
Wi-Fi, telemetry, sensor updates, and serial output continue throughout.

The four LDRs should have a small cross-shaped opaque divider between them;
without it, all four corners can receive nearly the same light even when the
panel is not aimed at the source.

### Servo command range versus physical rotation

`Servo.write(0..180)` uses normalized command units; those numbers are not a
measurement of the shaft angle. For these TowerPro SG90 servos, command `0`
currently maps to a 500 us pulse and command `180` maps to 2500 us. This is an
experimental maximum range with twice the pulse span of the original
1000--2000 us setting; it does not guarantee 180 degrees of physical movement.

A normal three-wire servo sends no shaft-position or stall feedback to the
ESP32. Therefore, the firmware cannot safely detect the exact instant the motor
hits a physical stop. It uses the entire configured 500--2500 us range during
the bounded scan and does not command past either endpoint. Do not expand the
pulse range farther. Exact physical-stop detection requires a feedback servo,
encoder, limit switches, or a properly designed current sensor and cutoff.

For the first powered test, support or disconnect the panel load and watch each
servo near both endpoints. If either unit buzzes, chatters, becomes warm, or
stops moving before the command reaches an endpoint, cut servo power immediately
and move that axis's `*_SERVO_MIN_PULSE_US` or `*_SERVO_MAX_PULSE_US` inward by
50--100 us. Power the servos from a separate regulated 4.8--5 V supply capable
of their current draw, with its ground connected to ESP32 ground.

Two positional servos still cannot reach a direction outside their mechanical
workspace. The scan chooses the best reachable pose; guaranteed full-azimuth
coverage requires wider-travel positioning hardware with position feedback. A
true continuous-rotation servo cannot scan and return to an absolute angle
without an encoder. The expected TowerPro SG90 is a positional servo.
