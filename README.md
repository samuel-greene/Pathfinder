# Pathfinder

## Hiking GPS Telemetry & Navigation Display

An ESP32 firmware project for a handheld hiking navigation device. It continuously parses NMEA GPS data over UART and renders field-relevant state on an SSD1306 OLED over I2C. The current firmware is structured around a non-blocking loop so GPS byte ingestion is never delayed by UI work.

## Current capabilities

- GPS health state machine: distinguishes no UART/NMEA data, satellite acquisition, stale position data, and a live fix.
- Three automatically rotating screens: navigation, telemetry, and hike statistics.
- Waypoint guidance engine (distance and bearing) gated behind explicit configuration, avoiding a misleading default destination.
- Live telemetry for position, speed, altitude, satellite count, HDOP, and fix freshness.
- Approximate track-distance accumulator that samples every two seconds and rejects implausibly large GPS jumps.
- Serial diagnostics for NMEA ingestion, checksum failures, satellite visibility, HDOP, GPS state, and tracking data.
- 4 Hz OLED updates on an independent `millis()` schedule.

## Hardware

- ESP32-compatible board
- UART GPS module (9600 baud by default; RX `D7`, TX `D6`)
- 128x64 SSD1306 I2C OLED (`0x3C`)

## Configuring a waypoint

In `gps_device.ino`, set `Config::kWaypointEnabled` to `true`, then provide a latitude, longitude, and descriptive name. Until that is done, the navigation screen shows live coordinates rather than implying a configured route.

## Next hardware milestone

The intended product direction is a compact, battery-powered hiking device with a 200x300 e-paper display, integrated GPS antenna, offline topographic rendering, and locally stored trail/point-of-interest information. These are planned features, not yet implemented in this firmware.