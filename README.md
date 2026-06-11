# HAB-01 

HAB-01 is a High-Altitude Balloon (HAB) mission developed by LEAP Space Club to provide a low-cost near-space testing platform for aerospace, embedded systems, and CubeSat development.

The mission is designed to reach stratospheric altitudes of 30–35 km while collecting atmospheric and system-performance data in extreme environmental conditions including low pressure, low temperatures, and increased UV exposure.

## Mission Objectives

- Reach 30–35 km stratospheric altitude
- Record GPS position throughout ascent, float, and descent
- Transmit real-time telemetry using a 433 MHz LoRa link
- Log environmental data to onboard storage
- Characterize battery performance under cold-soak conditions
- Validate communication systems in near-space environments
- Pre-qualify CubeSat components using the HLCI-TP framework

## Payload Features

- STM32-based flight computer
- LoRa telemetry system
- GPS navigation and tracking
- Pressure sensing (MS5611)
- Temperature sensing (DS18B20)
- UV monitoring
- Magnetometer and accelerometer
- SD card data logging
- GSM backup tracking

## Ground Station

- Real-time telemetry dashboard
- LoRa receiver architecture
- WebSocket-based data pipeline
- Live map tracking using Leaflet.js

## HLCI-TP Framework

HAB-01 introduces the High-Altitude to LEO Correlation Index – Thermal Power (HLCI-TP), a quantitative framework that estimates how closely a balloon flight replicates Low Earth Orbit environmental stresses through:

- Thermal fatigue modeling
- Battery degradation analysis
- UV exposure correlation

## Vision

HAB-01 serves as the first step in LEAP Space Club's long-term CubeSat development roadmap, enabling affordable and repeatable near-space testing for future satellite missions.
