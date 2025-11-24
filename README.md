# nrf52dk-dht22
## Turn a simple interface to connect nrf52dk with DHT22 sensor into BLE (bluetooth low energy) peripheral app.

This project was originally taken from the following repository:
https://github.com/MaazSk/nrf7002dk-dht22

This project was also published in the following link:
https://www.hackster.io/maaz-shaikh/nrf7002dk-and-dht22-sensor-step-by-step-guide-805df7

It was written by Maaz Shaikh on August 15, 2025.

I run this app on my device nRF52_DK-nRF52832 and it is working successfully.

I used GitHub Copilot to turn the app into a BLE peripheral that advertises a custom Environmental Service with temperature and humidity characteristics (read + notify). The code periodically samples the DHT22, exposes the last-measured values for reads, and notifies subscribed centrals. It also handles connections and safe disconnection.

I managed to get it worked. I still continuing enhancing this app further using GitHub Copilot as an assistant.

Build info
SDK: nRF Connect SDK v2.9.2

  Flash runner: nrfjprog

  Build size: 2.40 kB

Input files
  Configuration:

    nrf52dk-dht22/prj.conf

  Overlays:

    nrf52dk-dht22/boards/nrf52dk_nrf52832.overlay
