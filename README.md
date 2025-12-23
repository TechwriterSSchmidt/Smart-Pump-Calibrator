# Pump Volume Test (ESP32-S3)

Ein kleines Testprogramm zur Steuerung einer Pumpe über einen MOSFET mit einem ESP32-S3 DevKit.

## Hardware
*   **Board**: ESP32-S3 DevKitC-1 (N16R8)
*   **Pumpe**: Angeschlossen an MOSFET Modul
*   **MOSFET Pin**: GPIO 4
*   **Taster**: GPIO 5 (gegen GND)
*   **Status LED**: Onboard WS2812 (GPIO 48)

## Funktion
1.  **Bereit (Grün)**: Warten auf Tasterdruck.
2.  **Aktiv (Gelb)**: Pumpe läuft für 5 Sekunden.
3.  **Stopp (Rot)**: Pumpe aus. Warten auf Reset durch Taster.

## Pinout Referenz
*   [OceanLabz ESP32-S3 Pinout](https://www.oceanlabz.in/esp32-s3-devkit-pinout-reference/)
