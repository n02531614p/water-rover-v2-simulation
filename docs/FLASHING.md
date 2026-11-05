# Flashing the ESP32 (Arduino IDE 1.8.19)

1. **Add the ESP32 boards.** File → Preferences → *Additional Boards Manager URLs*, paste
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json` → OK.
   Tools → Board → Boards Manager → search **esp32** → install **esp32 by Espressif Systems** (latest 3.x).
2. **Select the board.** Tools → Board → ESP32 Arduino → **ESP32 Dev Module**. Upload Speed 921600
   (drop to 115200 if uploads fail).
3. **Install the libraries.** Sketch → Include Library → Manage Libraries → install **PubSubClient**
   (Nick O'Leary) and **ESP32Servo** (Kevin Harrington).
4. **Port.** Plug the ESP32 in with a *data* USB cable (many cables are charge-only). Tools → Port → the new
   COM port. No port appears? Install the USB driver for the chip printed near the USB socket:
   **CP2102** → Silicon Labs CP210x driver, **CH340** → WCH CH340 driver. Re-plug.
5. **Open the sketch.** Download the repo (Code → Download ZIP), unzip, and open
   `firmware/waterdroid_esp32/waterdroid_esp32.ino`. Keep the folder name `waterdroid_esp32` and the `src/`
   folder next to the `.ino`.
6. **Set Wi-Fi.** At the top of the sketch set `WIFI_SSID`, `WIFI_PASS` and `MQTT_HOST` (the PC running
   Mosquitto / Node-RED).
7. **Upload** (→ button). If it stops at `Connecting........_____`, hold the **BOOT** button on the ESP32
   until the upload starts.
8. **Check it.** Tools → Serial Monitor, **115200 baud**. You should see
   `{"event":"boot","fw":"waterdroid_esp32"}` and then one status line per second.

The rover can be flashed with only USB connected; keep the motor/pump rail switched off until the boot
line appears and every relay is off.
