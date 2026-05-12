# Slave Firmware

`esp32_slave_firmware.c` runs on the robotic arm ESP32 actuator node.

Responsibilities:

- Initialize Wi-Fi station mode and ESP-NOW reception.
- Print the board MAC address for master pairing.
- Receive compact servo command packets.
- Validate packet size.
- Drive three servo PWM outputs using the ESP32 LEDC peripheral.
- Move servos to a central fallback position if communication is lost.

Flash this firmware first so the master firmware can be configured with the correct receiver MAC address.
