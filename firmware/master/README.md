# Master Firmware

`esp32_master_firmware.c` runs on the wearable ESP32 sensor node.

Responsibilities:

- Read MPU6050 acceleration data through I2C.
- Read elbow potentiometer data through ADC.
- Estimate pitch and roll.
- Apply a simple complementary filter.
- Map calibrated motion ranges into servo PWM duty cycles.
- Send the command packet to the slave ESP32 using ESP-NOW.
- Print calibration and transmission diagnostics over serial.

Before flashing, update `MAC_ESCLAVO` with the station MAC address printed by the slave firmware.
