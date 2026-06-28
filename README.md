# SPARK-batata-POLYMAZE-2026
A maze solving robot, for POLYMAZE robotics competition.
This robot discovers and solves the maze autonomously.
I've used an ESP32-S3 MCU, TB6612FNG motor driver, 2 x Pololu Micrometal Gearmotors with Encoders (50:1, 650 RPM), Pololu QTR-MD-13RC Reflectance Sensor Array, 2200 mAh Lipo Battery and a Buck Converter.
i've also designed my own chassis using solidworks and used a resin 3D printer for high precision.

# Demonstration-Videos
https://github.com/user-attachments/assets/75726a70-f94f-48bb-b9f4-54cb48ecce05
#
https://github.com/user-attachments/assets/8409f5ba-ea20-430d-aaf1-ed6198ea9565

# Improvements
there are many improvements to be addressed here, for example the fact that i didn't use buttons to control my robot and decide what to do with it, and the fact that i didn't use the EEPROM memory to save the shortest path,
and of course since i'm using an ESP32 i could've made a simple dashboard and communicate with my robot to check everything that's happening inside in real time, by checking the qtr sensors readings, the followed path and the shortest path, speed and PID adjustments ...
