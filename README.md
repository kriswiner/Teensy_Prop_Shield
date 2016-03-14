# Teensy_Prop_Shield

Conglomeration of separate sketches with basic data output. Parameterizes registers, allows easy choice of data rates and full scale ranges, provides for simple bias calibration and serial output. Also tap, rotation, portrait/landscape detection. Gyro goes to sleep after a certain time without motion. Straightforward to configure for any individual application.

Added Madgwick and Mahony sensor fusion to get quaternions and Euler angles. All of the sensor data is properly scaled and calibrated for offset bias, and the Madgwick fusion algorithm produces quaternions and Euler angles derived therefrom at a rate of 780 - 840 Hz (depending on whether the accel drops into low-power mode = higher fusion rate) when running the Teensy at 72 MHz.
