# Teensy_Prop_Shield

Conglomeration of separate sketches with basic data output. Parameterizes registers, allows easy choice of data rates and full scale ranges, provides for simple bias calibration and serial output. Also tap, rotation, portrait/landscape detection. Gyro goes to sleep after a certain time without motion. Straightforward to configure for any individual application.

Added Madgwick and Mahony sensor fusion to get quaternions and Euler angles. Next need to finish the mag calibration. Fusion rate is 660 Hz when running the Teensy at 72 MHz.
