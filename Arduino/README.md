Software to collect filtered rotation values of two IMUs, and send it to computer.

#Notes&TODO:
- Calibration depends on the IMU used (factory settings) and initial position. Auto-calibration algorithm at start should be implemented. Calibration is done when the raw data (in quaternions) are gx=0, gy=0, gz=0 ax=0, ay=0, az=g. 
- Some TEST instructions are left i nthe code, they serve to debug when the first problem stated occurs (Arduino freezes). 

#External Libraries:
- Jeff Rowberg’s I2C lib for the MPU-6050

#Hardware:
- Arduino Uno
- Xbee S1
- MPU-6050 *2