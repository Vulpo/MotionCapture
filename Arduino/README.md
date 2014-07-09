Software to collect filtered rotation values of two IMUs, and send it to computer.

#Notes&TODO:
- Calibration depends on the IMU used (factory settings) and initial position. Auto-calibration algorithm at start should be implemented. Calibration is done when the raw data (in quaternions) are gx=0, gy=0, gz=0 ax=0, ay=0, az=g. 
- Data is read when an interrupt from the MPU is received (expected interarrival time: 40ms, 25Hz). Spurrious data may occure at one frame, with no consequence for the following because the MPU'smeasurements FIFO is reset after every time it is read. This glitch would maybe not happen if the measurements were actively requested every 40ms (do not use delay(), but a active while loop checking the time difference! because delay(40)+instructionsDelay>40ms) but might be trickier to secure against corrupted&lost datas. 

#External Libraries:
- Jeff Rowberg’s I2C lib for the MPU-6050

#Hardware:
- Arduino Uno
- Xbee S1
- MPU-6050 *2