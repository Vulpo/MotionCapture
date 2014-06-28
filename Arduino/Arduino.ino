const bool debug = false;
#define TEST if(debug)

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu_1;
MPU6050 mpu_2(0x69);  // <-- use for AD0 high

/* =========================================================================
NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
depends on the MPU-6050's INT pin being connected to the Arduino's
external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
digital I/O pin 2.
* ========================================================================= */

/* =========================================================================
NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
when using Serial.write(buf, len). The Teapot output uses this method.
The solution requires a modification to the Arduino USBAPI.h file, which
is fortunately simple, but annoying. This will be fixed in the next IDE
release. For more info, see these links:

http://arduino.cc/forum/index.php/topic,109987.0.html
http://code.google.com/p/arduino/issues/detail?id=958
* ========================================================================= */



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t forceReadMPU = 1;

uint8_t mpuIntStatus_1; // holds actual interrupt status byte from MPU
uint8_t devStatus_1; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize_1; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount_1; // count of all bytes currently in FIFO
bool mpuNeedToBeRead_1;
uint8_t fifoBuffer_1[64]; // FIFO storage buffer

uint8_t mpuIntStatus_2;
uint8_t devStatus_2;
uint16_t packetSize_2;
uint16_t fifoCount_2;
bool mpuNeedToBeRead_2;
uint8_t fifoBuffer_2[64];

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

SoftwareSerial xbee = SoftwareSerial(4, 5);
float measures[3];
byte serialBuffer[2*2*sizeof(measures)+1]; //two gyroscopes, two bytes per digit, 4 digit per measure + '\0'

// ================================================================
// === INTERRUPT DETECTION ROUTINE ===
// ================================================================

volatile bool mpuInterrupt_1 = false;     // indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt_2 = false;

void dmpDataReady_1() {
    mpuInterrupt_1 = true;
}
// Función que invoca cuando detecta una interrupción en el MPU 2
void dmpDataReady_2() {
    mpuInterrupt_2 = true;
}




// ================================================================
// === INITIAL SETUP ===
// ================================================================

void setup() {
  xbee.begin(19200);
  serialBuffer[2*2*sizeof(measures)] = 0x00; // set '\0' at the end of the array, for Serial.print

    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
#endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(19200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle the 115200 baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu_1.initialize();
    mpu_2.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu_1.testConnection() ? F("MPU6050_1 connection successful") : F("MPU6050_1 connection failed"));
    Serial.println(mpu_2.testConnection() ? F("MPU6050_2 connection successful") : F("MPU6050_2 connection failed"));

    // wait for ready
    /*
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available()); // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  */
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus_1 = mpu_1.dmpInitialize();
    devStatus_2 = mpu_2.dmpInitialize();    
    
    if (devStatus_1 == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu_1.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady_1, RISING); // Utilizamos la primera interrupción externa (número 0) que está en el pin digital 2
                                                     // Cuando la interrupción tiene lugar invoca la función "dmp_1_DataReady"
                                                     // RISING dispara la interrupción cuando el pin pasa de valor alto (HIGH) a bajo (LOW)
        mpuIntStatus_1 = mpu_1.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));

        // get expected DMP packet size for later comparison
        packetSize_1 = mpu_1.dmpGetFIFOPacketSize();
        
            // supply your own gyro offsets here, scaled for min sensitivity
        mpu_1.setXGyroOffset(17);
        mpu_1.setYGyroOffset(-22);
        mpu_1.setZGyroOffset(33);
        mpu_1.setXAccelOffset(2310);
        mpu_1.setYAccelOffset(1823);
        mpu_1.setZAccelOffset(1477);
        
    }else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP 1 Initialization failed (code "));
        Serial.print(devStatus_1);
        Serial.println(F(")"));
    }

    if (devStatus_2 == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu_2.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(1, dmpDataReady_2, RISING); // Utilizamos la segunda interrupción externa (número 1) que está en el pin digital 3
                                                     // Cuando la interrupción tiene lugar invoca la función "dmp_1_DataReady"
                                                     // RISING dispara la interrupción cuando el pin pasa de valor alto (HIGH) a bajo (LOW)        
        mpuIntStatus_2 = mpu_2.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize_2 = mpu_2.dmpGetFIFOPacketSize();

        mpu_1.setXGyroOffset(-5);
        mpu_1.setYGyroOffset(-60);
        mpu_1.setZGyroOffset(69);
        mpu_1.setXAccelOffset(-1088);
        mpu_1.setYAccelOffset(580);
        mpu_1.setZAccelOffset(1936);
        
    }else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP 2 Initialization failed (code "));
        Serial.print(devStatus_2);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// === MAIN PROGRAM LOOP ===
// ================================================================
uint8_t ctrl=0;
void loop() {
      //------------------
      TEST{
        if(ctrl>=10){
          Serial.println('|');
          ctrl=0;
              // blink LED to indicate activity
              blinkState = !blinkState;
              digitalWrite(LED_PIN, blinkState);
        }
        else{
          Serial.print('|');
          ctrl++;
        }
      } else{
          digitalWrite(LED_PIN, true); 
      }
      //------------------
      
     // if programming failed, don't try to do anything
     if (!dmpReady) return;

     // wait for MPU interrupt or extra packet(s) available.
     //Processing can be done meanwhile to reduce the processing time (wait+process merged)
      TEST Serial.print('1');
     while ( !mpuInterrupt_1 && fifoCount_1 < packetSize_1){
         delay(20) ;
     }
      TEST Serial.print('a');
     mpuInterrupt_1 = false;
     mpuIntStatus_1 = mpu_1.getIntStatus();
     fifoCount_1 = mpu_1.getFIFOCount();
      TEST Serial.print('b');
     // check for overflow (this should never happen unless our code is too inefficient)
     if ((mpuIntStatus_1 & 0x10) || fifoCount_1 == 1024) {
         // reset so we can continue cleanly
         mpu_1.resetFIFO();
         Serial.println(F("FIFO 1 overflow!"));
          TEST Serial.print('$');
     // otherwise, check for DMP data ready interrupt (this should happen frequently)
     } else if (mpuIntStatus_1 & 0x02) {
         // wait for correct available data length, should be a VERY short wait
         while (fifoCount_1 < packetSize_1) fifoCount_1 = mpu_1.getFIFOCount();
          TEST Serial.print('c');
         // read a packet from FIFO
         if (packetSize_1 >= 64) Serial.println("Packet size incoherence 1");
         mpu_1.getFIFOBytes(fifoBuffer_1, packetSize_1);
        
         // track FIFO count here in case there is > 1 packet available
         // (this lets us immediately read more without waiting for an interrupt)
         fifoCount_1 -= packetSize_1;
         //mpu_1.resetFIFO();// Parce que j'arrive pas à trouver une fonction setFIFOsize(). Juste espérer qu'elle ne se remplit pas pendant l'exécution de la ligne précédente...
          TEST Serial.print('d');
     }
     serializeMeasuresToBuffer(1);
    
      TEST Serial.print('2');
     while ( !mpuInterrupt_2 && fifoCount_1 < packetSize_2){
         delay(20) ;
     }
      TEST Serial.print('a');
  
     // reset interrupt flag and get INT_STATUS byte
     mpuInterrupt_2 = false;
     mpuIntStatus_2 = mpu_2.getIntStatus();
      TEST Serial.print('b');
  
     // check for overflow (this should never happen unless our code is too inefficient)
     if ((mpuIntStatus_2 & 0x10) || fifoCount_2 == 1024) {
         // reset so we can continue cleanly
         mpu_2.resetFIFO();
         Serial.println(F("FIFO 2 overflow!"));
      TEST Serial.print('£');
     // otherwise, check for DMP data ready interrupt (this should happen frequently)
     } else if (mpuIntStatus_2 & 0x02) {
         // wait for correct available data length, should be a VERY short wait
      
         while (fifoCount_2 < packetSize_2) fifoCount_2 = mpu_2.getFIFOCount();
          TEST Serial.print('c');
         // read a packet from FIFO
         if (packetSize_2 >= 64) Serial.println("Packet size incoherence 2");
         mpu_2.getFIFOBytes(fifoBuffer_2, packetSize_2);
          
         // track FIFO count here in case there is > 1 packet available
         // (this lets us immediately read more without waiting for an interrupt)
         fifoCount_2 -= packetSize_2;
         //mpu_2.resetFIFO();// Parce que j'arrive pas à trouver une fonction setFIFOsize(). Juste espérer qu'elle ne se remplit pas pendant l'exécution de la ligne précédente...
      TEST Serial.print('d');
    }
  
    serializeMeasuresToBuffer(2);
    sendBuffer();
    TEST Serial.print('.');
}

void serializeMeasuresToBuffer(uint8_t index){
    if (index != 1 && index != 2){
       Serial.println("Error index.");
       return;
    }
    
    if(index==1){
        mpu_1.dmpGetQuaternion(&q, fifoBuffer_1);
        mpu_1.dmpGetGravity(&gravity, &q);
        mpu_1.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
    else{
        mpu_2.dmpGetQuaternion(&q, fifoBuffer_2);
        mpu_2.dmpGetGravity(&gravity, &q);
        mpu_2.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
    
    measures[0] = ypr[2]*180/M_PI;
    measures[1] = ypr[0]*180/M_PI;
    measures[2] = ypr[1]*180/M_PI;
    
    index--;
    int partlength = (sizeof(serialBuffer)/2);
    index *= partlength; //gyro 1 in first half of buffer, gyro 2 at second half
        
    unsigned char *c = reinterpret_cast<unsigned char *>(&measures);
    for(int i=0; i<sizeof(measures); i++){
        serialBuffer[index+2*i] = ((*(c+i) & 0xF0)>>4)+0x30;
        serialBuffer[index+(2*i)+1] = (*(c+i) & 0x0F)+0x30;
    }
}

void sendBuffer(){              
    xbee.println(String((char*)serialBuffer));
    //Serial.println(String ((char*)serialBuffer));
}

