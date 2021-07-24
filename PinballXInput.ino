/* 
 *  Project     Pinball XInput
 *  @author     H.R.Graf
 *  @link       github.com/hrgraf/PinballXInput
 *  @license    MIT - Copyright (c) 2021 H.R.Graf
 * 
 * Emulates an Xbox 360 Controller for PC-based Pinball Games.
 * Using an Arduino Leonardo (resp. Pro Micro clone) and a MPU6050 accelerometer.
 * 
 * Based on examples/libraries from 
 *   David Madison (https://github.com/dmadison/ArduinoXInput)
 *   Jeff Rowberg (https://github.com/jrowberg/i2cdevlib)
 *   
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// ================================================================

// Wiring
#if defined(ARDUINO_AVR_LEONARDO)
  #define LED_PIN 17 // RX LED
#else
  #define LED_PIN LED_BUILTIN 
#endif
#define FLIP_L_PIN 18
#define FLIP_R_PIN 19

// I2C
#define MPU_ADDR 0x68 // AD0 low (default)
//#define MPU_ADDR 0x69 // AD0 high

// nudge sensitivity 
#define N_MAX 1000 // higher limit requires higher acceleration

// ================================================================

#include <XInput.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

static bool blinkState = false;

// MPU/DMP
static MPU6050 mpu(MPU_ADDR);
static uint8_t fifoBuffer[64];
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

// orientation/motion vars
static Quaternion q;           // [w, x, y, z]         quaternion container
static VectorInt16 aa;         // [x, y, z]            accel sensor measurements
static VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
static VectorFloat gravity;    // [x, y, z]            gravity vector

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{
    // setup IOs
    pinMode(LED_PIN, OUTPUT);
    pinMode(FLIP_L_PIN, INPUT_PULLUP);
    pinMode(FLIP_R_PIN, INPUT_PULLUP);

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial) // wait for Leonardo enumeration, others continue immediately
      ; 
    Serial.println("Pinball XInput");

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        #ifndef USB_XINPUT
            Wire.setWireTimeout(10000, 0);
        #endif
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // show wiring
    Serial.print("LED pin ");
    Serial.println(LED_PIN);
    
    Serial.print("Flipper pin L=");
    Serial.print(FLIP_L_PIN);
    Serial.print(", R=");
    Serial.println(FLIP_R_PIN);
    
    Serial.print("I2C pin SDA=");
    Serial.print(SDA);
    Serial.print(", SCL=");
    Serial.println(SCL);

    // start XInput
    XInput.setAutoSend(false);  // disable automatic output
    XInput.setRange(JOY_LEFT, -N_MAX, N_MAX); // nudge range
    XInput.begin();
  
    // initialize accelerometer
    Serial.println("Initializing MPU6050 ...");
    mpu.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // load and configure the DMP
    Serial.println("Initializing DMP...");
    static uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) // OK
    {
        // Calibration Time: generate offsets and calibrate MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println("Enabling DMP...");
        mpu.setDMPEnabled(true);
        Serial.println("DMP ready!");
        dmpReady = true;
        
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.print("DMP packetSize ");
        Serial.println(packetSize);
    }
    else // ERROR
    {
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() 
{
    boolean flip_l = !digitalRead(FLIP_L_PIN);
    boolean flip_r = !digitalRead(FLIP_R_PIN);
    
    XInput.setButton(BUTTON_LB, flip_l);
    XInput.setButton(BUTTON_RB, flip_r);
    XInput.send();  // send data over USB

    if (!dmpReady) 
        return;

    #if 1
        uint16_t fifoCount = mpu.getFIFOCount();
        if (fifoCount < packetSize)
            return;
        #ifndef USB_XINPUT
            Serial.print("fifo\t");
            Serial.print(fifoCount);
            Serial.print("\t");
        #endif
        if (fifoCount > 200)
        {
            Serial.println("overflow");
            mpu.resetFIFO();
            return;
        }
    
        mpu.getFIFOBytes(fifoBuffer, packetSize); //Get 1 packet
    #else
        // read a packet from DMP FIFO
        if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
            return;
    #endif
    
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    #ifndef USB_XINPUT
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
    #else
        // nudge with left stick
        short nx =  aaReal.y;
        short ny = -aaReal.x;

        XInput.setJoystick(JOY_LEFT, nx, ny); // nudge
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

// ================================================================
