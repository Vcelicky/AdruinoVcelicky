//libraries definition
#include <DHT.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HX711.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_TEAPOT
#define LED_PIN 13

//definition of variables for battery status
float Aref = 1.2;
unsigned int total;
float voltage;
int percentage;

//definition of variables for MPU6050
MPU6050 mpu;

bool blinkState = false;
bool dmpReady = false;
volatile bool mpuInterrupt = false;

uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

Quaternion q;

VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;

VectorFloat gravity;

float euler[3];
float ypr[3];
float humidity;
float temperature;
float humidity2;
float temperature2;

int chk; 

void dmpDataReady() {
  mpuInterrupt = true;
}

//definition of variables for time measure
unsigned long currentTimeAkc;
unsigned long startTimeAkc;
unsigned long currentTime;
unsigned long startTime;

//definition of variables for DHT22
DHT dht(52, DHT22);
DHT dht2(53, DHT22);

//definition of variables for load sensor
HX711 scale(4, 5);  

void setup() {

  // use the internal ~1.1volt reference
  analogReference(INTERNAL1V1);

  //join I2C bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  //set speed of serial communication 
  Serial.begin(115200);
  while (!Serial); 

  //initialize device
  Serial.println(F("Initializing I2C devices"));
  mpu.initialize();

  //testing connection
  Serial.println(F("Testing device connections"));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  //waiting for read any character
  Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read());
  while (!Serial.available());
  while (Serial.available() && Serial.read());

  //load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //set gyro and accel offsets
  mpu.setXAccelOffset(-607);
  mpu.setYAccelOffset(1837);
  mpu.setZAccelOffset(3750); //1788
  
  mpu.setXGyroOffset(22); //220
  mpu.setYGyroOffset(56); //76
  mpu.setZGyroOffset(5); //-85

  //check if it work properly
  if (devStatus == 0) {

    //turn on DMP
    Serial.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);

    //enable interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)"));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    //set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt"));
    dmpReady = true;
  
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //setting control LED 
  pinMode(LED_PIN, OUTPUT);

  //initialize devices
  dht.begin();
  dht2.begin();

  //measure start time
  startTime = millis();
}

void loop() {

  //if program fails
  if (!dmpReady) {
    Serial.println("END");
    return;
  }

  //wait for MPU interrupt
  while (!mpuInterrupt && fifoCount < packetSize) {

    //measuring actual time
    currentTime = millis();

    //repeated every 2s
    if(currentTime >=  (startTime + 2000)){

      //setting LED status
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);

      //measuring first DHT22   
      humidity = dht.readHumidity();
      temperature = dht.readTemperature();
    
      //measuring second DHT22
      humidity2 = dht2.readHumidity();
      temperature2 = dht2.readTemperature();

      //reading value from analog input to clear old input
      analogRead(1);

      //measuring input voltage
      for (int x = 0; x < 16; x++) { 
        total = total + analogRead(1);
      }

      //converting input voltage to voltage and calculatio percentage
      voltage = total * Aref / 1024;
      percentage = (voltage - 6) * 33.3;

      //printing values
      Serial.println();
      Serial.println("--------------------------DHT22--------------------------");
      Serial.print("Humidity(IN): ");
      Serial.print(humidity);
    
      Serial.print(" %, Temperature(IN): ");
      Serial.print(temperature);
      Serial.println(" C");
    
      Serial.print("Humidity(OUT): ");
      Serial.print(humidity2);
    
      Serial.print(" %, Temperature(OUT): ");
      Serial.print(temperature2);
      Serial.println(" C");
      Serial.println("--------------------------DHT22--------------------------");
      Serial.println();
      
      Serial.println("-------------------------Battery-------------------------");
      Serial.print("Voltage: ");
      Serial.print(voltage);
      Serial.println(" V");

      //warnning for low battery
      if(percentage <= 20){
        Serial.println("Warnning: Low battery");
      }
        
      Serial.print("Percentage: ");
      
      if(percentage > 100)
        Serial.print(100);
      if(percentage < 0)
        Serial.print(0);  
      else if(percentage <= 100 && percentage >= 0) 
        Serial.print(percentage);
      
      Serial.println(" %");
      Serial.println("-------------------------Battery-------------------------");
      Serial.println(); 
      total = 0;

      //measuring actual time
      startTime = millis();
    }
  }

  //reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  //get FIFO count
  fifoCount = mpu.getFIFOCount();

  //checking for overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
  }

  //checking for DMP data ready interrupt
  else if (mpuIntStatus & 0x02) {

    // read a packet from FIFO
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        currentTimeAkc = millis();
        if(currentTimeAkc >= (startTimeAkc + 2000))
        {

          Serial.println();
          Serial.println("----------------------Accelerometer----------------------");
          //Serial.print("Z:  ");
          //Serial.print(ypr[0] * 180 / M_PI);
          Serial.print("X:  ");
          Serial.print(ypr[1] * 180 / M_PI);
          Serial.print("°, Y:  ");
          Serial.print(ypr[2] * 180 / M_PI);
          Serial.println("°");
          Serial.println();
          
          if((ypr[1] * 180 / M_PI > 10) || (ypr[2] * 180 / M_PI > 10)  || (ypr[1] * 180 / M_PI < -10) || (ypr[2] * 180 / M_PI < -10)){
            Serial.println("State: hive was moved");
          }

          else Serial.println("State: hive is on right place");

          Serial.println("----------------------Accelerometer----------------------");
          Serial.println();
          
          startTimeAkc = millis();
        }
    #endif
  
    #ifdef OUTPUT_TEAPOT
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++;
    #endif
  }
}

