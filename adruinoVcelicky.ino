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


MPU6050 mpu;

float Aref = 1.2;
unsigned int total;
float voltage;
int percentage;

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

unsigned long currentTimeAkc;
unsigned long startTimeAkc;
unsigned long currentTime;
unsigned long startTime;

int chk; 

void dmpDataReady() {
  mpuInterrupt = true;
}

DHT dht(52, DHT22);
DHT dht2(53, DHT22);

HX711 scale(A1, A0);  

void setup() {
  
  analogReference(INTERNAL1V1);
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;
  
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  while (!Serial); 
  
  Serial.println(F("Initializing I2C devices"));
  mpu.initialize();
  
  Serial.println(F("Testing device connections"));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read());
  while (!Serial.available());
  while (Serial.available() && Serial.read());
 
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
    
  mpu.setXAccelOffset(-607);
  mpu.setYAccelOffset(1837);
  mpu.setZAccelOffset(3750); //1788
  
  mpu.setXGyroOffset(22); //220
  mpu.setYGyroOffset(56); //76
  mpu.setZGyroOffset(5); //-85
  
  if (devStatus == 0) {
    Serial.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);
  
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)"));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
  
    Serial.println(F("DMP ready! Waiting for first interrupt"));
    dmpReady = true;
  
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  pinMode(LED_PIN, OUTPUT);

  dht.begin();
  dht2.begin();

  startTime = millis();
}

void loop() {
  
  if (!dmpReady) {
    Serial.println("END");
    return;
  }
  
  while (!mpuInterrupt && fifoCount < packetSize) {
    currentTime = millis();
    
    if(currentTime >=  (startTime + 2000)){
       
      humidity = dht.readHumidity();
      temperature = dht.readTemperature();
    
      humidity2 = dht2.readHumidity();
      temperature2 = dht2.readTemperature();

      analogRead(1);
      
      for (int x = 0; x < 16; x++) { 
        total = total + analogRead(1);
      }
      
      voltage = total * Aref / 1024;
      percentage = (voltage - 6) * 33.3;

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

      
      if(percentage <= 20){
        Serial.println("Warnning: Low battery");
      }
        
      Serial.print("Percentage: ");
      if(percentage > 100)
        Serial.print(100);  
      else Serial.print(percentage);
      Serial.println(" %");
      Serial.println("-------------------------Battery-------------------------");
      Serial.println(); 
      total = 0;
      
      startTime = millis();
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
  }
   
  else if (mpuIntStatus & 0x02) {
    
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
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
    
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

