//libraries definition
#include <DHT.h>
#include <SoftwareSerial.h>
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
float Aref = 1.315;   //1.2 or 1.315
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

int chk; 

void dmpDataReady() {
  mpuInterrupt = true;
}
bool movedHive;

//definition of variables for time measure
unsigned long currentTimeAkc;
unsigned long startTimeAkc;
unsigned long currentTime;
unsigned long startTime;
unsigned long startTimeWaitAkc;
long int interval;

//definition of variables for DHT22
DHT dht(52, DHT22);
DHT dht2(53, DHT22);

int humidity;
int temperature;
int humidity2;
int temperature2;

//definition of variables for load sensor
HX711 scale(4, 5); 
int weight; 

//definition of variables for SigFox modem
#define TX 12
#define RX 13

//initialization of software serial link
SoftwareSerial Sigfox(RX, TX);

String command;
String message;

void setup() {

  interval = 600000;
  
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
  /*Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read());
  while (!Serial.available());
  while (Serial.available() && Serial.read());*/
  
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
  startTimeWaitAkc = millis();

  //set default state of hive
  movedHive = false;

  //starting software serial link 
  Sigfox.begin(9600);
  Serial3.begin(9600);
}

void loop() {

  //if program fails
  if (!dmpReady) {
    Serial.println("END");
    return;
  }
    
  //wait for MPU interrupt
  while (!mpuInterrupt && fifoCount < packetSize) {

    if (Serial3.available()) {
      message = Serial3.readString();
    }    

    message.trim();
    if(!message.equals("")){
      Serial.println("----------------------SIGFOX-MESSAGE---------------------");
      Serial.println(message);
      Serial.println("----------------------SIGFOX-MESSAGE---------------------");
      message = "";
    }
    
    if (Serial.available()) {
     command = Serial.readString();
    }
  
    command.trim();
    if(!command.equals("")){
      Serial.println("----------------------SIGFOX-COMMAND---------------------");
      command = command + "\r";
      Serial.println(command);
      Serial.println("----------------------SIGFOX-COMMAND---------------------");
      
      Serial3.println(command);
      command = "";
    }

    
    //measuring actual time
    currentTime = millis();

    //repeated every 2s
    if(currentTime >=  (startTime + interval + 500)){

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
      
      if(percentage >= 100){
        percentage = 100;
        Serial.print(percentage);
      }
      if(percentage <= 0){
        percentage = 0;
        Serial.print(percentage); 
      } 
      else if(percentage < 100 && percentage > 0) 
        Serial.print(percentage);
      
      Serial.println(" %");
      Serial.println("-------------------------Battery-------------------------");
      Serial.println(); 
      total = 0;

      Serial.println("-------------------------Message-------------------------");
      messageConvert();
      Serial.println("-------------------------Message-------------------------");
      
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

    //read a packet from FIFO
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
        //display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        currentTimeAkc = millis();
        if((millis() > startTimeWaitAkc + 10000) && (currentTimeAkc >= (startTimeAkc + interval)))
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

          if(movedHive == false){
            
            if((ypr[1] * 180 / M_PI > 8) || (ypr[2] * 180 / M_PI > 8)  || (ypr[1] * 180 / M_PI < -8) || (ypr[2] * 180 / M_PI < -8)){
              movedHive = true;
              Serial.println("State: hive was moved");
            }
            else Serial.println("State: hive is on right place");          
          }
          else Serial.println("State: hive was moved");
          

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

void messageConvert() {
  
    Serial.println();
    char binTemperature[9] = {0};
    
    if(temperature < 0){
      temperature = (-temperature) + 128;
      itoa((byte) temperature, binTemperature, 2);
      }
    else{
      itoa((byte) temperature, binTemperature, 2);
      }
      
    Serial.print("Outside temperature -> ");
    Serial.print(binTemperature);
    Serial.print(":");
    Serial.println(strlen(binTemperature));

    char binTemperature2[9] = {0}; 
    
    if(temperature2 < 0){
      temperature2 = (-temperature2) + 128;
      itoa((byte) temperature2, binTemperature2, 2);
      }
    else{
      itoa((byte) temperature2, binTemperature2, 2);
      }
      
    Serial.print("Inside temperature -> ");
    Serial.print(binTemperature2);
    Serial.print(":");
    Serial.println(strlen(binTemperature2));

    char binHumidity[9] = {0}; 
    itoa((byte) humidity, binHumidity, 2);
    Serial.print("Outside humidity -> ");
    Serial.print(binHumidity);
    Serial.print(":");
    Serial.println(strlen(binHumidity));

    char binHumidity2[9] = {0}; 
    itoa((byte) humidity2, binHumidity2, 2);
    Serial.print("Inside humidity -> ");
    Serial.print(binHumidity2);
    Serial.print(":");
    Serial.println(strlen(binHumidity2));

    char binPercentage[9] = {0}; 
    itoa((byte) percentage, binPercentage, 2);
    Serial.print("Battery percentage -> ");
    Serial.print(binPercentage);
    Serial.print(":");
    Serial.println(strlen(binPercentage));

    weight = 0;
    char binWeight[9] = {0}; 
    itoa((byte)weight , binWeight, 2);
    Serial.print("Weight of hive -> ");
    Serial.print(binWeight);
    Serial.print(":");
    Serial.println(strlen(binWeight));

    Serial.print("State of hive move -> ");
    Serial.println(movedHive);

    char finalBinMessage[49] = {"000000000000000000000000000000000000000000000000"};
    
    Serial.println();
    Serial.println("Empty message:");
    Serial.print(finalBinMessage);
    Serial.print(":");
    Serial.println(strlen(finalBinMessage));

    int i;
    for(i = 0; i < 8; i++){
        if(i < strlen(binTemperature))finalBinMessage[47 - i] = binTemperature[(strlen(binTemperature)-1) - i];
    }

    for(i = 0; i < 8; i++){
        if(i < strlen(binTemperature2))finalBinMessage[39 - i] = binTemperature2[(strlen(binTemperature2)-1) - i];
    }

    for(i = 0; i < 7; i++){
        if(i < strlen(binHumidity))finalBinMessage[31 - i] = binHumidity[(strlen(binHumidity)-1) - i];
    }

    for(i = 0; i < 7; i++){
        if(i < strlen(binHumidity2))finalBinMessage[24 - i] = binHumidity2[(strlen(binHumidity2)-1) - i];
    }

    for(i = 0; i < 7; i++){
        if(i < strlen(binPercentage))finalBinMessage[17 - i] = binPercentage[(strlen(binPercentage)-1) - i];
    }

    for(i = 0; i < 7; i++){
        if(i < strlen(binWeight))finalBinMessage[10 - i] = binWeight[(strlen(binWeight)-1) - i];
    }

    if(movedHive == true)finalBinMessage[0] = '1';
    else finalBinMessage[0] = '0';

    Serial.println("Final message:");
    Serial.print(finalBinMessage);
    Serial.print(":");
    Serial.println(strlen(finalBinMessage));

    int j = 0;   
    char finalMessage[13] = {0};
    
    for(i = 0; i < 48; i = i + 4){
      
      String hexaDecimal = String(finalBinMessage);
      hexaDecimal = hexaDecimal.substring(i, i+4);
      int val = strtol(hexaDecimal.c_str(), NULL, 2);
      
      Serial.print(hexaDecimal);
      Serial.print(":");
      Serial.print(val);
      Serial.print(":");
      Serial.println(String(val, HEX));

      finalMessage[j] = String(val, HEX).charAt(0);
      j++;
    }

    Serial.println(finalMessage);
    Serial3.print("AT$SF=");
    Serial3.println(finalMessage);
}

