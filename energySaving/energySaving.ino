//libraries definition
#include <DHT.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HX711.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


// libs and variables for sleep functionality
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define OUTPUT_READABLE_YAWPITCHROLL

//define HX711 pins
#define DT 5
#define SCK 4

//leds signalization
#define LED_ON 50 //Status ON
#define LED_CL 51 //Status Calibration
#define LED_SD 48 //Status Send

////initialization of HX711
HX711 scale(DT, SCK);

//definition of variables for load sensor
float weight = 0;
int weight2 = 0; 

float calibration_factor = -21740;
float zero_factor = 318940; //318940 

//definition of variables for battery status
#define CHARGER_CONTROL 49
#define NUM_SAMPLES 10
#define DIODE_DROP 0.87
float lowVoltage = 13.5;
float highVoltage = 13.8;
float Vcc = 4.97;
//Division factor is calculated as follows: divFactor = Vin / V10k
//Vin is the input voltage
//V10k id the voltage on 10k resistor
float divisionFactor = 16.379;
float Aref = 1.41;   //1.2 or 1.315 1.415
int total = 0;
int sample = 0;
unsigned char sample_count = 0;
float voltage = 0;
float current = 0;
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

//definition of variables for MPU6050 calibration
int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
int buffersize = 1000, acel_deadzone = 8, giro_deadzone = 1;
int calibrationState = 0;

//definition of variable for hive movement
bool movedHive;

//definition of variables for time measure
unsigned long currentTimeAkc;
unsigned long startTimeAkc;
unsigned long currentTime;
unsigned long startTime;
unsigned long startTimeWaitAkc;
long int interval;

//definition of variables for DHT22
DHT dhtOut(52, DHT22);
DHT dhtIn(53, DHT22);

int humidityOut;
int temperatureOut;
int humidityIn;
int temperatureIn;

//definition of variables for SigFox modem
#define TX 12
#define RX 13

//initialization of software serial link
SoftwareSerial Sigfox(RX, TX);

//definition of variables for sending messages
String command;
String message;

char finalMessage[13] = {0};
char lastMessage[13] = {0};
char binTemperatureOut[9] = {0};
char binTemperatureIn[9] = {0}; 
char binHumidityOut[9] = {0}; 
char binHumidityIn[9] = {0};
char binPercentage[9] = {0}; 
char binWeight[9] = {0};

/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect)
{

}


/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. SLEEP_MODE_PWR_SAVE*/
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}





void setup() {

  interval = 60000; //normal = 600000 = 10 min
  
  // use the internal ~1.1volt reference
  //analogReference(INTERNAL1V1);

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
  Serial.println();
  Serial.println(F("Testing device connections"));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  //load and configure the DMP
  Serial.println();
  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();
 
  //set gyro and accel offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

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

  //initialize devices
  dhtOut.begin();
  dhtIn.begin();

  //measure start time
  startTime = millis();
  startTimeWaitAkc = millis();

  //set default state of hive
  movedHive = false;

  //initialize HX711
  scale.set_scale(-21740);
  scale.set_offset(zero_factor);
  //scale.tare();
  
  //starting software serial link 
  Sigfox.begin(9600);
  Serial3.begin(9600);

  //initialize digital pin LED_ON, LED_CL, LED_SD as an output
  pinMode(LED_ON, OUTPUT); 
  pinMode(LED_CL, OUTPUT);
  pinMode(LED_SD, OUTPUT);

  //init of battery charger pin as an output
  pinMode(CHARGER_CONTROL, OUTPUT);
  digitalWrite(CHARGER_CONTROL, LOW);

   /*** Setup the WDT ***/
  
  Serial.println();
  Serial.println(F("Initializing Battery sleep settings"));
  
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds // NOTE from JP: this is not exactly 8.0 !, also seems to pauze system clock during sleep
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
  
  Serial.println();
  Serial.println(F("End Battery sleep"));
  
  
}

void loop() {

  //MPU6050 calibration 
  
  if(calibrationState == 0){

    digitalWrite(LED_CL, HIGH);
   
    Serial.println();
    Serial.println("Starting calibration of MPU6050");

    if (state==0){
      Serial.println("Reading sensors for first time");
      meansensors();
      state++;
      delay(1000);
    }

    if(state==1){
      Serial.println("Calculating offsets:");
      //calibration();
      state++;
      delay(1000);
    }

    if (state==2) {
      meansensors();
        
      Serial.print("\nAccelerometer X: ");
      Serial.println(ax_offset); 
      Serial.print("Accelerometer Y: ");
      Serial.println(ay_offset); 
      Serial.print("Accelerometer Z: ");
      Serial.println(az_offset); 
      Serial.print("Gyroskop X: ");
      Serial.println(gx_offset); 
      Serial.print("Gyroskop Y: ");
      Serial.println(gy_offset); 
      Serial.print("Gyroskop Z: ");
      Serial.println(gz_offset);
      Serial.println("Finished calibration of MPU6050");
  
      mpu.setXAccelOffset(ax_offset);
      mpu.setYAccelOffset(ay_offset);
      mpu.setZAccelOffset(az_offset);
      mpu.setXGyroOffset(gx_offset);
      mpu.setYGyroOffset(gy_offset);
      mpu.setZGyroOffset(gz_offset);
      
      calibrationState = 1;
      digitalWrite(LED_CL, LOW);
      
    }
    
    mpu.setXAccelOffset(-607);
    mpu.setYAccelOffset(1837);
    mpu.setZAccelOffset(3750);
    mpu.setXGyroOffset(22);
    mpu.setYGyroOffset(56);
    mpu.setZGyroOffset(5);
    
    digitalWrite(LED_ON, HIGH);
  }
  
  //if program fails
  if (!dmpReady) {
    Serial.println("END");
    return;
  }
    
  //wait for MPU interrupt
  while (!mpuInterrupt && fifoCount < packetSize) {
    //Serial.println("----------------------MPU INTERRUPT---------------------");
    
    if (Serial3.available()) {
      message = Serial3.readString();
    }    

    message.trim();
    if(!message.equals("")){
      digitalWrite(LED_ON, LOW);
      Serial.println("----------------------SIGFOX-MESSAGE---------------------");
      Serial.println(message);
      Serial.println("----------------------SIGFOX-MESSAGE---------------------");
      message = "";
      digitalWrite(LED_ON, HIGH);
    }
    
    if (Serial.available()) {
     command = Serial.readString();
    }
  
    command.trim();
    if(!command.equals("")){
      digitalWrite(LED_ON, LOW);
      Serial.println("----------------------SIGFOX-COMMAND---------------------");
      command = command + "\r";
      Serial.println(command);
      Serial.println("----------------------SIGFOX-COMMAND---------------------");
      
      Serial3.println(command);
      command = "";
      digitalWrite(LED_ON, HIGH);
    }

    //measuring actual time
    currentTime = millis();
    //Serial.print(currentTime); /////////////////////////////////////////////////////////
    //repeated every 10min
    //if((currentTime - startTime) >=  (interval + 500) || calibrationState == 1){  // now use sleep as timer
    if (1 == 1){
      digitalWrite(LED_ON, LOW);
      calibrationState = 2;
      
      //measuring first DHT22   
      humidityOut = dhtOut.readHumidity();
      temperatureOut = dhtOut.readTemperature();
    
      //measuring second DHT22
      humidityIn = dhtIn.readHumidity();
      temperatureIn = dhtIn.readTemperature();

      //checking measuremts from DHT22s
      if((humidityOut < 0) || (humidityOut > 100)){
        humidityOut = 127;
      }

      if((temperatureOut < -50) || (temperatureOut > 80)){
        temperatureOut = 127;
      }
      
      if((humidityIn < 0) || (humidityIn > 100)){
        humidityIn = 127; 
      }
      
      if((temperatureIn < -50) || (temperatureIn > 80)){
        temperatureIn = 127;
      }
       
      //reading value from analog input to clear old input
      analogRead(A1);

      //measuring input voltage
      total = 0;
      sample_count = 0;
      while(sample_count < NUM_SAMPLES){
        sample = analogRead(A1);
        Serial.println(sample);
        total += sample;
        sample_count++;
        delay(10);
      }

      //converting input voltage to voltage and calculating percentage
      voltage = (float)total / (float)NUM_SAMPLES * Vcc / 1024.0;
      Serial.print("Voltage on 10k resistor: ");
      Serial.print(voltage);
      Serial.println(" V");
      voltage *= divisionFactor;
      voltage += DIODE_DROP;
      Serial.print("Voltage on battery: ");
      Serial.print(voltage);
      Serial.println(" V");

      if (voltage < lowVoltage){
        digitalWrite(CHARGER_CONTROL, HIGH);
      }
      else if(voltage >= highVoltage){
        digitalWrite(CHARGER_CONTROL, LOW);
      }

      //100% / difference between actual voltage and 0% voltage gives volts per %
      percentage = (voltage - 11) * (100 / (highVoltage - 11)); 

      //measuring current flowing from solar panel to battery
      analogRead(A2);
      total = 0;
      sample_count = 0;
      while(sample_count < NUM_SAMPLES){
        sample = analogRead(A2);
        Serial.println(sample);
        total += sample;
        sample_count++;
        delay(10);
      }
      current = ((((float)total / (float)NUM_SAMPLES * Vcc / 1024.0) - 2.5) * 10);
      Serial.print("Current flowing to battery: ");
      Serial.print(current);
      Serial.println(" A");

      //measure weight
      weight = scale.get_units();
      
      //printing values
      Serial.println();
      Serial.println("--------------------------DHT22--------------------------");
      Serial.print("Humidity(OUT): ");
      Serial.print(humidityOut);
    
      Serial.print(" %, Temperature(OUT): ");
      Serial.print(temperatureOut);
      Serial.println(" C");
    
      Serial.print("Humidity(IN): ");
      Serial.print(humidityIn);
    
      Serial.print(" %, Temperature(IN): ");
      Serial.print(temperatureIn);
      Serial.println(" C");
      Serial.println("--------------------------DHT22--------------------------");
      Serial.println();
      
      Serial.println("-------------------------Battery-------------------------");
      Serial.print("Voltage: ");
      Serial.print(voltage);
      Serial.println(" V");

      //warnning for low battery
      if(percentage <= 20){
        Serial.println("Warning: Low battery");
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

      
      Serial.println("-------------------------Weight--------------------------");
      Serial.print("Weight: ");
      Serial.print(weight);
      Serial.println(" kg");
      Serial.println("-------------------------Weight--------------------------");
            
      Serial.println("-------------------------Message-------------------------");
      messageConvert();
      Serial.println("-------------------------Message-------------------------");
      
      //measuring actual time
      startTime = millis();
      digitalWrite(LED_ON, HIGH);

      Serial.println("-------------------------Sleeping Start-------------------------");
      
        // Sleeping code
         //Serial.println(currentTime); 
       digitalWrite(LED_ON, HIGH);
       int a;  // each sleep iteration is 8 sec // 70 = 9 and half min // 69 seems to be more accurate
       for( a = 0; a < 69; a = a + 1 ){
            //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            //Serial.println("Loop f_wdt == 1");
            Serial.println(a);
            Serial.println("I will go sleep now");
            delay(100); //Allow for serial print to complete.
            digitalWrite(LED_ON, !digitalRead(LED_ON));
            enterSleep();
            Serial.println("Good morning, good morning :-) ");
       }
      //Serial.println("Loop before mpu");
      digitalWrite(LED_ON, HIGH);
      Serial.println("-------------------------Sleeping End-------------------------");
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

        //Wait for 2 minutes while accel beacome stabil
        if(millis() - startTimeWaitAkc>= 120000){
          //check for hive movement
          if(movedHive == false){
              if((ypr[1] * 180 / M_PI > 10) || (ypr[2] * 180 / M_PI > 10)  || (ypr[1] * 180 / M_PI < -10) || (ypr[2] * 180 / M_PI < -10)){
                movedHive = true;
              }
          }
        }

        //check if interval was reached
        currentTimeAkc = millis();
        if((currentTimeAkc - startTimeAkc) >= interval) //(startTimeWaitAkc - millis() >= 10000) && 
        {
          
          digitalWrite(LED_ON, LOW);
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
            Serial.println("State: hive is on right place");          
          }
          else Serial.println("State: hive was moved");
          

          Serial.println("----------------------Accelerometer----------------------");
          Serial.println();

          startTimeAkc = millis();
          digitalWrite(LED_ON, HIGH);
        }
    #endif
  }

}

void messageConvert() {
    
    int j, i, val;
    String hexaDecimal;
        
    Serial.println();

    //digitalWrite(LED_SD, HIGH);
    //converting outside temperature into binary
    if(temperatureOut < 0){
      temperatureOut = (-temperatureOut) + 128;
      itoa((byte) temperatureOut, binTemperatureOut, 2);
      }
    else{
      itoa((byte) temperatureOut, binTemperatureOut, 2);
      }
      
    Serial.print("Outside temperature -> ");
    Serial.print(binTemperatureOut);
    Serial.print(":");
    Serial.println(strlen(binTemperatureOut));

    //converting inside temperature into binary
    if(temperatureIn < 0){
      temperatureIn = (-temperatureIn) + 128;
      itoa((byte) temperatureIn, binTemperatureIn, 2);
      }
    else{
      itoa((byte) temperatureIn, binTemperatureIn, 2);
      }
      
    Serial.print("Inside temperature -> ");
    Serial.print(binTemperatureIn);
    Serial.print(":");
    Serial.println(strlen(binTemperatureIn));

    //converting outside humidity into binary
    itoa((byte) humidityOut, binHumidityOut, 2);
    Serial.print("Outside humidity -> ");
    Serial.print(binHumidityOut);
    Serial.print(":");
    Serial.println(strlen(binHumidityOut));

    //converting second humidity into binary
    itoa((byte) humidityIn, binHumidityIn, 2);
    Serial.print("Inside humidity -> ");
    Serial.print(binHumidityIn);
    Serial.print(":");
    Serial.println(strlen(binHumidityIn));

    //converting battery state into binary
    itoa((byte) percentage, binPercentage, 2);
    Serial.print("Battery percentage -> ");
    Serial.print(binPercentage);
    Serial.print(":");
    Serial.println(strlen(binPercentage));

    //converting weight of hive into binary 
    weight2 = round(weight);
    
    itoa((byte)weight2 , binWeight, 2);
    Serial.print("Weight of hive -> ");
    Serial.print(binWeight);
    Serial.print(":");
    Serial.println(strlen(binWeight));

    //state of hive move
    Serial.print("State of hive move -> ");
    Serial.println(movedHive);

    //creating final message in binary
    char finalBinMessage[49] = {"111000000000000000000000000000000000000000000000"};

    //empty message
    Serial.println();
    Serial.println("Empty message:");
    Serial.print(finalBinMessage);
    Serial.print(":");
    Serial.println(strlen(finalBinMessage));

    //filling the message with first temperature
    for(i = 0; i < 8; i++){
      if(i < strlen(binTemperatureOut)){
        finalBinMessage[47 - i] = binTemperatureOut[(strlen(binTemperatureOut)-1) - i];
      }
    }

    //filling the message with second temperature
    for(i = 0; i < 8; i++){
      if(i < strlen(binTemperatureIn)){
        finalBinMessage[39 - i] = binTemperatureIn[(strlen(binTemperatureIn)-1) - i];
      }
    }

    //filling the message with outside humidity
    for(i = 0; i < 7; i++){
      if(i < strlen(binHumidityOut)){
        finalBinMessage[31 - i] = binHumidityOut[(strlen(binHumidityOut)-1) - i];
      }
    }

    //filling the message with inside humidity
    for(i = 0; i < 7; i++){
      if(i < strlen(binHumidityIn)){
        finalBinMessage[24 - i] = binHumidityIn[(strlen(binHumidityIn)-1) - i];
      }
    }

    //filling the message with battery state
    for(i = 0; i < 7; i++){
      if(i < strlen(binPercentage)){
        finalBinMessage[17 - i] = binPercentage[(strlen(binPercentage)-1) - i];
      }
    }

    //filling the message with hive weight
    for(i = 0; i < 7; i++){
      if(i < strlen(binWeight)){
        finalBinMessage[10 - i] = binWeight[(strlen(binWeight)-1) - i];
      }
    }

    //filling the message with state of hive move
    if(movedHive == true){
      finalBinMessage[3] = '1';
    }
    else finalBinMessage[3] = '0';

    //final message
    Serial.println("Final message:");
    Serial.print(finalBinMessage);
    Serial.print(":");
    Serial.println(strlen(finalBinMessage));

    //convert final binary message to hexadecimal
    j = 0;
    strcpy(lastMessage, finalMessage);
       
    for(i = 0; i < 48; i = i + 4){
      
      hexaDecimal = String(finalBinMessage);
      hexaDecimal = hexaDecimal.substring(i, i+4);
      val = strtol(hexaDecimal.c_str(), NULL, 2);
      
      Serial.print(hexaDecimal);
      Serial.print(":");
      Serial.print(val);
      Serial.print(":");
      Serial.println(String(val, HEX));

      finalMessage[j] = String(val, HEX).charAt(0);
      j++;
    }

  //sending the message
  if (!String(finalMessage).equals(String(lastMessage)))
   {  
      digitalWrite(LED_SD, HIGH);
      Serial.println(finalMessage);
      Serial3.print("AT$SF=");
      Serial3.println(finalMessage);
      delay(200);
      digitalWrite(LED_SD, LOW);
   }
}

void meansensors(){
  
  long i=0, buff_ax=0, buff_ay=0, buff_az=0, buff_gx=0, buff_gy=0, buff_gz=0;

  while (i < (buffersize + 101)){
    
    //read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //first 100 measures are discarded
    if((i > 100) && (i <= (buffersize+100))){ 
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if(i == (buffersize + 100)){
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    
    //needed so we don't get repeated measures
    delay(2); 
  }
}

void calibration(){
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384-mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  
  while (1){
    
    int ready=0;
    
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    Serial.print(".");

    if(abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;
    
    if(abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if(abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if(abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if(abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if(abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if(ready==6) break;
  }
}
