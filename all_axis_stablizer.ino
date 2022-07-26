#include <Arduino.h>
#include<Wire.h>
#include<Servo.h>
#include <SD.h>
#include <protothreads.h>

#define battPin0 A11 ///< Select the input pins for the batteries
#define battPin1 A14 ///< Select the input pins for the batteries
#define battPin2 A12 ///< Select the input pins for the batteries

Servo servo;  ///<Instanciate First Servo Motor
Servo servo2; ///<Instanciate Second Servo Motor
Servo servo3; ///<Instanciate Third Servo Motor

int servo_pin = A2; ///<Pin where Servo is connected to Arduino
int servo_pin2 = A3; ///<Pin where Servo is connected to Arduino
int servo_pin3 = A1; ///<Pin where Servo is connected to Arduino
int sd_cs_pin = 53; ///< Pin where cs pin is connected to Arduino
int light_pin = A6; ///< Pin to activate LED
unsigned long t = 0; ///< Count time for datalogger

int p_airelon_max = 150; ///< Maximum value for airelon physical limits. 
int p_airelon_min = 30; ///< Minimum value for airelon physical limits.
int p_flap_r_max = 100; ///< Maximum Value for right flap physical limits.
int p_flap_r_min = 0; ///< Minimum value for right flap physical limits.
int p_flap_l_max = 80; ///< Maximum value for left flap physical limits.
int p_flap_l_min = 180; ///< Minimum value for left flap physical limits.
int p_rudder_max = 180; ///< Maximum value for rudder physical limits.
int p_rudder_min = 0; ///< Minimum value for rudder physical limits.
int p_elevator_max = 120; ///< Maximum value for elevator physical limits.
int p_elevator_min = 30; ///< Minimum value for elevator physical limits.

File file; ///< Object to write on memory

const int MPU_addr = 0x68; ///< Address of MPU6050 for communications
int16_t AcX; ///< Variable to store X axis acelerometer data from IMU.
int16_t AcY; ///< Variable to store Y axis acelerometer data from IMU.
int16_t AcZ; ///< Variable to store Z axis acelerometer data from IMU.
int16_t Tmp; ///< Variable for temperature.
int16_t GyX; ///< Variable to store X axis giroscope data from IMU.
int16_t GyY; ///< Variable to store Y axis giroscope data from IMU.
int16_t GyZ; ///< Variable to store Z axis giroscope data from IMU.

int Yangle; ///< Aux variable to convert IMU data into degrees
int Xangle; ///< Aux variable to convert IMU data into degrees
int Zangle; ///< Aux variable to convert IMU data into degrees  


float battWeighting = 0.1; ///< Battery variable which define a weighting to apply to our exponential moving average calculation for battery. 


int abattValue0 = 0; ///< battery variables, variable to store average value (exponential moving average) calculation.
int abattValue1 = 0; ///< battery variables, variable to store average value (exponential moving average) calculation.
int abattValue2 = 0; ///< battery variables, variable to store average value (exponential moving average) calculation.


float Cell1 = 0.00; ///< variable to store actual cell voltages.
float Cell2 = 0.00; ///< variable to store actual cell voltages.
float Cell3 = 0.00; ///< variable to store actual cell voltages.

float percentage; ///< Percentage of remaining battery life.

float adcVolt = 0.0041780351906158 ; ///< ADC constant for read voltage purposes.



// Protothread

pt ptReadWrite; 

/*! \brief Read sensor data and update servo position
 *
 *  Using a thread object as paramater reads sensor data and update servo position
 *
 *  \param pt* pt
 */
 int readWriteThread(struct pt* pt) {
  PT_BEGIN(pt);
  //PT_SLEEP(pt, 200);
  //PT_YIELD(pt);
  // Loop forever
  
  for(;;) {
    get_GyroData();
    update_Servo_position();
    PT_SLEEP(pt, 20);
    PT_YIELD(pt);
  }
PT_END(pt);
}

pt ptDatalogger;

/*! \brief Get sensor data to write on file
 *
 *  This function uses thread to get sensor data and write on text file into SD card
 *
 *  \param pt* pt 
 */ 
int dataloggerThread(struct pt* pt) {
  PT_BEGIN(pt);

  // Loop forever
  for(;;) {
  write_log(generate_logData());
  update_time();
  PT_YIELD(pt);
  }

  PT_END(pt);
}

pt ptLight;

/*! \brief Turns on LED
 *
 *  This function uses thread to turn on LED
 *
 *  \param pt* pt
 */
int lightThread(struct pt* pt) {
  PT_BEGIN(pt);

  // Loop forever
  for(;;) {
  light_mode(1);
  PT_YIELD(pt);
  }

  PT_END(pt);
}

/*! \brief Read battery voltage.
 *
 *  This function read analog battery values then convert real voltage values
 *  and return the sum of all battery cell values.
 * 
 *  \retval float Cell1+Cell2+Cell3 
 */
float getBattVolts() {
  // read the value from the sensor:
  abattValue0 = (analogRead(battPin0) * battWeighting) + (abattValue0 * (1-battWeighting));
  abattValue1 = (analogRead(battPin1) * battWeighting) + (abattValue1 * (1-battWeighting));
  abattValue2 = (analogRead(battPin2) * battWeighting) + (abattValue2 * (1-battWeighting));
  //abattValue3 = (analogRead(battPin3) * battWeighting) + (abattValue3 * (1-battWeighting));
  // convert these values to cell voltages
  Cell1 = (adcVolt * abattValue0 * 1) ;
  Cell2 = (adcVolt * abattValue1 * 1.7298)-Cell1;
  Cell3 = (adcVolt * abattValue2 * 2.6078)-Cell2-Cell1;
  //Cell4 = (adcVolt * abattValue3 * 3.7659)-Cell3-Cell2-Cell1;
  return Cell1+Cell2+Cell3;
}

/*! \brief Print battery life.
 *
 *  This function is used to print the battery life in percentage for debug.
 * 
 */
void showBattVolts() {
  Serial.print (Cell1);
  Serial.print ("V. " );
  Serial.print (Cell2);
  Serial.print ("V. ");
  Serial.print (Cell3);
  Serial.print ("V. Total = " );
  Serial.print (Cell1+Cell2+Cell3);
  Serial.print (" " );
  Serial.print(percentage*100);
  Serial.println("%");
}

/*! \brief Get battery life in percentage
 *
 *  This function gets battery life raw and return to as percentage.
 *  Low and High values are from battery datasheet.
 *
 *  \param float volt
 *  \retval float percentage*100
 */
float get_percentage(float volt){
    float low  = 3.0 * 3;
    float high = 4.2 * 3;
    percentage = constrain((volt - low) / (high - low), 0.0, 1.0);
    return percentage*100;
}

/*! \brief Get gyro sensors data
 *
 *  Iniciates communications with IMU sensor
 *  then read the feedback values to updates in global variables
 *  and convert values to degrees. Wire.read() returns an int
 *  then is shifted 8 bits to the left which is equivalent to
 *  multiplying by 256 then bitwise OR. Turns two 8-bit values into one 16-bit value.
 *  
 */
void get_GyroData(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  Yangle = AcY/182.04; 
  Xangle = AcX/182.04;
  Zangle = GyZ/182.04;
}

/*! \brief Update time for datalogger
 *
 *  Updates time so datalonger knows the instant it saved the value.
 * 
 */
void update_time(){
  t++;
}

/*! \brief Get values and convert to string
 *
 *  This auxiliary function gets general data and converts to string to use in text file later.
 * 
 *  \retval String dataString
 */
String generate_logData(){
  String dataString ="";
  dataString += String(t);
  dataString += String(",");
  dataString += String(Xangle);
  dataString += String(",");
  dataString += String(constrain(Xangle + 90,p_airelon_min,p_airelon_max));
  dataString += String(",");
  dataString += String(Yangle);
  dataString += String(",");
  dataString += String(constrain(-Yangle + 90,p_elevator_min,p_elevator_max));
  dataString += String(",");
  dataString += String(Zangle);
  dataString += String(",");
  dataString += String(constrain(Zangle + 90,p_rudder_min,p_rudder_max));
  dataString += String(",");
  dataString += String(int(get_percentage(getBattVolts())));
  dataString += String(",");
  dataString += String(int(getBattVolts()));
  return dataString;
}

/*! \brief Write data in text file.
 *
 *  Open text file and write each line from generate_logData into it.
 *
 */
void write_log(String data){
  file = SD.open("log.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (file) {
    file.println(data);
    file.close();
  }
}

/*! \brief Update servo angle position
 *
 *  This function write on each servo motor to stabilize the control surfaces.
 *
 */
void update_Servo_position(){
  servo.write(constrain(Zangle+90,p_rudder_min,p_rudder_max));
  servo3.write(constrain(-Yangle + 90,p_airelon_min,p_airelon_max));
  servo2.write(constrain(Xangle + 90,p_elevator_min,p_elevator_max));
}

/*! \brief Configs the LED.
 *
 *  This function is used to config the arduino LED.
 *
 */
void light_mode(int mode){
  switch(mode){
    case 0:
    digitalWrite(light_pin,LOW);
    break;
    case 1:
    digitalWrite(light_pin,HIGH);
    break;
    default:
    digitalWrite(light_pin,LOW);
    break;
  }
}

/*! \brief Configs IMU sensor.
 *
 *  This function configs the IMU(Inertial Measure Unity) sensor parameters.
 *  In this case the IMU used is MPU6050.
 */
void mpu_setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

/*! \brief Initializates Servo.
 *
 *  This functions initializates each servo motors to the desired pins.
 *
 */
void servos_attach(){
  servo.attach(servo_pin);
  servo2.attach(servo_pin2);
  servo3.attach(servo_pin3);
}

/*! \brief Gets a mean of battery life
 *
 *  This function returns an avarage on battery voltage. 
 *
 */
void get_baterry_average(int times){
  for (int i=0; i<times; i++){
      getBattVolts();
    }
}

/*! \brief Arduino setup function
 *
 *  This function start up and config the system.
 *
 */
void setup() {
  pinMode(light_pin,OUTPUT);  
  Serial.begin(115200);  
  mpu_setup();  
  //Starting SD
  SD.begin(sd_cs_pin);
  servos_attach();
  get_baterry_average(50);
  PT_INIT(&ptReadWrite);
  PT_INIT(&ptDatalogger);
}


/*! \brief Arduino infinite loop.
 *
 *  This function is the main execution loop..
 *
 */
void loop() {

  PT_SCHEDULE(readWriteThread(&ptReadWrite));
  PT_SCHEDULE(dataloggerThread(&ptDatalogger));
  PT_SCHEDULE(lightThread(&ptLight));
}
