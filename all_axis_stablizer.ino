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

int servo_pin = A2;
int servo_pin2 = A3;
int servo_pin3 = A1;
int sd_cs_pin = 53;
int light_pin = A6;
unsigned long t = 0;

int p_airelon_max = 150; ///< Control surfaces phisical limits: Airelon
int p_airelon_min = 30; ///< Control surfaces phisical limits: Airelon
int p_flap_r_max = 100; ///< Control surfaces phisical limits: Flap Right Maximum 
int p_flap_r_min = 0; ///< Control surfaces phisical limits: Flap Right Minimum
int p_flap_l_max = 80; ///< Control surfaces phisical limits: Flap Left Maximum
int p_flap_l_min = 180; ///< Control surfaces phisical limits: Flap Left Minimum 
int p_rudder_max = 180; ///< Control surfaces phisical limits: Rudder Maximum
int p_rudder_min = 0; ///< Control surfaces phisical limits: Rudder Minimum
int p_elevator_max = 120; ///< Control surfaces phisical limits: Elevator Maximum
int p_elevator_min = 30; ///< Control surfaces phisical limits: Elevator Minimum

File file;

const int MPU_addr = 0x68; ///< MPU variables
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; ///< MPU variables
int Yangle,Xangle,Zangle; ///< MPU variables


float battWeighting = 0.1; ///< battery variables, define a weighting to apply to our exponential moving average calculation for battery 


int abattValue0 = 0; ///< battery variables, variable to store average value (exponential moving average) calculation
int abattValue1 = 0; ///< battery variables, variable to store average value (exponential moving average) calculation
int abattValue2 = 0; ///< battery variables, variable to store average value (exponential moving average) calculation


float Cell1 = 0.00; ///< variable to store actual cell voltages
float Cell2 = 0.00; ///< variable to store actual cell voltages
float Cell3 = 0.00; ///< variable to store actual cell voltages

float percentage;
float adcVolt = 0.0041780351906158 ; ///< one point on the ADC equals this many volts



// Protothread

pt ptReadWrite; 

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
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
/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
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
/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
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

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
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

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
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

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
 */
float get_percentage(float volt){
    float low  = 3.0 * 3;
    float high = 4.2 * 3;
    percentage = constrain((volt - low) / (high - low), 0.0, 1.0);
    return percentage*100;
}

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
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

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
 */
void update_time(){
  t++;
}

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
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

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
 */
void write_log(String data){
  file = SD.open("log.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (file) {
    file.println(data);
    file.close();
  }
}

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
 */
void update_Servo_position(){
  servo.write(constrain(Zangle+90,p_rudder_min,p_rudder_max));
  servo3.write(constrain(-Yangle + 90,p_airelon_min,p_airelon_max));
  servo2.write(constrain(Xangle + 90,p_elevator_min,p_elevator_max));
}

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
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

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
 */
void mpu_setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
 */
void servos_attach(){
  servo.attach(servo_pin);
  servo2.attach(servo_pin2);
  servo3.attach(servo_pin3);
}

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
 */
void get_baterry_average(int times){
  for (int i=0; i<times; i++){
      getBattVolts();
    }
}

/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
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


/*! \brief Brief description function below.
 *
 *  Detailed description starts here.
 *
 *  \param pt ponteiro
 *  \retval int x
 */
void loop() {

  PT_SCHEDULE(readWriteThread(&ptReadWrite));
  PT_SCHEDULE(dataloggerThread(&ptDatalogger));
  PT_SCHEDULE(lightThread(&ptLight));
}
