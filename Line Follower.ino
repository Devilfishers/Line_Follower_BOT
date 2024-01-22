#include <QTRSensors.h>  // including the QTRSensor library in order for the arduino to understand the commands that are given to the sensors 
QTRSensors qtr;         //  setting QTRSensor string to qtr for it to simplify the calling of the library

const uint8_t SensorCount = 8; //declaring the sonsor to activate all of its 8 IR LEDs with 8-bit integer values
uint16_t sensorValues[SensorCount]; // declaring a list that will contain sensor values in it with 16-bit integer values

float Kp = 0.07;  // declaring PROPORTIONAL constant for the PID function that will be created in the upcoming lines
float Ki = 0.0008;  // declaring INTGERAL constant for the PID function that will be created in the upcoming lines
float Kd = 0.6;   // declaring DERIVATIVE constant for the PID function that will be created in the upcoming lines

int P; //declaring an integer named P for the process that is proportional to the error value
int I; //declaring an integer named I for initiliazing the first error value for the summation of past error values
int D; //declaring an integer named D for setting the calculation that will be held in the DERIVATIVE segment in the upcoming PID_control function, to be equal to 

int lastError = 0; // declaring an integer for DERIVATIVE segment to use in order to calculate the error values that the system will encounter

const uint8_t maxspeeda = 240; //declaring the maximum speed value (up to 255) for right(A) motor
const uint8_t maxspeedb = 240; //declaring the maximum speed value (up to 255) for left(B) motor

const uint8_t basespeeda = 150;  //declaring the base speed value for motorA
const uint8_t basespeedb = 150;  //declaring the base speed value for motorB

int aenbl = 9; // declaring the ENA pin on the motor driver that is connected to the D9 pin on the arduino to control PWM of the motorA
int in1 = 7; // declaring the IN1 pin on the motor driver that is connected to the D7 pin on the arduino to set a comminucation channel between OUT1 of the motor driver and arduino
int in2 = 6; // declaring the IN2 pin on the motor driver that is connected to the D6 pin on the arduino to set a comminucation channel between OUT2 of the motor driver and arduino

int benbl = 3; // declaring the ENB pin on the motor driver that is connected to the D3 pin on the arduino to control PWM of the motorB
int in3 = 5; // declaring the IN3 pin on the motor driver that is connected to the D5 pin on the arduino to set a comminucation channel between OUT3 of the motor driver and arduino
int in4 = 4; // declaring the IN4 pin on the motor driver that is connected to the D4 pin on the arduino to set a comminucation channel between OUT4 of the motor driver and arduino

void calibration() {
  
  digitalWrite(LED_BUILTIN, HIGH); // turning on the LED of the arduino in order to inform the user that the system is in the calibration mode 
  
  for (uint16_t i = 0; i < 400; i++) // this loop allows user to manually calibrate the sensor by helding IR LEDs on a dark and respectively light surfaces about 10 seconds. By...
//... limiting the intagrator(i) in the loop up to 400, the user corresponds computing operation with 10 seconds in real life
  {
    qtr.calibrate(); //calibration starts
  }
  digitalWrite(LED_BUILTIN, LOW); // arduino light turns of, indicating that the calibration has ended
}

void setup() {   //the main setup function for hardwares to recognized by the system
  
  qtr.setTypeRC(); //choosing which one of the sensors models between QTR-8A and QTR-8RC to be worked with
  
  qtr.setSensorPins((const uint8_t[]){11, 12, A5, A4, A3, A2, A1, A0}, SensorCount); //introducing the connections of IR LEDs to the arduino from 1th to 8th outputs.
 
  qtr.setEmitterPin(13); //declaring the LEDON pin in order to activate IR LEDs

  pinMode(aenbl, OUTPUT); //setting ENA as OUTPUT for the electrical current to flow correctly
  pinMode(in1, OUTPUT);  //setting IN1 as OUTPUT for the electrical current to flow correctly
  pinMode(in2, OUTPUT); //setting IN2 as OUTPUT for the electrical current to flow correctly
  
  
  pinMode(benbl, OUTPUT); //setting ENB as OUTPUT for the electrical current to flow correctly
  pinMode(in3, OUTPUT); //setting IN3 as OUTPUT for the electrical current to flow correctly
  pinMode(in4, OUTPUT); //setting IN4 as OUTPUT for the electrical current to flow correctly 
  
  pinMode(LED_BUILTIN, OUTPUT); //setting LED of the arduino which is built-in to the microcontroller as OUTPUT

  boolean Ok = false; //declaring a boolean variable in order for the calibration operation not to be infinitely
  
  while (Ok == false) // initiliazing the loop
  { 
    calibration(); // calling the calibration function
   
   Ok = true; //breaking out of the loop
  }
}

void forward(int posa, int posb) { //creating a function for setting the appropriate speed values for motorA and motorB so that the robot goes forward by various rotation angles
  
   digitalWrite(in4, HIGH); //setting IN4 as HIGH also sets OUT4 as HIGH 
   digitalWrite(in3, LOW); //setting IN3 as LOW also sets OUT3 as LOW
   //by setting OUT4 as HIGH and OUT3 as LOW for motorB, the current that flows in the motor driver manipulates the H-Bridge in order for the motorB to turn anticlock-wise direction
 
   digitalWrite(in2,HIGH); //setting IN2 as HIGH also sets OUT2 as HIGH
   digitalWrite(in1,LOW); //setting IN1 as LOW also sets OUT1 as LOW
   //by setting OUT2 as HIGH and OUT1 as LOW for motorA, the current that flows in the motor driver manipulates the H-Bridge in order for the motorA to turn cloclwise direction
 
   analogWrite(aenbl, posa); //setting ENA pin to a desired speed value by changing posa parameter between 0 and 255
   analogWrite(benbl, posb); //setting ENB pin to a desired speed value by changing posb parameter between 0 and 255
}


void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues); // declaring a 16-bit integer variable named "position" for making it equal to the data set that is obtained from the...
//..."qtr.readLineBlack" function which reads the previously declared "sensorValues" list in order to determine the position of robot  
  
  int error = 3500 - position; //declaring and calculatin an error value with respect to position value.3500 is the ideal position (the centre)

  P = error; //creating a PROPORTIONAL segment in order to make a speed value that is proportional to the error value by using P variable
  
  I = I + error; //creating a INTEGRAL segment in order to make a speed value according to the STEADY-STATE error value by using summing past error values with usage of I variable
  
  D = error - lastError; //creating a DERIVATIVE segment in order to make a speed value to prevent overshooting by calculating the overcoming error values with usage of D variable
  lastError = error;
  
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculating the correction of the speed value since the procces in the system is speed and actuators are motors
  
  int motorspeeda = basespeeda + motorspeed; //updating the speed value of motorA according to the distance of the systems' center
  int motorspeedb = basespeedb - motorspeed; //updating the speed value of motorB according to the distance of the systems' center
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;      //this equation prevents the overshooting of motorspeed variable for motorA since the motors can not read a value more than 255
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;     //this equation prevents the overshooting of motorspeed variable for motorB since the motors can not read a value more than 255
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;            //this equation prevents the overshooting of motorspeed variable for motorA since the motors can not read a value less than 0
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;        //this equation prevents the overshooting of motorspeed variable for motorB since the motors can not read a value less than 0
  } 
  forward(motorspeeda, motorspeedb);//according to the correction of speedvalues, the forward function embedded within the PID function for the robot to move is activated 
}

void loop() {  // main loop function for the robot to operate
  
    PID_control(); // calling the PID_control for the code to infinitely loop

}



/*************************************************************************
* Function Name: PID_control
**************************************************************************
* Summary: 
* This is the function of the PID control system. The distinguishing 
* feature of the PID controller is the ability to use the three control 
* terms of proportional, integral and derivative influence on the controller 
* output to apply accurate and optimal control. This correction is applied to
* the speed of the motors, which should be in range of the interval [0, max_speed],
* max_speed <= 255. 
* 
* Parameters:
* none
* 
* Returns:
*  none
*************************************************************************/
