/*  Title:            Motor Position Manipulation With Tolerance
    Author:           Joe Cook
    Equipment:        In BOM Folder
    Notes:            Version 1: Includes position control, tolerance/deadband, uses analogRead input              
*/

//position variables
#define positionInput A0 //analog input from pot to decide valve position
int positionInputReading; //analog read position in
#define positionFeedback A2 //motor rotory feedback from 10 turn POT
int positionFeedbackReading; //to analog read the position feedback pin 

//motor output variables
int clockwise = D5; //powers clockwise pin on DRV8876 H-Bridge
int anticlockwise = D6; //powers anticlockwise pin on DRV8876 H-Bridge

//variables for position control switch statement (logic cant be used within switch)
#define ON_TARGET 1
#define CLOCKWISE 2
#define ANTICLOCKWISE 3
int switchValue;

//tolerancing variables
int tolerance = 20; //actual plus minus value for tolerancing
int lastPositionFeedback; //changing value to be measured against tolerance


void setup(){
  analogReadResolution(12); //setting A-D to 12 bit
  Serial.begin(9600); //setting up serial port with 9600 baud rate

  //setting output pins
  pinMode(clockwise, OUTPUT);
  pinMode(anticlockwise, OUTPUT);


}

void loop(){

  positionSwitch();
  //calibrate();
  serialMonitoring();

}

//calibration function to be completed in setup
void calibrate(){
  //warning message
  Serial.print("CALIBRATION MODE");

  positionFeedbackReading = analogRead(positionFeedback); //read the position feedback pin

  //406 represents 1 turn out of 10 rotations (12bit)
  //valve will turn 8 times in between the first and last turn of 10TPOT
  if(406 == analogRead(positionFeedback))
  {switchValue = ON_TARGET;} //motor is in correct position

  if((406 > positionFeedbackReading) //motor needs to rotate clockwise to meet set position
  & (406 > (lastPositionFeedback + tolerance))) //allows for tolerancing, wont move until tolerance breached
  {switchValue = CLOCKWISE;} //motor needs to rotate clockwise to meet set position

  if((406 < positionFeedbackReading) //motor needs to rotate anticlockwise to meet set position
  & (406 < (lastPositionFeedback - tolerance)))//allows for tolerancing, wont move until tolerance breached
  {switchValue = ANTICLOCKWISE;}

  switch(switchValue){

    //set both motor inputs high to use DRV8876 BRAKE function
    case ON_TARGET:
    digitalWrite(clockwise, HIGH);
    digitalWrite(anticlockwise, HIGH);
    break;

    //set clockwise pin high and anticlockwise low
    case CLOCKWISE:
    digitalWrite(clockwise, HIGH);
    digitalWrite(anticlockwise, LOW);
    break;

    //set anticlockwise pin high and clockwise low
    case ANTICLOCKWISE:
    digitalWrite(clockwise, LOW);
    digitalWrite(anticlockwise, HIGH);
    break;

    //set both motor inputs low to use DRV8876 COAST function
    default:
    digitalWrite(clockwise, LOW);
    digitalWrite(anticlockwise, LOW);
    break;

  }
  //value for tolerancing comparison
  lastPositionFeedback = positionFeedbackReading; //creates a value to be compared at beginning of function
}

//switch statement works alongside if statements to allow for logic function dependant on motor position
void positionSwitch(){

  positionFeedbackReading = analogRead(positionFeedback); //read the position feedback pin

  
  if(analogRead(positionInput) == positionFeedbackReading) //motor is in correct position no tolerancing needed
  {switchValue = ON_TARGET;} 
  if((analogRead(positionInput) > positionFeedbackReading) //motor needs to rotate clockwise to meet set position
  & (analogRead(positionInput) > (lastPositionFeedback + tolerance))) //allows for tolerancing, wont move until tolerance breached
  {switchValue = CLOCKWISE;}
  if((analogRead(positionInput) < positionFeedbackReading) //motor needs to rotate anticlockwise to meet set position
  & (analogRead(positionInput) < (lastPositionFeedback - tolerance)))//allows for tolerancing, wont move until tolerance breached
  {switchValue = ANTICLOCKWISE;}

  switch(switchValue){

    //set both motor inputs high to use DRV8876 BRAKE function
    case ON_TARGET:
    digitalWrite(clockwise, HIGH);
    digitalWrite(anticlockwise, HIGH);
    break;

    //set clockwise pin high and anticlockwise low
    case CLOCKWISE:
    digitalWrite(clockwise, HIGH);
    digitalWrite(anticlockwise, LOW);
    break;

    //set anticlockwise pin high and clockwise low
    case ANTICLOCKWISE:
    digitalWrite(clockwise, LOW);
    digitalWrite(anticlockwise, HIGH);
    break;

    //set both motor inputs low to use DRV8876 COAST function
    default:
    digitalWrite(clockwise, LOW);
    digitalWrite(anticlockwise, LOW);
    break;
  
  }
  //value for tolerancing comparison
  lastPositionFeedback = positionFeedbackReading; //creates a value to be compared at beginning of function

 //}
}

//monitoring input and output values
void serialMonitoring(){

  Serial.print("\t Position Input = ");
  Serial.print(analogRead(positionInput));
  Serial.print("\t Actual Position = ");
  Serial.print(analogRead(positionFeedback));
  Serial.print("\t last position feedback = ");
  Serial.println(lastPositionFeedback);

}



