/*  Title:            PPMS #17421 Motor Driver using PWM INPUT
    Author:           Joe Cook
    Equipment:        In BOM Folder
    Notes:            Follows structure of flow chart              
*/

//Variable Definitions

  
  //(PWM MEASURE) 
    //mark to space ratio measuring position variables
    #define pulsePin D5  // Define the input pin for the pulse signal
    volatile long pulseStartTime = 0; // Variable to store the start time of the pulse (voltatile used to ensure updates in isr)
    volatile long pulseEndTime = 0;   // Variable to store the end time of the pulse
    volatile long pulseWidth = 0;     // Variable to store the pulse width
    //flags to indicate 1 cycle
    volatile long flag = 0; 
    //mapping PWM Variables 
    int mappedPWM = 0; //variable for mapping PWM value to motor position value

  //(MOTOR CONTROL)
    //position variables
    #define positionFeedback A0 //motor rotory feedback from 10 turn POT
    int positionFeedbackReading; //to analog read the position feedback pin 

    //motor output variables
    int clockwise = D10; //powers clockwise pin on DRV8876 H-Bridge
    int anticlockwise = D9; //powers anticlockwise pin on DRV8876 H-Bridge

  

    //tolerancing variables
    int tolerance = 50; //actual plus minus value for tolerancing 
    int flagTolerance = 0; //flag to ensure value is met before tolerance is used


void setup() {
  //(PWM MEASURE)
    pinMode(pulsePin, INPUT_PULLDOWN); // Set pulsePin as input
    attachInterrupt(digitalPinToInterrupt(pulsePin), changingEdgeISR, CHANGE); //Attaches interrupt to the PWM pin

  //(MOTOR CONTROL)
    //setting input pins
    pinMode(positionFeedback, INPUT);
    //setting output pins
    pinMode(clockwise, OUTPUT);
    pinMode(anticlockwise, OUTPUT);
    //setting up analogRead to 12 bit resolution
    analogReadResolution(10);

  //(SERIAL MONITORING)
    Serial.begin(9600);

}

void loop() {
//(PWM MEASURE)
    if(flag == 2){
        pulseWidth = pulseEndTime - pulseStartTime; //subtracts stamp on rising edge from stamp on falling edge
        flag = 0;
    }

//(MOTOR CONTROL)
    //logic system to control motor rotation and brake 
    //brake if motor in correct position
    if(pulseWidth == analogRead(positionFeedback)){
      digitalWrite(clockwise, 1);
      digitalWrite(anticlockwise, 1);
      flagTolerance = 1;
    }
    //rotate clockwise if motor is lower than correct position
    if((pulseWidth > (analogRead(positionFeedback) + tolerance)) && (flagTolerance = 1)){
      digitalWrite(clockwise, 1);
      digitalWrite(anticlockwise, 0);
      flagTolerance = 0;
    }
    //rotate clockwise if motor is lower than correct position
    if((pulseWidth < (analogRead(positionFeedback) - tolerance)) && (flagTolerance = 1)){
      digitalWrite(clockwise, 0);
      digitalWrite(anticlockwise, 1);
      flagTolerance = 0;
    }

    Serial.print("pulseWidth ");
    Serial.print(pulseWidth);
    Serial.print(" motor position ");
    Serial.print(analogRead(positionFeedback));
    Serial.print(" clockwise ");
    Serial.print(digitalRead(clockwise));
    Serial.print(" anticlockwise ");
    Serial.println(digitalRead(anticlockwise));



    }




//ISR For mark measurement on both edges USING IF
void changingEdgeISR(){  
  if(digitalRead(pulsePin) == 1){
  pulseStartTime = micros(); //measures stamp in microseconds
  flag = 1; //checks whether pulse gone high
  }
  if(digitalRead(pulsePin) == 0){
  pulseEndTime = micros(); //measures stamp in microseconds
  flag = 2; //checks whether pulse gone low
  }
}