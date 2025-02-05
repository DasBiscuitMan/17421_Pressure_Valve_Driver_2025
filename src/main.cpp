/*  Title:            PPMS #17421 Motor Driver using PWM INPUT
    Author:           Joe Cook
    Equipment:        In BOM Folder
    Notes:            Only implements PID within the smaller integrations
                      brake before incorporating PID temporarily
*/
//Librarys Included
#include <PID_v1.h>

//Variable Definitions
  
  //(PWM MEASURE) 
    //mark to space ratio measuring position variables
    #define pulsePin D5  // Define the input pin for the pulse signal
    volatile double pulseStartTime = 0; // Variable to store the start time of the pulse (voltatile used to ensure updates in isr)
    volatile double pulseEndTime = 0;   // Variable to store the end time of the pulse
    volatile double pulseWidth = 0;     // Variable to store the pulse width (needs to be motor starting position CALIBRATE)

    volatile boolean debug_code = false; //If True PWM will print
    volatile boolean flag = 0; //used in switch to ensure calculation only occurs on low edge

  //(MOTOR CONTROL)
    //error value
    int error = 0; //int so that no decimals
    int tolerance = 5; //to provide deadband around setpoint 
    //motor output variables
    int clockwise = D10; //powers clockwise pin on DRV8876 H-Bridge
    int anticlockwise = D9; //powers anticlockwise pin on DRV8876 H-Bridge
    //Speed Variables
    double speed = 0; 
    //LED Indicators
    #define blueLED A2
    #define redLED A3
    #define greenLED D4
    //OVERSHOOT CHECKING
    #define setpointMet D7
    //Switch Values
    int motorSwitch = 0;
    #define B 0
    #define LCW 1
    #define LACW 2
    #define FCW 3
    #define FACW 4
    #define DELAY_FCW 5
    #define DELAY_FACW 6
    //Delay Variables
    unsigned long FCWdelayStart = 0; 
    unsigned long FACWdelayStart = 0;
    const long delayInterval = 1000; //time delay

  //SLIDING WINDOW for Feedback Potentiometer and PWM Measurement
    //position variables
    #define positionFeedback A0 //motor rotory feedback from 10 turn POT
    double positionFeedbackReading; //to analog read the position feedback pin 
    //Time delay
    double storedTimeStamp = 0;
    //values for sliding window
    const unsigned int numReadings = 100; //ammount of sampling values in the array
    //array with size
    double potValues[numReadings]; 
    double pwmValues[numReadings];
    unsigned int i = 0; //pointer for if function
    //value for summing the array
    double potSum = 0; 
    double pwmSum = 0;
    //value for average of array
    double averageFeedbackValue = 0; 
    double averagePWMValue = 0;

  //mapping PWM Variables 
  double mappedPWM = 0; //variable for mapping PWM value to motor position value
  double maxWidth = 4000; //max us pulse can be (dependant on frequency used)
  //values found by manually altering pot to exactly 7 turns
  double lowValue = 430;
  double highValue = 3288;
  
  //Adaptive PID Variables
  int rotationBand = 409; //Error < than this then  step value used
  double Fkp = 1, Fki = 0.125, Fkd = 0; //rotation step values
  double Setpoint, Feedback, Output;

  PID myPID(&Feedback, &Output, &Setpoint, Fkp, Fki, Fkd, DIRECT);

void setup() {
  //(PWM MEASURE)
    pinMode(pulsePin, INPUT_PULLUP); // Set pulsePin as input
    attachInterrupt(digitalPinToInterrupt(pulsePin), changingEdgeISR, CHANGE); //Attaches interrupt to the PWM pin
  //(MOTOR CONTROL)
    //setting input pins
    pinMode(positionFeedback, INPUT_PULLDOWN);
    //setting output pins
    pinMode(clockwise, OUTPUT);
    pinMode(anticlockwise, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    pinMode(blueLED, OUTPUT);

    //setting up analogRead to 12 bit resolution
    analogReadResolution(12);

  //(SERIAL MONITORING)
    Serial.begin(9600); 

  //OVERSHOOT CHECKING
    pinMode(setpointMet, OUTPUT); //measure using logic analyser then measure 
  
  //INITIALIZE PID
    myPID.SetMode(AUTOMATIC);
    //set the output limits (default is 0,255)
    myPID.SetOutputLimits(0, 4096);

  //Stop movement on startup due to tolerancing
    motorSwitch = B;

}
void loop() {

  SlidingWindow();

  motorControl();

  excelPlotting();

    }


//(MOTOR CONTROL)
void motorControl(){
  //Set PID Values to Values defined within program
  Feedback = averageFeedbackValue;
  Setpoint = mappedPWM;
  //error value
  error = Setpoint - Feedback;
  myPID.Compute();
  //speed limit
  speed = abs(Output); //sets speed always a positive number
  if(speed > 255) speed = 255; //upper limit
  
  //set direction and speed according to PID value
  //Brake Mode
  if (error == 0) motorSwitch = B;
  //Set Large Step Mode
    //Clockwise Control
  if (error > 0 + rotationBand) motorSwitch = LCW;
    //Anticlockwise Control
  if (error < 0 - rotationBand) motorSwitch = LACW;
  //Set Small Step Mode
    //Clockwise Control
  if ((error < 0 + rotationBand) && (error > 0 + tolerance)){
    if(motorSwitch != FCW && motorSwitch != DELAY_FCW){ //initially send into the delay function
      motorSwitch = DELAY_FCW;
      FCWdelayStart = millis(); //take initial time stamp
    }
  }
    //Anticlockwise Control
  if ((error > 0 - rotationBand) && (error < 0 - tolerance)){
    if(motorSwitch != FACW && motorSwitch != DELAY_FACW){ //initially send into the delay function
    motorSwitch = DELAY_FACW;
    FACWdelayStart = millis(); //take initial time stamp
    }
  }

switch(motorSwitch){
  //Brake (1)
  case(B):    Output = 0; //resets ki to reduce creep up
              myPID.SetTunings(0, 0, 0); //resets ki to reduce creep up
              digitalWrite(clockwise, 1);
              digitalWrite(anticlockwise, 1);
              setColour(255, 0, 255); //green
              break;
  //Large Step Clockwise (1)
  case(LCW):  analogWrite(clockwise, 255);
              analogWrite(anticlockwise, 0);
              setColour(255, 0, 100); //torquoise
              break;
  //Large Step Anticlockwise (2)
  case(LACW): analogWrite(clockwise, 0);
              analogWrite(anticlockwise, 255);
              setColour(0, 255, 255); //red
              break;
  //Full rotation Step Clockwise (3)
  case(FCW):  myPID.SetTunings(Fkp, Fki, Fkd);
              myPID.SetControllerDirection(DIRECT);
              myPID.Compute();
              analogWrite(clockwise, speed);
              analogWrite(anticlockwise, 0);
              setColour(255, 0, 100); //torquoise
              break;
  //Full rotation Step Anticlockwise (4)
  case(FACW): myPID.SetTunings(Fkp, Fki, Fkd);
              myPID.SetControllerDirection(REVERSE);
              myPID.Compute();
              analogWrite(clockwise, 0);
              analogWrite(anticlockwise, speed);
              setColour(0, 255, 255); //red
              break;  
  //Delay Before Full rotation Step Clockwise (5)
  case(DELAY_FCW):
              if(millis() - FCWdelayStart >= delayInterval){ //check whether interval time has passed
                motorSwitch = FCW; //set 360 rotation values if time passed
              } else {
                // Keep the motor in brake mode during the delay
                digitalWrite(clockwise, 1);
                digitalWrite(anticlockwise, 1);
              }
              break;
  case(DELAY_FACW):
              if(millis() - FACWdelayStart >= delayInterval){ //check whether interval time has passed
                motorSwitch = FACW; //set 360 rotation values if time passed
              } else {
                // Keep the motor in brake mode during the delay
                digitalWrite(clockwise, 1);
                digitalWrite(anticlockwise, 1);
              }
              break;
  }
}

//ISR For mark measurement on both edges USING IF
void changingEdgeISR(){  
 flag = (digitalRead(pulsePin));
 switch(flag){
  
    case (1) : pulseStartTime = micros(); break;
               
    case (0) : pulseEndTime = micros();
               pulseWidth = pulseEndTime - pulseStartTime; 
    }
}


//AVERAGING both the Pot values and the PWM Values
void SlidingWindow(){
    //Delay function (alter to micros if needed)
  unsigned long currentMicros = micros();//stamp of time since arduino started up

  if(currentMicros >= storedTimeStamp + 10){ //alter number for delay time wanted (10 us) 
    storedTimeStamp = currentMicros; //stores current value to be compared with next measured value
    
    // Remove the oldest value from the sum
    potSum -= potValues[i];
    pwmSum -= pwmValues[i];
    
    // Read the new value and add it to the sum
    potValues[i] = analogRead(positionFeedback);
    potSum += potValues[i];
    pwmValues[i] = pulseWidth;
    pwmSum += pwmValues[i];
    
    // Move to the next position in the array
    i = (i + 1) % numReadings;

    // Calculate the average
    averageFeedbackValue = potSum / numReadings;
    averagePWMValue = pwmSum / numReadings;
    //Mapping PWM values to match analogReadResolution
    mappedPWM = map(averagePWMValue, 0, maxWidth, lowValue, highValue);
 
  }
}

//LED 
void setColour(int redValue, int greenValue, int blueValue) {
  analogWrite(redLED, redValue);
  analogWrite(greenLED, greenValue);
  analogWrite(blueLED, blueValue);
}

void excelPlotting(){
  //Delay function (alter to micros if needed)
  static unsigned long storedTimeStamp = 0;
  unsigned long currentTime = millis(); //stamp of time since arduino started up
  if(currentTime >= storedTimeStamp + 200){ //alter number for delay time wanted
    storedTimeStamp = currentTime; //stores current value to be compared with next measured value
  Serial.print(micros() / 1e6);
  Serial.print(":");
  Serial.print(Setpoint);
  Serial.print(":");
  Serial.print(Feedback);
  Serial.print(":");
  Serial.println(speed);
 // Serial.print(":");
  //Serial.println(motorSwitch);
  }
}


