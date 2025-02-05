/*  Title:            PPMS #17421 Motor Driver using PWM INPUT
    Author:           Joe Cook
    Equipment:        In BOM Folder
    Notes:            Utilizes PID Library by Brett Beuregard          
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

    //averaging Variables
    volatile int numberOfSamplePulses = 500;
    volatile int pulseCount = 0;
    volatile double pulseWidthTotal = 0.0;
    volatile double averagePulseWidth = 0.0;
   
    //mapping PWM Variables 
    double mappedPWM = 0; //variable for mapping PWM value to motor position value
    double maxWidth = 4000; //max us pulse can be (dependant on frequency used)

  //(MOTOR CONTROL)
    //error value
    int error = 0; //int so that no decimals 
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
    unsigned int i = 0; //pointer for if function
    //value for summing the array
    double potSum = 0; 
    //value for average of array
    double averageFeedbackValue = 0; 

  //PID Variables
  double kp = 1;
  double ki = 0;
  double kd = 0;

  double Setpoint, Feedback, Output;

  PID myPID(&Feedback, &Output, &Setpoint, kp, ki, kd, DIRECT);

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

}
void loop() {

  pwmCalculate();

  slidingWindow();

  motorControl();

  excelPlotting();
  //serialPrinting();
  
    }


//(MOTOR CONTROL)
void motorControl(){
  //Set PID Values to Values defined within program
  Feedback = averageFeedbackValue;
  Setpoint = mappedPWM;
  //error value
  error = Setpoint - Feedback;
  //set tunings and compute PID protocol
  myPID.SetTunings(kp, ki, kd);
  myPID.Compute();

  //speed limit
  speed = abs(Output); //sets speed always a positive number
  if(speed > 255) speed = 255; //upper limit
  if(speed < 110) speed = 110; //lower limit (110 no valve 160 with valve)
  
  //set direction and speed according to PID value
  if (error == 0){
    digitalWrite(clockwise, 1);
    digitalWrite(anticlockwise, 1);
    setColour(255, 0, 255); //green
  } 
  if (error > 0){
    myPID.SetControllerDirection(DIRECT);
    analogWrite(clockwise, speed);
    analogWrite(anticlockwise, 0);
    setColour(255, 0, 100); //torquoise
  } 
  if (error < 0){ 
    myPID.SetControllerDirection(REVERSE);
    analogWrite(clockwise, 0);
    analogWrite(anticlockwise, speed);
    setColour(0, 255, 255); //red
  }
}

//ISR For mark measurement on both edges USING IF
void changingEdgeISR(){  
 flag = (digitalRead(pulsePin));
 switch(flag){
  
    case (1) : pulseStartTime = micros(); break;
               
    case (0) : pulseEndTime = micros();
               pulseWidth = pulseEndTime - pulseStartTime; 
               pulseCount++;
               pulseWidthTotal += pulseWidth;
    }
}

void pwmCalculate(){
  //function needed to remove delay in calculations
  if(pulseCount >= numberOfSamplePulses) 
    { pulseCount = 0;
    averagePulseWidth = pulseWidthTotal/numberOfSamplePulses; 
    if (debug_code) Serial.println(averagePulseWidth);
      pulseWidthTotal = 0;   
    }
  mappedPWM = map(averagePulseWidth, 0, maxWidth, 0, 4096);
}

//AVERAGING
void slidingWindow(){
    //Delay function (alter to micros if needed)
  unsigned long currentMicros = micros();//stamp of time since arduino started up

  if(currentMicros >= storedTimeStamp + 10){ //alter number for delay time wanted (10 us) 
    storedTimeStamp = currentMicros; //stores current value to be compared with next measured value
    
    // Remove the oldest value from the sum
    potSum -= potValues[i];
    
    
    // Read the new value and add it to the sum
    potValues[i] = analogRead(positionFeedback);
    potSum += potValues[i];
    
    // Move to the next position in the array
    i = (i + 1) % numReadings;

    // Calculate the average
    averageFeedbackValue = potSum / numReadings;

  }
}
//LED 
void setColour(int redValue, int greenValue, int blueValue) {
  analogWrite(redLED, redValue);
  digitalWrite(greenLED, greenValue);
  analogWrite(blueLED, blueValue);
}

void excelPlotting(){
  
  //Delay function (alter to micros if needed)
  static unsigned long storedTimeStamp = 0;
  unsigned long currentTime = millis(); //stamp of time since arduino started up
  if(currentTime >= storedTimeStamp + 200){ //alter number for delay time wanted
    storedTimeStamp = currentTime; //stores current value to be compared with next measured value
  Serial.print(micros() / 1e6);
  Serial.print(" ");
  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.println(Feedback);
  }
  
}
void serialPrinting(){

    Serial.print(" average pulseWidth ");
    Serial.print(averagePulseWidth);
    Serial.print(" Setpoint ");
    Serial.print(Setpoint);
    Serial.print(" Feedback ");
    Serial.print(Feedback);
    Serial.print(" error ");
    Serial.print(error);
    Serial.print(" Output ");
    Serial.print(Output);
    Serial.print(" Speed ");
    Serial.println(speed);
}


