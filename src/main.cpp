/*  Title:            PPMS #17421 Motor Driver using PWM INPUT
    Author:           Joe Cook
    Equipment:        In BOM Folder
    Notes:            Utilizes PID Library by Brett Beuregard, 
                      Uses sliding window for PWM averaging to reduce lag    
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
    int tolerance = 0; //to provide deadband around setpoint 
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

  //Adaptive PID Variables
  double Lkp = 1, Lki = 0.004, Lkd = 0; //large step values
  int smallStepBand = 100; //Error < than this then small step values used
  char smallStepIndicator = 'B'; //Eliminates band entry 
  double Skp = 10, Ski = 0.5, Skd = 0; //Small step values

  double Setpoint, Feedback, Output;

  PID myPID(&Feedback, &Output, &Setpoint, Lkp, Lki, Lkd, DIRECT);

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

  SlidingWindow();

  pwmCalculate();

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
  //set small or large tunings and compute PID protocol
  if((abs(error) < smallStepBand) && (smallStepIndicator != 'L')){ //& means wont go into this band when using large step
  myPID.SetTunings(Skp, Ski, Skd);
  //Set flag
  smallStepIndicator = 'S'; 
  }
  else if (abs(error) > smallStepBand && smallStepIndicator != 'S'){}
  myPID.SetTunings(Lkp, Lki, Lkd);
  //Set flag
  smallStepIndicator = 'L';
  }
  myPID.Compute();

  //speed limit
  speed = abs(Output); //sets speed always a positive number
  if(speed > 255) speed = 255; //upper limit
  //if(speed < 110) speed = 110; //lower limit (110 no valve 160 with valve)
  
  //set direction and speed according to PID value
  if (error == 0){
    smallStepIndicator = 'B'; //Reset Flag
    digitalWrite(clockwise, 1);
    digitalWrite(anticlockwise, 1);
    setColour(255, 0, 255); //green
  } 
  if (error > 0 + tolerance){
    myPID.SetControllerDirection(DIRECT);
    analogWrite(clockwise, speed);
    analogWrite(anticlockwise, 0);
    setColour(255, 0, 100); //torquoise
  } 
  if (error < 0 - tolerance){ 
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

  }
}

//Calculate PWM
void pwmCalculate(){
  //Delay function seems to give stabler values, also means the values arent constantly changing
  static unsigned long storedTimeStamp = 0;
  unsigned long currentTime = millis(); //stamp of time since arduino started up
  if(currentTime >= storedTimeStamp + 500){ //alter number for delay time wanted
    storedTimeStamp = currentTime; //stores current value to be compared with next measured value

  //Mapping PWM values to match analogReadResolution
  mappedPWM = map(averagePWMValue, 0, 4000, 0, 4096);

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
  Serial.print(Feedback);
  Serial.print(" ");
  Serial.print(speed);
  Serial.print(" ");
  Serial.println(smallStepIndicator);
  
  }
  
}
void serialPrinting(){

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


