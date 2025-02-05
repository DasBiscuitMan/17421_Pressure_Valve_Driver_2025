/*  Title:            PPMS #17421 Motor Driver using PWM INPUT
    Author:           Joe Cook
    Equipment:        In BOM Folder
    Notes:            Error has replaced position difference for PID control          
*/
//Variable Definitions
  
  //(PWM MEASURE) 
    //mark to space ratio measuring position variables
    #define pulsePin D5  // Define the input pin for the pulse signal
    volatile long pulseStartTime = 0; // Variable to store the start time of the pulse (voltatile used to ensure updates in isr)
    volatile long pulseEndTime = 0;   // Variable to store the end time of the pulse
    volatile long pulseWidth = 0;     // Variable to store the pulse width (needs to be motor starting position CALIBRATE)

    volatile boolean debug_code = false; //If True PWM will print
    volatile boolean flag = 0; //used in switch to ensure calculation only occurs on low edge

    //averaging Variables
    volatile int numberOfSamplePulses = 500;
    volatile int pulseCount = 0;
    volatile long pulseWidthTotal = 0;
    volatile long averagePulseWidth = 0;
   
    //mapping PWM Variables 
    long mappedPWM = 0; //variable for mapping PWM value to motor position value
    long maxWidth = 4000; //max us pulse can be (dependant on frequency used)

  //(MOTOR CONTROL)
    //position variables
    #define positionFeedback A0 //motor rotory feedback from 10 turn POT
    int positionFeedbackReading; //to analog read the position feedback pin 
    //motor output variables
    int clockwise = D10; //powers clockwise pin on DRV8876 H-Bridge
    int anticlockwise = D9; //powers anticlockwise pin on DRV8876 H-Bridge
    //Speed Variables
    int speed = 0; 
    //LED Indicators
    #define blueLED A2
    #define redLED A3
    #define greenLED D4
    //OVERSHOOT CHECKING
    #define setpointMet D7

  //SLIDING WINDOW for Feedback Potentiometer and PWM Measurement
    //Time delay
    unsigned long storedTimeStamp = 0;
    //values for sliding window
    const unsigned int numReadings = 100; //ammount of sampling values in the array
    //array with size
    unsigned int potValues[numReadings]; 
    unsigned int i = 0; //pointer for if function
    //value for summing the array
    unsigned int potSum = 0; 
    //value for average of array
    unsigned int averageFeedbackValue = 0; 

  //PID 
  //Gains
  float kp = 0.1; 
  float ki = 0.001; //
  float kd = 0;
  float u = 0; //PID calculation
  //delta t variables
  float deltaT;
  int currentTime;
  int previousTime;
  //error variables
  long error;
  float previousError;
  float eIntegral;
  float eDerivative;

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
}
void loop() {

  pwmCalculate();

  slidingWindow();
  
  PID();

  motorControl();
    
  serialPrinting();
  
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

//(MOTOR CONTROL)
void motorControl(){

  //speed limit
  speed = fabs(u); //sets speed always a positive number
  if(speed > 255) speed = 255; //upper limit
  if(speed < 120) speed = 200; //lower limit
  
  //set direction and speed according to PID value
  if (u == 0){
    digitalWrite(clockwise, 1);
    digitalWrite(anticlockwise, 1);
    setColour(255, 0, 255); //green
  } 
  if (u > 0){
    analogWrite(clockwise, speed);
    analogWrite(anticlockwise, 0);
    setColour(255, 0, 100); //torquoise
  } 
  if (u < 0){ 
    analogWrite(clockwise, 0);
    analogWrite(anticlockwise, speed);
    setColour(0, 255, 255); //red
  }

}


//PID Control
void PID(){
  //recording deltaT
  currentTime = micros();
  deltaT = ((float) currentTime - previousTime) / 1.0e6; //converts to seconds
  //Proportional, Integral and Derivative
  error = mappedPWM - averageFeedbackValue; //proportional error
  eIntegral = eIntegral + error * deltaT; //integral value
  eDerivative = error - previousError / deltaT; //delta value
  //PID Calculation
  u = (kp * error) + (ki * eIntegral) + (kd * eDerivative);
  //setting values for next iteration
  previousTime = currentTime;
  previousError = error;
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

void serialPrinting(){

    Serial.print(" average pulseWidth ");
    Serial.print(averagePulseWidth);
    Serial.print(" mapped pulseWidth ");
    Serial.print(mappedPWM);
    Serial.print(" motor position ");
    Serial.print(averageFeedbackValue);
    Serial.print(" error ");
    Serial.print(error);
    Serial.print(" PID Value ");
    Serial.print(u);
    Serial.print(" Speed ");
    Serial.println(speed);
}


