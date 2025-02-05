/*  Title:            PPMS #17421 Motor Driver using PWM INPUT
    Author:           Joe Cook
    Equipment:        In BOM Folder
    Notes:            Differences from Version 4
                      doesnt use dynamic tolerance, instead uses similar tolerance to version 1            
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
    int mappedPWM = 0; //variable for mapping PWM value to motor position value
    int maxWidth = 4000; //max us pulse can be (dependant on frequency used)

  //(MOTOR CONTROL)
    //position variables
    #define positionFeedback A0 //motor rotory feedback from 10 turn POT
    int positionFeedbackReading; //to analog read the position feedback pin 
    //motor output variables
    int clockwise = D10; //powers clockwise pin on DRV8876 H-Bridge
    int anticlockwise = D9; //powers anticlockwise pin on DRV8876 H-Bridge
    //Switch statement Variables
    #define ON_TARGET 1
    #define CLOCKWISE 2
    #define ANTICLOCKWISE 3
    int switchValue = 0;
    //LED Indicators
    #define blueLED A2
    #define redLED A3
    #define greenLED D4
    //tolerancing variables
    int tolerance = 5; //value for tolerance so can be altered in main program
    signed int positionDifference; //value for simplified if statement
    //speed variables
    int speed = 10; //standard speed 
    int slowSpeed = 110; //speed within slowing band
    int slowBand = 100; //band to slow speed hopefully reducing overshoot
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
}

//(MOTOR CONTROL)
void motorControl(){
    //Mapping PWM values
    mappedPWM = map(averagePulseWidth, 0, maxWidth, 0, 4096);
    // Calculate the difference between the current position and the target position
    positionDifference = (mappedPWM - averageFeedbackValue);

   //designed to stop motor moving until full average calculation complete 
    // Logic system to control motor rotation and brake 
    if (abs(positionDifference) == 0) { // If there is no difference between current and target
        switchValue = ON_TARGET; // Brake if motor in correct position;
     } 
    if (positionDifference > (0 + tolerance)) { // 
          switchValue = CLOCKWISE; // Rotate clockwise
    } 
    if (positionDifference < (0 - tolerance)) { // 
          switchValue = ANTICLOCKWISE; // Rotate anticlockwise
    }

  switch (switchValue) {
    case(ON_TARGET):
      //OVERSHOOT CHECKING
      digitalWrite(setpointMet, HIGH); //measure using logic analyser then measure 

      analogWrite(clockwise, 255);
      analogWrite(anticlockwise, 255);
      setColour(200, 1, 127); //0 = high
      break;

    case(CLOCKWISE):
      //OVERSHOOT CHECKING
      digitalWrite(setpointMet, LOW); //measure using logic analyser then measure 

    if (positionDifference > 0 + slowBand){
      analogWrite(clockwise, speed);
      analogWrite(anticlockwise, 0);
      setColour(0, 1, 255); //0 = high
    }
    if (positionDifference < 0 + slowBand){
      analogWrite(clockwise, slowSpeed);
      analogWrite(anticlockwise, 0);
      setColour(0, 1, 255); //0 = high
    }
      break;
    
    case(ANTICLOCKWISE):
      //OVERSHOOT CHECKING
      digitalWrite(setpointMet, LOW); //measure using logic analyser then measure 

    if (positionDifference < 0 - slowBand){
      analogWrite(clockwise, 0);
      analogWrite(anticlockwise, speed);
      setColour(0, 1, 255); //0 = high
    }
    if (positionDifference > 0 - slowBand){
      analogWrite(clockwise, 0);
      analogWrite(anticlockwise, slowSpeed);
      setColour(0, 1, 255); //0 = high
    }
    break;

    default:
      analogWrite(clockwise, 255);
      analogWrite(anticlockwise, 255);
      setColour(225, 0, 255); //0 = high
  }
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
    Serial.print(" switchValue ");
    Serial.print(switchValue);
    Serial.print(" positionDifference ");
    Serial.println(positionDifference);
}


