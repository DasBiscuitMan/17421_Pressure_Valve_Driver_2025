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
    volatile long pulseWidth = 0;     // Variable to store the pulse width (needs to be motor starting position CALIBRATE)
    //flags to indicate 1 cycle
    volatile long flag = 0; 
    //mapping PWM Variables 
    int mappedPWM = 0; //variable for mapping PWM value to motor position value
    int maxWidth = 2000; //max us pulse can be (dependant on frequency used)
  //(MOTOR CONTROL)
    //position variables
    #define positionFeedback A0 //motor rotory feedback from 10 turn POT
    int positionFeedbackReading; //to analog read the position feedback pin 
    //motor output variables
    int clockwise = D10; //powers clockwise pin on DRV8876 H-Bridge
    int anticlockwise = D9; //powers anticlockwise pin on DRV8876 H-Bridge
    //LED Indicators
    #define blueLED A2
    #define redLED A3
    #define greenLED D4

    //tolerancing variables
    int toleranceValue = 10; //value for tolerance so can be altered in main program
    int tolerance = toleranceValue; //actual plus minus value for tolerancing 
    int flagTolerance = 0; //flag to ensure value is met before tolerance is used

  

  //SLIDING WINDOW for Feedback Potentiometer and PWM Measurement
    //Time delay
    unsigned long storedTimeStamp = 0;
    //values for sliding window
    const unsigned int numReadings = 100; //ammount of sampling values in the array
    //array with size
    unsigned int potValues[numReadings]; 
    unsigned int pwmValues[numReadings]; 
    unsigned int i = 0; //pointer for if function
    //value for summing the array
    unsigned int potSum = 0; 
    unsigned int pwmSum = 0; 
    //value for average of array
    unsigned int averageFeedbackValue = 0; 
    unsigned int averagePWMValue = 0;

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
    Serial.begin(9600); //CANNOT BE USED WHEN NOT USING PC (MESSES UP)
}
void loop() {
//(PWM MEASURE)
  pwmMeasure();

  SlidingWindow();
  
  motorControl();
    
  serialPrinting();
  
    }

//ISR For mark measurement on both edges USING IF
void changingEdgeISR(){  
  if(digitalRead(pulsePin) == 1 & flag == 0){ //adding flag makes sequential so that high pin is read first
  pulseStartTime = micros(); //measures stamp in microseconds
  flag = 1; //checks whether pulse gone high
  }
  if(digitalRead(pulsePin) == 0 & flag == 1){
  pulseEndTime = micros(); //measures stamp in microseconds
  flag = 2; //checks whether pulse gone low
  }
}

//(PWM MEASURE)
void pwmMeasure(){
    if(flag == 2){
        pulseWidth = pulseEndTime - pulseStartTime; //subtracts stamp on rising edge from stamp on falling edge
        flag = 0;
    }
  //mapping procedure for pulse measurement  
  mappedPWM = map(pulseWidth, 0, maxWidth, 0, 4096); //4096 is 12 bit max ADC value 2^12
}


//(MOTOR CONTROL)
void motorControl(){
    //logic system to control motor rotation and brake 
    //brake if motor in correct position
    if((averagePWMValue - averageFeedbackValue) <= tolerance){
      tolerance = toleranceValue; //tolerance set again to allow for deadband
      digitalWrite(clockwise, 1);
      digitalWrite(anticlockwise, 1);
      setColour(200, 1, 127); //0 = high
    }
    //rotate clockwise if motor is lower than correct position
    if(averagePWMValue > (averageFeedbackValue + tolerance)){
      tolerance = 0; //set to 0 to stop deadband being in place in movement
      digitalWrite(clockwise, 1);
      digitalWrite(anticlockwise, 0);
      setColour(0, 1, 255); //0 = high
    }
    //rotate clockwise if motor is lower than correct position
    if(averagePWMValue < (averageFeedbackValue - tolerance)){
      tolerance = 0; //set to 0 to stop deadband being in place in movement
      digitalWrite(clockwise, 0);
      digitalWrite(anticlockwise, 1);
      setColour(0, 1, 255); //0 = high
    }
}

void serialPrinting(){
    Serial.print("pulseWidth ");
    Serial.print(pulseWidth);
    Serial.print(" mapped pulseWidth ");
    Serial.print(mappedPWM);
    Serial.print(" average pulseWidth ");
    Serial.print(averagePWMValue);
    Serial.print(" motor position ");
    Serial.print(averageFeedbackValue);
    Serial.print(" clockwise ");
    Serial.print(digitalRead(clockwise));
    Serial.print(" anticlockwise ");
    Serial.println(digitalRead(anticlockwise));
}

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
    pwmValues[i] = mappedPWM;
    pwmSum += pwmValues[i];
    
    // Move to the next position in the array
    i = (i + 1) % numReadings;

    // Calculate the average
    averageFeedbackValue = potSum / numReadings;
    averagePWMValue = pwmSum / numReadings;

  }
}

void setColour(int redValue, int greenValue, int blueValue) {
  analogWrite(redLED, redValue);
  digitalWrite(greenLED, greenValue);
  analogWrite(blueLED, blueValue);
}




