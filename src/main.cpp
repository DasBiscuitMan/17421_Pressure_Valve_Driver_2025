/*  Title:            PPMS #17421 Motor Driver using PWM INPUT
    Author:           Joe Cook
    Equipment:        In BOM Folder
    Notes:            Implementing current sense and calibration
*/
//Librarys Included
  #include <Arduino.h>
  #include <PID_v1.h>

// Function Prototypes
  void pwmTimeout(); //Used to add a timeout to PWM so doesnt latch high
  void calibrate(); //Maintenance and new valve setup
  void calibrateMotorControl(); //Motor control with limits removed
  void readString(); //Reads messages sent to serial buffer
  void printInstructions(); //Reads instructions within the calibration routine
  void changingEdgeISR(); //Measures PWM signal in
  void SlidingWindow(); //Averages PWM and Analog Potentiometer feedback signal
  void currentSlidingWindow(); //Averages motor current
  void motorControl(); //Controls motor using PID depending on paramters
  void computeError(); //Computes error for use in motorControl
  void setColour(int redValue, int greenValue, int blueValue); //Controls RGB LED
  void excelPlotting(); //Used for data analysis (system response, PID tuning etc.)


//Variable Definitions
  
  //(PWM MEASURE) 
    //mark to space ratio measuring position variables
    #define pulsePin D1  // Define the input pin for the pulse signal
    volatile double pulseStartTime = 0; // Variable to store the start time of the pulse (voltatile used to ensure updates in isr)
    volatile double pulseEndTime = 0;   // Variable to store the end time of the pulse
    volatile double pulseWidth = 0;     // Variable to store the pulse width (needs to be motor starting position CALIBRATE)
    volatile double lastPulseTime = 0; //Variable to add timeout for PWM measure so doesnt latch on
    const double timeoutThreshold = 5000; //stored time to cause timeout (us)

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
    #define blueLED D4
    #define redLED D5
    #define greenLED D6
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
    #define OUT_OF_BOUNDS 7
    //Delay Variables
    unsigned long FCWdelayStart = 0; 
    unsigned long FACWdelayStart = 0;
    const long delayInterval = 30; //time delay
    /* Outer Limit Values for 7 turns remove motor (leave pot on) 
    turn valve to closed attach motor shaft but not motor
    keep pot attached
    rotate valve until open 
    set these open and closed limits as values */
    double lowerLimit = 420; //
    double upperLimit = 3246; //
    //Blipping System
    int rotationBand = 200; //Error < than this then  blip value used
    //delay for blipping 
    const long onDelay = 20; 
    const long offDelay = 500;
    //non blocking delay for blip
    unsigned long ACWpreviousMillis = 0; 
    unsigned long CWpreviousMillis = 0;
    unsigned long ACWcurrentMillis = 0;
    unsigned long CWcurrentMillis = 0;

  //SLIDING WINDOW for Feedback Potentiometer and PWM Measurement
    //position variables
    #define positionFeedback A0 //motor rotory feedback from 10 turn POT
    double positionFeedbackReading = 2000; //to analog read the position feedback pin 
    //Time delay
    double storedTimeStamp = 0;
    //values for sliding window
    const unsigned int numReadings = 10; //ammount of sampling values in the array
    //array with size
    double potValues[numReadings]; 
    double pwmValues[numReadings];
    unsigned int i = 0; //pointer for if function
    //value for summing the array
    double potSum = 0; 
    double pwmSum = 0;
    //value for average of array
    double averageFeedbackValue = 2000; //value to ensure no initial calibration error
    double averagePWMValue = 0;
    //system to only begin displaying values once initial 10 values have been set into array (startup errror fix)
    int arrayCounter = 0; //count how many initial array entrances there have been
    bool arrayComplete = false; //check whether initial array full
  
  //mapping PWM Variables 
    double mappedPWM = 0; //variable for mapping PWM value to motor position value
    double maxWidth = 4000; //max us pulse can be (dependant on frequency used)
  //upper and lower limit values found by manually altering pot 
    double lowValue = 14;
    double highValue = 4081;

  //Sliding Window for Current Sense (needs to be seperate)
    //Motor Current Variables
    #define currentSense A3
    double currentSenseRead = 0;
    //Time Delay
    double storedTimeStampmA = 0;
    const unsigned int InumReadings = 1000; //ammount of sampling values in the array
    double currentValues[InumReadings];
    double currentSum = 0;
    double averageCurrentValue = 0;
    unsigned int imA = 0; //pointer for if function
    const double currentLimit = 200; //overcurrent limit
  
  //Adaptive PID Variables

    double Fkp = 0.6, Fki = 0.05, Fkd = 0; //rotation step values Kp 1 Ki 0.125
    double Setpoint, Feedback, Output;

    PID myPID(&Feedback, &Output, &Setpoint, Fkp, Fki, Fkd, DIRECT);

  //Serial Monitoring Variables
    double serialStoredTimeStamp = 0;

  //CALIBRATE Variables
    bool calibrateFlag = false; 
    bool manFlag = false;
    bool guideFlag = false;
    bool PWMFlag = false;
    bool potFlag = false;
    bool homeFlag = false;
    int homeSetpoint;
    String readstr = ""; //String declaration

void setup() 
{
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

    //CURRENT SENSE
    pinMode(currentSense, INPUT);

  //REMOVE CALIBRATE ENTER
    //Stop movement on startup due to tolerancing
    motorSwitch = B;
    SlidingWindow();
    SlidingWindow();
    SlidingWindow();

}

void loop() 
{

  calibrate();
  
  pwmTimeout();

  SlidingWindow();

  currentSlidingWindow();

  computeError();
  
  motorControl();

  //excelPlotting();

}

//(CALIBRATION ROUTINE)
void calibrate()
{
  
  //Read Serial Buffer
  readString();

  //(Calibrate Routine Entering Parameters)
  //Manual Entrance
  if(readstr == "c")
  {
    calibrateFlag = true; //latches in calibrate mode
    readstr = "l"; //unused and stops exiting and entering
    printInstructions();
  }
  //Fatal Error Entrance (Potentiometer Runover)
  if(averageFeedbackValue > upperLimit + 10 || averageFeedbackValue < lowerLimit - 10) 
  {
    calibrateFlag = true; //latches in calibrate mode
    readstr = "l"; //unused and stops exiting and entering
    Serial.println("POTENTIOMETER RUNOVER!");
    Serial.println("");
    printInstructions();
  }
  //Fatal Error Entrance (Over Current)
  if(currentSenseRead > currentLimit)
  {
    calibrateFlag = true; //latches in calibrate mode
    readstr = "l"; //unused and stops exiting and entering
    Serial.println("OVERCURRENT!");
    Serial.println("");
    printInstructions();
  }
  //(Enter Calibrate Routine)
  while(calibrateFlag == true) //beginning of calibrate loop
  {
    //Place Motor in break
    digitalWrite(clockwise, 1);
    digitalWrite(anticlockwise, 1);
  
    //Read Serial Port within calibrate loop
    readString();
    //CALIBRATE ROUTINES
    //Add guide to commisioning a new valve
  

    //(Guide)
    if(readstr == "g")
    {
      guideFlag = true;
      //Guide Here
      Serial.println("Guide ");
      Serial.println("");
      Serial.println("(1) Measure PWM Signal:");
      Serial.println("    Enter PWM Measurment Routine Through the Main Menu");
      Serial.println("    Ensure PWM in is Measured and Mapped Correctly");
      Serial.println("    PWM Calibration Complete");
      Serial.println("");
      Serial.println("(2) Potentiometer Damaged:");
      Serial.println("    Replace Potentiometer");
      Serial.println("    Enter Potentiometer Measurement Routine Through the Main Menu");
      Serial.println("    Measure Fully Clockwise and Fully Anticlockwise Values");
      Serial.println("    Set these as 'lowValue' and 'highValue'");
      Serial.println("    Set 'lowerLimit' to 10% of 'highValue' - ' lowValue'");
      Serial.println("    Rotate the Potentiometer to 'lowerLimit'");
      Serial.println("    Rotate the Valve Fully Closed");
      Serial.println("    Now Attach Valve to the Shaft but NOT to the motor");
      Serial.println("    Fully Open the Valve (7 turns) so the Potentiometer Rotates with it");
      Serial.println("    Set 'upperLimit' to the Fully Opened Value ");
      Serial.println("    Pot Calibration Complete");
      Serial.println("");
      Serial.println("(3) Manual Motor Control:");
      Serial.println("    Enter Manual Control Through the Main Menu");
      Serial.println("    Jog Motor to Required Position");
      Serial.println("    Manual Control Complete");
      Serial.println("");
      Serial.println("(4) Homing Sequence:");
      Serial.println("    Enter 'Home Motor' Through the Main Menu");
      Serial.println("    Ensuring Feedback is present Motor will Rotate to Home Position aka lowerLimit");
      Serial.println("    ");
      Serial.println("");
      Serial.println("To Exit Guide and Return to Calibrate Menu Press 'd'");
     } 
    while(guideFlag == true)
    {
      readString();
      if(readstr == "d")
      {
      guideFlag = false;
      calibrateFlag = true;
      Serial.println("Returning to Calibrate");
      delay(1000);
      printInstructions();
      break; // Exit guide loop 
      }
    }
    
    //Manual Rotation
    if(readstr == "m") 
    {      
      Serial.println("Manual Rotation");
      Serial.println("");
      Serial.println("Press 's' to shut");
      Serial.println("Press 'o' to open");
      Serial.println("Press 'b' to brake");
      Serial.println("");
      Serial.println("Press 'd' if done");
      delay(2000);
      manFlag = true;
    }
    while(manFlag == true)
    {
      readString();
      // Immediately check for "cancel"
      if(readstr == "d")
      {
        manFlag = false;
        calibrateFlag = true;
        Serial.println("Returning to Calibrate");
        delay(1000);
        printInstructions();
        break; // Exit guide loop 
      }
      // Rotation Modes
      if (readstr == "s") // close/shut
      {
        // Start the motor rotation for a brief period
        digitalWrite(clockwise, 0);
        digitalWrite(anticlockwise, 1);
        setColour(0, 255, 255); // red
        delay(25); // small delay for a short movement
        readstr = "p"; //stops string being s
        digitalWrite(anticlockwise, 0);
      }

      if (readstr == "o") // open
      {
        // Start the motor rotation for a brief period
        digitalWrite(clockwise, 1);
        digitalWrite(anticlockwise, 0);
        setColour(0, 255, 255); // red
        delay(20);
        readstr = "p"; //stops string being o
        digitalWrite(clockwise, 0);
      }
      if (readstr == "b") //brake
      {
        digitalWrite(clockwise, 1);
        digitalWrite(anticlockwise, 1);
        setColour(0, 255, 255); // red
        delay(500);
        readstr = "p"; //stops string being o
        digitalWrite(clockwise, 0);
        digitalWrite(anticlockwise, 0);
      }
    }

    //(PWM In Measurement)
    if(readstr == "p")
    {
      Serial.print("Entering PWM Measurement");
      delay(2000);
      PWMFlag = true;
    }
    while(PWMFlag == true)
    {
      readString();
      // Immediately check for "cancel"
      if(readstr == "d")
      {
        PWMFlag = false;
        calibrateFlag = true;
        Serial.println("Returning to Calibrate");
        delay(1000);
        printInstructions();
        break; // Exit PWM loop 
      }
      
      SlidingWindow();

      Serial.print("PWM Mark Length In: "); Serial.print(averagePWMValue); Serial.print("us ");
      Serial.print("Mapped PWM: "); Serial.print(mappedPWM); Serial.println("us ");
    }

    //(Potentiometer Measurement)
    if(readstr == "f") 
    {      
      Serial.print("Entering Potentiometer Measurement");
      delay(2000);
      potFlag = true;
    }
    while(potFlag == true)
    {
      readString();
      // Immediately check for "cancel"
      if(readstr == "d")
      {
        potFlag = false;
        calibrateFlag = true;
        Serial.println("Returning to Calibrate");
        delay(1000);
        printInstructions();
        break; // Exit guide loop 
      }
      //Pot measurement
      SlidingWindow();
      Serial.print("Potentiometer Position ");
      Serial.println(averageFeedbackValue);
    }

  
    //(Home Loop)
    if(readstr == "h") 
    {
      Serial.println("Entering Homing Sequence");
      delay(2000);
      homeFlag = true;
    }
    while(homeFlag == true)
    { 
      // Immediately check for "cancel"
        readString();
        if (readstr == "d") 
        {       
          homeFlag = false;
          calibrateFlag = true; // Keep calibrate mode active
          Serial.println("Home Set");
          Serial.println("Returning to Calibrate");
          delay(1000);
          printInstructions();
          break; // Exit home loop 
        }

      Serial.print(lowerLimit); Serial.print(" "); Serial.print(averageFeedbackValue); Serial.print(" "); Serial.print(error); Serial.print(" "); Serial.println(motorSwitch);
      SlidingWindow();
      //Altered Compute Error
      //Set PID Values to Values defined within program
      Feedback = averageFeedbackValue;
      Setpoint = lowerLimit; //Alters to home setpoint value
      //ERROR CHECKING
      //check whether pwm in has exceeded 10-80% limits
      if(Setpoint > upperLimit) Setpoint = upperLimit;
      if(Setpoint < lowerLimit) Setpoint = lowerLimit;
      //error value
      error = Setpoint - Feedback;
      myPID.Compute();
      //speed limit
      speed = abs(Output); //sets speed always a positive number
      if(speed > 255) speed = 255; //upper limit

      //Motor Control With Out of Bounds Removed
      calibrateMotorControl();
    }

    //(Exit calibration routine)
    if(readstr == "x")
    {
      Serial.println("Exiting Calibration Routine");
      calibrateFlag = false;
    }
  }
}

//Allow repeating function multiple times
void readString()
{
    if (Serial.available() > 0) //wait for data available
  {     
  readstr = Serial.readString();  //read until timeout
  readstr.trim();   
  }
}

//Reads Instructions within Calibration Routine
void printInstructions()
{
    Serial.println(" ");
    Serial.println("Entering Calibrate Mode"); //Inform entering calibrate
    Serial.println(" ");
    //Write Instructions Here
    Serial.println("For Setup Guide Press, 'g'");
    Serial.println("To check PWM signal in Press, 'p'");
    Serial.println("To read Potentiometer Feedback Press, 'f'");
    Serial.println("For Manual Contol Press, 'm'");
    Serial.println("To Home Motor Press, 'h'");
    Serial.println(" ");  
    Serial.println("To exit Calibrate Press, 'x'");
}
//(COMPUTE AND ERROR MODES)
void computeError()
{
  //Set PID Values to Values defined within program
  Feedback = averageFeedbackValue;
  Setpoint = mappedPWM;
  //ERROR CHECKING
  //check whether pwm in has exceeded 10-80% limits
  if(Setpoint > upperLimit) Setpoint = upperLimit;
  if(Setpoint < lowerLimit) Setpoint = lowerLimit;
  //error value
  error = Setpoint - Feedback;
  myPID.Compute();
  //speed limit
  speed = abs(Output); //sets speed always a positive number
  if(speed > 255) speed = 255; //upper limit
}

//(MOTOR CONTROL)
void motorControl()
{
  //Set direction and speed according to PID value
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
  //Brake (0)
  case(B):    Output = 0; //resets ki to reduce creep up
              myPID.SetTunings(0, 0, 0); //resets ki to reduce creep up
              digitalWrite(clockwise, 1);
              digitalWrite(anticlockwise, 1);
              setColour(255, 0, 255); //green
              break;
  //Large Step Clockwise (1)
  case(LCW):  Output = 0; //reset output to allow PID 
              analogWrite(clockwise, 255);
              analogWrite(anticlockwise, 0);
              setColour(0, 255, 255); //red
              break;
  //Large Step Anticlockwise (2)
  case(LACW): Output = 0; //reset output to allow PID 
              analogWrite(clockwise, 0);
              analogWrite(anticlockwise, 255);
              setColour(0, 255, 255); //red
              break;
  //Full rotation Step Clockwise (3)
  case(FCW)://Blipping system
            CWcurrentMillis = millis();

            // If motor is currently off, check if it is time to turn it on
            if (digitalRead(clockwise) == LOW) 
            {
              if (CWcurrentMillis - CWpreviousMillis >= offDelay) 
              {
                CWpreviousMillis = CWcurrentMillis; // Save previous time turned on
                digitalWrite(clockwise, HIGH);      // Turn motor on
                digitalWrite(anticlockwise, LOW);   // Ensure other direction is off
              }
            }
            // If motor is currently on, check if it is time to turn it off
            else 
            {
              if (CWcurrentMillis - CWpreviousMillis >= onDelay) 
              {
                CWpreviousMillis = CWcurrentMillis; // Save previous time turned off
                digitalWrite(clockwise, LOW);       // Turn motor off
                digitalWrite(anticlockwise, LOW);   // Ensure other direction is off
              }
            }
              setColour(0, 255, 255); //red
              break;
  //Full rotation Step Anticlockwise (4)
  case(FACW): //Blipping System Delay
            ACWcurrentMillis = millis();

            // If motor is currently off, check if it is time to turn it on
            if (digitalRead(anticlockwise) == LOW) 
            {
              if (ACWcurrentMillis - ACWpreviousMillis >= offDelay) 
              {
                ACWpreviousMillis = ACWcurrentMillis; // Save previous time turned on
                digitalWrite(clockwise, LOW);      // Turn motor on
                digitalWrite(anticlockwise, HIGH);   // Ensure other direction is off
              }
            }
            // If motor is currently on, check if it is time to turn it off
            else 
            {
              if (ACWcurrentMillis - ACWpreviousMillis >= onDelay) 
              {
                ACWpreviousMillis = ACWcurrentMillis; // Save previous time turned off
                digitalWrite(clockwise, LOW);       // Turn motor off
                digitalWrite(anticlockwise, LOW);   // Ensure other direction is off
              }
            }
                setColour(0, 255, 255); //red
              break;  
  //Delay Before Full rotation Step Clockwise (5)
  case(DELAY_FCW):
              if(millis() - FCWdelayStart >= delayInterval){ //check whether interval time has passed
                motorSwitch = FCW; //set 360 rotation values if time passed
              } else {
                // Keep the motor in brake mode during the delay
                digitalWrite(clockwise, 0);
                digitalWrite(anticlockwise, 0);
              }
              break;
  //Delay Before Full rotation Step Antilockwise (6)
  case(DELAY_FACW):
              if(millis() - FACWdelayStart >= delayInterval){ //check whether interval time has passed
                motorSwitch = FACW; //set 360 rotation values if time passed
              } else {
                // Keep the motor in brake mode during the delay
                digitalWrite(clockwise, 0);
                digitalWrite(anticlockwise, 0);
              }
              break;
    } 

}

//(CALIBRATE MOTOR CONTROL)
void calibrateMotorControl()
{
  //Set direction and speed according to PID value
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
  //Brake (0)
  case(B):    Output = 0; //resets ki to reduce creep up
              myPID.SetTunings(0, 0, 0); //resets ki to reduce creep up
              digitalWrite(clockwise, 1);
              digitalWrite(anticlockwise, 1);
              setColour(255, 0, 255); //green
              break;
  //Large Step Clockwise (1)
  case(LCW):  Output = 0; //resets ki to reduce creep up
              analogWrite(clockwise, 255);
              analogWrite(anticlockwise, 0);
              setColour(0, 255, 255); //red
              break;
  //Large Step Anticlockwise (2)
  case(LACW): Output = 0; //resets ki to reduce creep up
              analogWrite(clockwise, 0);
              analogWrite(anticlockwise, 255);
              setColour(0, 255, 255); //red
              break;
  //Full rotation Step Clockwise (3)
  case(FCW):  myPID.SetTunings(Fkp, Fki, Fkd);
              myPID.SetControllerDirection(DIRECT);
              myPID.Compute();
              analogWrite(clockwise, speed);
              analogWrite(anticlockwise, 0);
              setColour(0, 255, 255); //red
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
  //Delay Before Full rotation Step Antilockwise (6)
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
void changingEdgeISR()
{  
 flag = (digitalRead(pulsePin));
 switch(flag){
  
    case (1) : pulseStartTime = micros(); break;
               
    case (0) : pulseEndTime = micros();
               pulseWidth = pulseEndTime - pulseStartTime; 
               lastPulseTime = pulseEndTime; //allows for timeout
    }
}

//Timeout for PWM
void pwmTimeout()
{
  double currentTime = micros();
  if(currentTime - lastPulseTime > timeoutThreshold) pulseWidth = 0;
}

//AVERAGING both the Pot values and the PWM Values
void SlidingWindow()
{
  //Removing initial array averaging error (blocking very shortly)
  if(arrayComplete == false)
  {
    //run loop 50 times
    while(arrayCounter < 50)
    {
      //Delay function (alter to micros if needed)
      unsigned long currentMicros = micros();//stamp of time since arduino started up

      if(currentMicros >= storedTimeStamp + 10) //alter number for delay time wanted (10 us)
      {  
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

        // Increment the loop counter only after the delay
        arrayCounter++;
      }
    }
    //after loop completion set flag to true
    arrayComplete = true;
  }

  //Normal Sliding Window Function
    //Delay function (alter to micros if needed)
    unsigned long currentMicros = micros();//stamp of time since arduino started up

  if(currentMicros >= storedTimeStamp + 10) //alter number for delay time wanted (10 us)
  {  
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

//Current Averaging (needs to be different)
void currentSlidingWindow()
{
  //Delay function (alter to micros if needed)
  unsigned long currentMicros = micros();//stamp of time since arduino started up

  if(currentMicros >= storedTimeStampmA + 10){ //alter number for delay time wanted (100 us) 
    storedTimeStampmA = currentMicros; //stores current value to be compared with next measured value

 // Remove the oldest value from the sum
    currentSum -= currentValues[i];
    
    // Read the new value and add it to the sum
    currentValues[imA] = analogRead(currentSense);
    currentSum += currentValues[imA];

    // Move to the next position in the array
    imA = (imA + 1) % InumReadings;

    // Calculate the average
    averageFeedbackValue = potSum / numReadings;
    averagePWMValue = pwmSum / numReadings;
    averageCurrentValue = currentSum / numReadings;
    //Mapping PWM values to match analogReadResolution
    mappedPWM = map(averagePWMValue, 0, maxWidth, lowValue, highValue);
    //Mapping current values to match gain and voltage of the system
    currentSenseRead = (map(analogRead(currentSense),0, 4096, 0, 3300))/11; //Divided by 11 as gain = 11 on PCB
  }

}

//LED 
void setColour(int redValue, int greenValue, int blueValue) 
{
  //Set to opposite values
  analogWrite(redLED, redValue);
  analogWrite(greenLED, greenValue);
  analogWrite(blueLED, blueValue);
}

void excelPlotting()
{
  /*
  //Delay function (alter to micros if needed)
  static unsigned long serialStoredTimeStamp = 0;
  unsigned long currentTime = millis(); //stamp of time since arduino started up
  if(currentTime >= serialStoredTimeStamp + 100){ //alter number for delay time wanted
    serialStoredTimeStamp = currentTime; //stores current value to be compared with next measured value
  Serial.print(micros() / 1e6);
  Serial.print(":");
  */
  Serial.print(Setpoint);
  Serial.print(":");
  Serial.print(Feedback);
  Serial.print(":");
  Serial.print(speed); 
  Serial.print(":");
  Serial.print(currentSenseRead);
  Serial.print(":");
  Serial.print(i);
  Serial.print(":");
  Serial.print(motorSwitch);
  Serial.print(":");
  Serial.print(digitalRead(anticlockwise));
  Serial.print(":");
  Serial.println(digitalRead(clockwise));
  //}
}


