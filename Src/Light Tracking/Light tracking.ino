#include <stdint.h>
#define FirmwareVersion "v1.0.4"

#define UARTBaudRate 115200 // Normal 9600 but changed to 115200

#define MotorL1Pin 5 // Pins on Arduino UNO that connect to motors
#define MotorL2Pin 6
#define MotorR1Pin 9
#define MotorR2Pin 10


#define LDRLeftPin A2 // Left LDR. Mapped to ATMega328P PCO (Pin 23) on Arduino Uno
#define LDRRightPin A1 // Right LDR. Mapped to ATMega328P PC1 (Pin 24) on Arduino Uno
#define LDRHardRightPin A3 // Hard Right LDR 
#define LDRHardLeftPin A4 // Hard Left LDR

#define ADCResolution        10                    // 10-Bit ADC
#define ADCOutputRange       (1 << ADCResolution) // (2^10 = 1024)
#define ADCFullScaleVoltage 5.0                   // Full-Scale Voltage

#define MotorDiff 1.27 // Percentage difference of power between the motors (this needs to be changed if a new robot is used) 

#define LDRDiffThreshold 200 //ADC difference between LDRs
#define LDRLockThreshold 550 // Using 2.2k Ohms 
#define LowThreshholdLDRValue 100 //Ambient Light (GRID=400?)

#define MotorLeftForwardPWMValue 100*MotorDiff //PWM value for the left motor when driving forward
#define MotorRightForwardPWMValue 100          //PWM value for the right motor when driving forward

#define MotorLeftReversePWMValue -150*MotorDiff //PWM value for the left motor when driving in reverse
#define MotorRightReversePWMValue -150          //PWM value for the right motor when driving in reverse

#define MotorLLeftTurnPWMValue 75*MotorDiff  //PWM value for the left motor when turning left
#define MotorRLeftTurnPWMValue 175           //PWM value for the right motor when turning left

#define MotorLRightTurnPWMValue 175*MotorDiff //PWM value for the left motor when turning right
#define MotorRRightTurnPWMValue 75            //PWM value for the right motor when turning right

#define MotorLHardLeftTurnPWMValue -75*MotorDiff //PWM value for the left motor when making a hard turn left
#define MotorRHardLeftTurnPWMValue 150           //PWM value for the right motor when making a hard turn left

#define MotorLHardRightTurnPWMValue 150*MotorDiff //PWM value for the left motor when making a hard turn right
#define MotorRHardRightTurnPWMValue -75           //PWM value for the right motor when making a hard turn left

#define MotorTurnDurationMS 150 // Duration of the turn in milliseconds

#define PWMValueMax 255 // Maximum PWM value allowed (Not implemented)
#define PWMValueMin -255 // Minimum PWM value allowed (Not implemented)

//#define LeaderModeEnable 0 // Set to 1 for Leader Mode, 0 for Follower Mode

/*#if LeaderModeEnable 
  typedef struct
  {
    int16_t  LeftMotorPWMValue;
    int16_t  RightMotorPWMValue;
    uint32_t DurationMilliseconds;
  }
  RouteLeg_t;
  
  RouteLeg_t LeaderRouteLegs[] = 
    {{MotorLeftForwardPWMValue, MotorRightForwardPWMValue, 5000},
     {0, 0, 5000},
     {MotorLeftReversePWMValue, MotorRightReversePWMValue, 5000},
     {0, 0, 7500},
     {MotorLeftForwardPWMValue, MotorRightForwardPWMValue, 2500},
     {0, 0, 2500},
     {MotorLeftReversePWMValue, MotorRightReversePWMValue, 2500}};
#endif*/

// Function to measure LDR Circuit Voltage
uint16_t MeasureLDRCircuitVoltage(int PinNumber)
{
  uint16_t AnalogReadResult;
  
  // Measure the voltage from the LDR circuit using ADC
  AnalogReadResult = (uint16_t)analogRead(PinNumber);
  
  return AnalogReadResult;
}
// Function to Set Motor Control Parameters
void SetMotorControlParameters(int16_t PWMValue,
                               uint8_t HBridgeControlPinA,
                               uint8_t HBridgeControlPinB)
{
  if (PWMValue >= 0)
  {
    analogWrite(HBridgeControlPinB, PWMValue);
    digitalWrite(HBridgeControlPinA, LOW);
  }
  else
  {
    analogWrite(HBridgeControlPinA, -PWMValue);
    digitalWrite(HBridgeControlPinB, LOW);
  }
}

// Function to Update Motor Speed
void UpdateMotorSpeed(int16_t  LeftMotorPWMValue,
                      int16_t  RightMotorPWMValue,
                      uint32_t DurationMilliseconds)
{
  // Set Left Motor Control Parameters
  SetMotorControlParameters(LeftMotorPWMValue,
                            MotorL1Pin,
                            MotorL2Pin);
  
  // Set Right Motor Control Parameters
  SetMotorControlParameters(RightMotorPWMValue,
                            MotorR1Pin,
                            MotorR2Pin);
  
  // Add blocking delay (in ms)
  delay(DurationMilliseconds);
}


// Setup function runs once when board is powered up or reset
void setup()
{
  // Initialise UART
  Serial.begin(UARTBaudRate);
  
  // Initialise GPIO
  pinMode(MotorL1Pin, OUTPUT);
  pinMode(MotorL2Pin, OUTPUT);
  pinMode(MotorR1Pin, OUTPUT);
  pinMode(MotorR2Pin, OUTPUT);
  
  // Initialise Motor Driver
  digitalWrite(MotorL1Pin, LOW);
  digitalWrite(MotorL2Pin, LOW);
  digitalWrite(MotorR1Pin, LOW);
  digitalWrite(MotorR2Pin, LOW);
  
  // Output message to console
  Serial.println("B37VB Motor Control and LDR tracking Basics Demonstration with conga line");
  Serial.print("Version: ");
  Serial.println(FirmwareVersion);
  
  /*#if LEADER_MODE_ENABLE
    Serial.println("Mode: Leader");
    
    // Start the buggy on the route
    UpdateMotorSpeed(LeaderRouteLegs[0].LeftMotorPWMValue,
                     LeaderRouteLegs[0].RightMotorPWMValue,
                     0);
  #else
    Serial.println("Mode: Follower");
  #endif*/
}

// Loop function runs over and over again forever
void loop()
{
 int LeftLDRValue;
  int RightLDRValue;
  int HardLeftLDRValue;
  int HardRightLDRValue;

  unsigned int CentralLDRDiffMagnitude;
  
  int LeftMotorPWMValue;
  int RightMotorPWMValue;
  int LeftLDRDiffMagnitude;
  int RightLDRDiffMagnitude;
  unsigned long TurnDuration;
  
  LeftLDRValue  = MeasureLDRCircuitVoltage(LDRLeftPin);
  //Serial.println(LeftLDRValue); // To extract value if needed for testing
  RightLDRValue = MeasureLDRCircuitVoltage(LDRRightPin);
  //Serial.println(RightLDRValue); // To extract value
  HardLeftLDRValue  = MeasureLDRCircuitVoltage(LDRHardLeftPin);
  //Serial.println(HardLeftLDRValue); // To extract value
  HardRightLDRValue = MeasureLDRCircuitVoltage(LDRHardRightPin);
  //Serial.println(HardRightLDRValue);

  CentralLDRDiffMagnitude = abs(LeftLDRValue - RightLDRValue);
  //Serial.println(CentralLDRDiffMagnitude);
  RightLDRDiffMagnitude = abs(RightLDRValue - HardRightLDRValue);
  //Serial.println(RightLDRDiffMagnitude);
  LeftLDRDiffMagnitude = abs(LeftLDRValue - HardLeftLDRValue);
  //Serial.println(LeftLDRDiffMagnitude);

  static uint32_t PreviousTimestamp = millis();
  
  uint32_t CurrentTimestamp;
  
  CurrentTimestamp = millis();
  
  /*#if LEADER_MODE_ENABLE
    static uint8_t LegIndex = 0;
    
    const uint8_t TotalRouteLegs = (sizeof(LeaderRouteLegs) / sizeof(LeaderRouteLegs[0]));
    
    if (LegIndex < TotalRouteLegs)
    {
      if ((CurrentTimestamp - PreviousTimestamp) 
            >= LeaderRouteLegs[LegIndex].DurationMilliseconds)
      {
        if (LegIndex == (TotalRouteLegs - 1))
        {
          LegIndex++;
          
          // Ensure that the buggy stops at the end of the route
          UpdateMotorSpeed(0, 0, 0);
        }
        else
        {
          LegIndex++;
          
          UpdateMotorSpeed(LeaderRouteLegs[LegIndex].LeftMotorPWMValue,
                           LeaderRouteLegs[LegIndex].RightMotorPWMValue,
                           0);
          
          PreviousTimestamp = CurrentTimestamp;
        }
      }
    
    else
    {
      // Included for completeness
      // Do nothing after route completed
    }
  #else
    // Follower code goes here
  #endif*/


  if ((CentralLDRDiffMagnitude > LDRDiffThreshold) && 
      max(CentralLDRDiffMagnitude,
      max(RightLDRDiffMagnitude,LeftLDRDiffMagnitude))
      ==CentralLDRDiffMagnitude /*&& RightLDRDiffMagnitude,LeftLDRDiffMagnitude*/ /*&& 
      min(LeftLDRValue,RightLDRValue)> LowThreshholdLDRValue*/)
        
  {
    if (LeftLDRValue > RightLDRValue)
    {
      LeftMotorPWMValue  = MotorLLeftTurnPWMValue;
      RightMotorPWMValue = MotorRLeftTurnPWMValue;
      TurnDuration       = MotorTurnDurationMS;
      Serial.println(1);

    }
    if (LeftLDRValue < RightLDRValue)
    {
      LeftMotorPWMValue  = MotorLRightTurnPWMValue;
      RightMotorPWMValue = MotorRRightTurnPWMValue;
      TurnDuration       = MotorTurnDurationMS; 
      Serial.println(2);

    }
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                    TurnDuration);  
      
  }
  else if (LeftLDRDiffMagnitude > LDRDiffThreshold && 
          max(CentralLDRDiffMagnitude,
          max(RightLDRDiffMagnitude,LeftLDRDiffMagnitude))
          ==LeftLDRDiffMagnitude &&
          HardLeftLDRValue > LeftLDRValue /*&&
          min(HardLeftLDRValue,LeftLDRValue)> LowThreshholdLDRValue*/) 
  { 
    LeftMotorPWMValue  = MotorLHardLeftTurnPWMValue;
    RightMotorPWMValue = MotorRHardLeftTurnPWMValue;
    TurnDuration       = MotorTurnDurationMS;
    Serial.println(3);

       
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                    TurnDuration);
  }
  else if (RightLDRDiffMagnitude > LDRDiffThreshold &&
          max(CentralLDRDiffMagnitude,
          max(RightLDRDiffMagnitude,LeftLDRDiffMagnitude))
          ==RightLDRDiffMagnitude &&
          HardRightLDRValue > RightLDRValue /*&&
          min(HardRightLDRValue,RightLDRValue)> LowThreshholdLDRValue*/)
  {

    LeftMotorPWMValue  = MotorLHardRightTurnPWMValue;
    RightMotorPWMValue = MotorRHardRightTurnPWMValue;
    TurnDuration       = MotorTurnDurationMS;
    Serial.println(4);
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                    TurnDuration);   
  }

  else
  {   
    if (max(LeftLDRValue, RightLDRValue) > LDRLockThreshold /*||
        max(HardLeftLDRValue,LeftLDRValue) > LDRLockThreshold ||
        max(HardRightLDRValue,RightLDRValue) > LDRLockThreshold*/)
        
    {
      LeftMotorPWMValue  = 0;
      RightMotorPWMValue = 0;
      Serial.println(0);

    }
    /*else if (max(max(LeftLDRValue, RightLDRValue), 
              max(HardLeftLDRValue, HardRightLDRValue)) 
              < LowThreshholdLDRValue)
    {
      LeftMotorPWMValue  = 0;
      RightMotorPWMValue = 0;
    }*/
    else
    {  
      LeftMotorPWMValue  = MotorLeftForwardPWMValue;
      RightMotorPWMValue = MotorRightForwardPWMValue;
  
    }
    
    UpdateMotorSpeed(LeftMotorPWMValue,
                     RightMotorPWMValue,
                     0);
  }

  
  
}
