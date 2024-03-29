#include <stdint.h>
#define FirmwareVersion "v1.0.3"

#define UARTBaudRate 115200 // Changed to higher 

#define MotorL1Pin 5
#define MotorL2Pin 6
#define MotorR1Pin 9
#define MotorR2Pin 10


#define LDR_L_PIN A2 // Mapped to ATMega328P PCO (Pin 23) on Arduino Uno
#define LDR_R_PIN A1 // Mapped to ATMega328P PC1 (Pin 24) on Arduino Uno
#define LDR_RR_PIN A3
#define LDR_LL_PIN A4

#define ADCResolution         10                    // 10-Bit ADC
#define ADCOutputRange       (1 << ADCResolution) // (2^10 = 1024)
#define ADC_FULL_SCALE_VOLTAGE 5.0                   // Full-Scale Voltage

#define MotorDiff 1.27 // Percentage difference of power between the motors (this needs to be changed if a new robot is used) 

#define LDR_DIFF_THRESHOLD 150
#define LDR_LOCK_THRESHOLD 550 // Using 2.2k Ohms 
#define LowThreshholdLDRValue 100 //Ambient Light (GRID=550)

#define MOTOR_L_FORWARD_PWM_VALUE 100*MotorDiff
#define MOTOR_R_FORWARD_PWM_VALUE 100

#define MOTOR_L_REVERSE_PWM_VALUE -150*MotorDiff
#define MOTOR_R_REVERSE_PWM_VALUE -150

#define MOTOR_L_LEFT_TURN_PWM_VALUE 75*MotorDiff
#define MOTOR_R_LEFT_TURN_PWM_VALUE 175

#define MOTOR_L_RIGHT_TURN_PWM_VALUE 175*MotorDiff
#define MOTOR_R_RIGHT_TURN_PWM_VALUE 75

#define MOTOR_L_HARD_LEFT_TURN_PWM_VALUE -75*MotorDiff
#define MOTOR_R_HARD_LEFT_TURN_PWM_VALUE 150

#define MOTOR_L_HARD_RIGHT_TURN_PWM_VALUE 150*MotorDiff
#define MOTOR_R_HARD_RIGHT_TURN_PWM_VALUE -75

#define MOTOR_TURN_DURATION_MS 150 

#define PWM_VALUE_MAX 255
#define PWM_VALUE_MIN -255

#define LEADER_MODE_ENABLE 0 // Set to 1 for Leader Mode, 0 for Follower Mode

#if LEADER_MODE_ENABLE
  typedef struct
  {
    int16_t  LeftMotorPWMValue;
    int16_t  RightMotorPWMValue;
    uint32_t DurationMilliseconds;
  }
  RouteLeg_t;
  
  RouteLeg_t LeaderRouteLegs[] = 
    {{MOTOR_L_FORWARD_PWM_VALUE, MOTOR_R_FORWARD_PWM_VALUE, 5000},
     {0, 0, 5000},
     {MOTOR_L_REVERSE_PWM_VALUE, MOTOR_R_REVERSE_PWM_VALUE, 5000},
     {0, 0, 7500},
     {MOTOR_L_FORWARD_PWM_VALUE, MOTOR_R_FORWARD_PWM_VALUE, 2500},
     {0, 0, 2500},
     {MOTOR_L_REVERSE_PWM_VALUE, MOTOR_R_REVERSE_PWM_VALUE, 2500}};
#endif

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
  
  #if LEADER_MODE_ENABLE
    Serial.println("Mode: Leader");
    
    // Start the buggy on the route
    UpdateMotorSpeed(LeaderRouteLegs[0].LeftMotorPWMValue,
                     LeaderRouteLegs[0].RightMotorPWMValue,
                     0);
  #else
    Serial.println("Mode: Follower");
  #endif
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
  
  LeftLDRValue  = MeasureLDRCircuitVoltage(LDR_L_PIN);
  //Serial.println(LeftLDRValue); // To extract 
  RightLDRValue = MeasureLDRCircuitVoltage(LDR_R_PIN);
  //Serial.println(RightLDRValue);
  HardLeftLDRValue  = MeasureLDRCircuitVoltage(LDR_LL_PIN);
  //Serial.println(HardLeftLDRValue);
  HardRightLDRValue = MeasureLDRCircuitVoltage(LDR_RR_PIN);
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
  
  #if LEADER_MODE_ENABLE
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
  #endif


  if ((CentralLDRDiffMagnitude > LDR_DIFF_THRESHOLD) && 
      max(CentralLDRDiffMagnitude,
      max(RightLDRDiffMagnitude,LeftLDRDiffMagnitude))
      ==CentralLDRDiffMagnitude /*&& RightLDRDiffMagnitude,LeftLDRDiffMagnitude*/ /*&& 
      min(LeftLDRValue,RightLDRValue)> LowThreshholdLDRValue*/)
        
  {
    if (LeftLDRValue > RightLDRValue)
    {
      LeftMotorPWMValue  = MOTOR_L_LEFT_TURN_PWM_VALUE;
      RightMotorPWMValue = MOTOR_R_LEFT_TURN_PWM_VALUE;
      TurnDuration       = MOTOR_TURN_DURATION_MS;
      Serial.println(1);

    }
    if (LeftLDRValue < RightLDRValue)
    {
      LeftMotorPWMValue  = MOTOR_L_RIGHT_TURN_PWM_VALUE;
      RightMotorPWMValue = MOTOR_R_RIGHT_TURN_PWM_VALUE;
      TurnDuration       = MOTOR_TURN_DURATION_MS; 
      Serial.println(2);

    }
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                    TurnDuration);  
      
  }
  else if (LeftLDRDiffMagnitude > LDR_DIFF_THRESHOLD && 
          max(CentralLDRDiffMagnitude,
          max(RightLDRDiffMagnitude,LeftLDRDiffMagnitude))
          ==LeftLDRDiffMagnitude &&
          HardLeftLDRValue > LeftLDRValue /*&&
          min(HardLeftLDRValue,LeftLDRValue)> LowThreshholdLDRValue*/) 
  { 
    LeftMotorPWMValue  = MOTOR_L_HARD_LEFT_TURN_PWM_VALUE;
    RightMotorPWMValue = MOTOR_R_HARD_LEFT_TURN_PWM_VALUE;
    TurnDuration       = MOTOR_TURN_DURATION_MS;
    Serial.println(3);

       
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                    TurnDuration);
  }
  else if (RightLDRDiffMagnitude > LDR_DIFF_THRESHOLD &&
          max(CentralLDRDiffMagnitude,
          max(RightLDRDiffMagnitude,LeftLDRDiffMagnitude))
          ==RightLDRDiffMagnitude &&
          HardRightLDRValue > RightLDRValue /*&&
          min(HardRightLDRValue,RightLDRValue)> LowThreshholdLDRValue*/)
  {

    LeftMotorPWMValue  = MOTOR_L_HARD_RIGHT_TURN_PWM_VALUE;
    RightMotorPWMValue = MOTOR_R_HARD_RIGHT_TURN_PWM_VALUE;
    TurnDuration       = MOTOR_TURN_DURATION_MS;
    Serial.println(4);
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                    TurnDuration);   
  }

  else
  {   
    if (max(LeftLDRValue, RightLDRValue) > LDR_LOCK_THRESHOLD /*||
        max(HardLeftLDRValue,LeftLDRValue) > LDR_LOCK_THRESHOLD ||
        max(HardRightLDRValue,RightLDRValue) > LDR_LOCK_THRESHOLD*/)
        
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
      LeftMotorPWMValue  = MOTOR_L_FORWARD_PWM_VALUE;
      RightMotorPWMValue = MOTOR_R_FORWARD_PWM_VALUE;
  
    }
    
    UpdateMotorSpeed(LeftMotorPWMValue,
                     RightMotorPWMValue,
                     0);
  }

  
  
}
