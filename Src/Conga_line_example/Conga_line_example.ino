
#include <stdint.h>

#define FIRMWARE_VERSION "v1.0.0"

#define UART_BAUDRATE 115200

#define LOCK_INDICATOR_PIN       7 // Mapped to ATMega328P PD4 (Pin 6) on Arduino Uno
#define MOTOR_L1_PIN             5 // Mapped to ATMega328P PD5 (Pin 11) on Arduino Uno
#define MOTOR_L2_PIN             6 // Mapped to ATMega328P PD6 (Pin 12) on Arduino Uno
#define TRANSMIT_ID_CONTROL_PIN  A0 // Mapped to ATMega328P PD7 (Pin 13) on Arduino Uno
#define MOTOR_R1_PIN             9 // Mapped to ATMega328P PB1 (Pin 15) on Arduino Uno
#define MOTOR_R2_PIN            10 // Mapped to ATMega328P PB2 (Pin 16) on Arduino Uno
#define BUGGY_ID_PIN_0          16 // Mapped to ATMega328P PC2 (Pin 25) on Arduino Uno
#define BUGGY_ID_PIN_1          17 // Mapped to ATMega328P PC3 (Pin 26) on Arduino Uno
#define BUGGY_ID_PIN_2          18 // Mapped to ATMega328P PC4 (Pin 27) on Arduino Uno
#define BUGGY_ID_PIN_3          19 // Mapped to ATMega328P PC5 (Pin 28) on Arduino Uno
#define LDR_L_PIN               A1 // Mapped to ATMega328P PCO (Pin 23) on Arduino Uno
#define LDR_R_PIN               A2 // Mapped to ATMega328P PC1 (Pin 24) on Arduino Uno

#define ADC_RESOLUTION         10                    // 10-Bit ADC
#define ADC_OUTPUT_RANGE       (1 << ADC_RESOLUTION) // (2^10 = 1024 = 10000000000b)
#define ADC_FULL_SCALE_VOLTAGE 5.0                   // Full-Scale Voltage

#define BUGGY_ID_LEADER_MODE 0

#define LDR_LO_LEVEL_THRESHOLD    100
#define LDR_HI_LEVEL_THRESHOLD    800
#define LDR_STOP_LEVEL_THRESHOLD 1000
#define LDR_PAIR_DIFF_THRESHOLD   100

#define BUGGY_ID_PULSE_PERIOD_MS            2000
#define BUGGY_ID_PULSE_ON_TIME_STEP_SIZE_MS 50

#define BUGGY_ID_PULSE_DETECT_PERIOD_MARGIN_MS    20
#define BUGGY_ID_PULSE_DETECT_ON_TIME_MARGIN_MS   10
#define BUGGY_ID_PULSE_DETECT_LOCK_THRESHOLD      3
#define BUGGY_ID_PULSE_DETECT_DELAY_MS            (BUGGY_ID_PULSE_PERIOD_MS - BUGGY_ID_PULSE_DETECT_PERIOD_MARGIN_MS)
#define BUGGY_ID_PULSE_DETECT_NO_PULSE_TIMEOUT_MS (BUGGY_ID_PULSE_PERIOD_MS + BUGGY_ID_PULSE_DETECT_PERIOD_MARGIN_MS)

#define MOTOR_L_FORWARD_PWM_VALUE     150
#define MOTOR_R_FORWARD_PWM_VALUE     150
#define MOTOR_L_REVERSE_PWM_VALUE    -150
#define MOTOR_R_REVERSE_PWM_VALUE    -150
#define MOTOR_L_LEFT_TURN_PWM_VALUE   100
#define MOTOR_R_LEFT_TURN_PWM_VALUE   200
#define MOTOR_L_RIGHT_TURN_PWM_VALUE  200
#define MOTOR_R_RIGHT_TURN_PWM_VALUE  100

#define LIGHT_TRACKING_LEADER_MODE_UPDATE_PERIOD_MS   100
#define LIGHT_TRACKING_FOLLOWER_MODE_UPDATE_PERIOD_MS 100

#define LIGHT_TRACKING_FOLLOWER_MODE_SEEK_PATTERN 0

// Define Global Variables
uint8_t BuggyID;

// Create user-defined Enum (enumerated) data type to define Pulse Detect FSM states
typedef enum
{
  PULSE_DETECT_FSM_INIT,
  PULSE_DETECT_FSM_DETECT_START_CONDITION,
  PULSE_DETECT_FSM_DETECT_PULSE_START,
  PULSE_DETECT_FSM_DETECT_PULSE_END,
  PULSE_DETECT_FSM_DISABLE_PULSE_START_DETECTION
}
PulseDetectFSM_StateTypeDef;

// Function to Measure LDR Circuit Voltage
uint16_t MeasureLDRCircuitVoltage(int PinNumber)
{
  uint16_t AnalogReadResult;
  
  // Measure the voltage from the LDR circuit using ADC
  AnalogReadResult = analogRead(PinNumber);
  
  return AnalogReadResult;
}

// Function to Detect Buggy ID Pulses
uint8_t DetectBuggyIDPulses()
{
  static PulseDetectFSM_StateTypeDef FSMState;
  
  static uint16_t ValidPulseCount = 0;
  
  static uint8_t LockState = 0;
  
  static uint32_t ValidPulseWidthNom;
  static uint32_t ValidPulseWidthMin;
  static uint32_t ValidPulseWidthMax;
  
  static uint32_t PulseStartTimestamp;
  static uint32_t PulseStopTimestamp;
  
  uint32_t CurrentTimestamp;
  
  uint16_t LeftLDRValue;
  uint16_t RightLDRValue;
  
  uint32_t MeasuredPulseWidth;
  
  switch (FSMState)
  {
    case PULSE_DETECT_FSM_INIT:
      // Reset Pulse Counter
      ValidPulseCount = 0;
      
      // Reset Lock State flag
      LockState = 0;
      
      // Turn off Lock Indicator LED
      digitalWrite(LOCK_INDICATOR_PIN, LOW);
      
      // Calculate the Minimum and Maximum Valid Pulse Widths
      ValidPulseWidthNom = (BuggyID * BUGGY_ID_PULSE_ON_TIME_STEP_SIZE_MS);
      ValidPulseWidthMin = ValidPulseWidthNom - BUGGY_ID_PULSE_DETECT_ON_TIME_MARGIN_MS;
      ValidPulseWidthMax = ValidPulseWidthNom + BUGGY_ID_PULSE_DETECT_ON_TIME_MARGIN_MS;
      
      // Clear Persistent Timestamps
      PulseStartTimestamp = 0;
      PulseStopTimestamp  = 0;
      
      // Specify next FSM state
      FSMState = PULSE_DETECT_FSM_DETECT_START_CONDITION;
      break;
      
    case PULSE_DETECT_FSM_DETECT_START_CONDITION:
      // Measure LDR Circuit Voltages using ADC
      LeftLDRValue  = MeasureLDRCircuitVoltage(LDR_L_PIN);
      RightLDRValue = MeasureLDRCircuitVoltage(LDR_R_PIN);
      
      if ((LeftLDRValue < LDR_LO_LEVEL_THRESHOLD) &&
          (RightLDRValue < LDR_LO_LEVEL_THRESHOLD))
      {
        // Specify next FSM state
        FSMState = PULSE_DETECT_FSM_DETECT_PULSE_START;
      }
      break;
      
    case PULSE_DETECT_FSM_DETECT_PULSE_START:
      // Measure LDR Circuit Voltages using ADC
      LeftLDRValue  = MeasureLDRCircuitVoltage(LDR_L_PIN);
      RightLDRValue = MeasureLDRCircuitVoltage(LDR_R_PIN);
      
      // Record the time (in milliseconds) since power-up or reset
      CurrentTimestamp = millis();
      
      if ((LeftLDRValue > LDR_HI_LEVEL_THRESHOLD) ||
          (RightLDRValue > LDR_HI_LEVEL_THRESHOLD))
      {
        // Record Pulse Start Detection Time (in milliseconds since power-up or reset)
        PulseStartTimestamp = millis();
        
        // Specify next FSM state
        FSMState = PULSE_DETECT_FSM_DETECT_PULSE_END;
      }
      else if ((ValidPulseCount > 0) &&
               ((CurrentTimestamp - PulseStartTimestamp) > BUGGY_ID_PULSE_DETECT_NO_PULSE_TIMEOUT_MS))
      {
        // Specify next FSM state
        FSMState = PULSE_DETECT_FSM_INIT;
      }
      break;
      
    case PULSE_DETECT_FSM_DETECT_PULSE_END:
      // Measure LDR Circuit Voltages using ADC
      LeftLDRValue  = MeasureLDRCircuitVoltage(LDR_L_PIN);
      RightLDRValue = MeasureLDRCircuitVoltage(LDR_R_PIN);
      
      if ((LeftLDRValue < LDR_LO_LEVEL_THRESHOLD) &&
          (RightLDRValue < LDR_LO_LEVEL_THRESHOLD))
      {
        // Record Pulse Stop Detection Time (in milliseconds since power-up or reset)
        PulseStopTimestamp = millis();
        
        // Calculate the Width of the Detected Pulse (in milliseconds)
        MeasuredPulseWidth = (PulseStopTimestamp - PulseStartTimestamp);
        
        if ((MeasuredPulseWidth >= ValidPulseWidthMin) && 
            (MeasuredPulseWidth <= ValidPulseWidthMax))
        {
          // Increment Pulse Counter
          ValidPulseCount++;
          
          // If the required number of consecutive valid pulses have been detected, indicate lock
          if (ValidPulseCount == BUGGY_ID_PULSE_DETECT_LOCK_THRESHOLD)
          {
            // Set Lock State flag
            LockState = 1;
            
            // Turn on Lock Indicator LED
            digitalWrite(LOCK_INDICATOR_PIN, HIGH);
          }
          
          // Specify next FSM state
          FSMState = PULSE_DETECT_FSM_DISABLE_PULSE_START_DETECTION;
        }
        else
        {
          // Specify next FSM state
          FSMState = PULSE_DETECT_FSM_INIT;
        }
      }
      else
      {
        // Record the time (in milliseconds) since power-up or reset
        CurrentTimestamp = millis();
        
        if ((CurrentTimestamp - PulseStartTimestamp) > ValidPulseWidthMax)
        {
          // Specify next FSM state
          FSMState = PULSE_DETECT_FSM_INIT;
        }
      }
      break;
      
    case PULSE_DETECT_FSM_DISABLE_PULSE_START_DETECTION:
      // Record the time (in milliseconds) since power-up or reset
      CurrentTimestamp = millis();
      
      if ((CurrentTimestamp - PulseStartTimestamp) > BUGGY_ID_PULSE_DETECT_DELAY_MS)
      {
        // Specify next FSM state
        FSMState = PULSE_DETECT_FSM_DETECT_PULSE_START;
      }
      break;
      
    default:
      // Specify next FSM state
      FSMState = PULSE_DETECT_FSM_INIT;
      break;
  }
  
  return LockState;
}

// Function to Transmit Buggy ID Pulses
void TransmitBuggyIDPulses()
{
  static uint32_t PreviousTimestamp = millis();
  static uint8_t  LedEnablePinState = LOW;
  
  const uint32_t LedOnDurationMSec  = ((BuggyID + 1) * BUGGY_ID_PULSE_ON_TIME_STEP_SIZE_MS);
  const uint32_t LedOffDurationMSec = (BUGGY_ID_PULSE_PERIOD_MS - LedOnDurationMSec);
  
  uint32_t CurrentTimestamp;
  
  // Record the time (in milliseconds) since power-up or reset
  CurrentTimestamp = millis();
  
  if (LedEnablePinState == LOW)
  {
    if ((CurrentTimestamp - PreviousTimestamp) >= LedOffDurationMSec)
    {
      digitalWrite(TRANSMIT_ID_CONTROL_PIN, HIGH);
      
      LedEnablePinState = HIGH;
      
      PreviousTimestamp = CurrentTimestamp;
    }
  }
  else
  {
    if ((CurrentTimestamp - PreviousTimestamp) >= LedOnDurationMSec)
    {
      digitalWrite(TRANSMIT_ID_CONTROL_PIN, LOW);
      
      LedEnablePinState = LOW;
      
      PreviousTimestamp = CurrentTimestamp;
    }
  }
}

// Function to Read Buggy ID
uint8_t ReadBuggyID()
{
  uint8_t BuggyID;
  
  BuggyID = ((digitalRead(BUGGY_ID_PIN_3) << 3) |
             (digitalRead(BUGGY_ID_PIN_2) << 2) |
             (digitalRead(BUGGY_ID_PIN_1) << 1) |
             (digitalRead(BUGGY_ID_PIN_0) << 0));
  
  return BuggyID;
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
void UpdateMotorSpeed(int16_t LeftMotorPWMValue, int16_t RightMotorPWMValue)
{
  // Set Left Motor Control Parameters
  SetMotorControlParameters(LeftMotorPWMValue, MOTOR_L1_PIN, MOTOR_L2_PIN);
  
  // Set Right Motor Control Parameters
  SetMotorControlParameters(RightMotorPWMValue, MOTOR_R1_PIN, MOTOR_R2_PIN);
}

// Function to Track Light in Leader-Mode
void TrackLightLeaderMode()
{
  const uint32_t LeaderModeLightTrackingUpdatePeriod = LIGHT_TRACKING_LEADER_MODE_UPDATE_PERIOD_MS;
  
  static uint32_t PreviousTimestamp = millis();
  
  uint32_t CurrentTimestamp;
  
  // Record the time (in milliseconds) since power-up or reset
  CurrentTimestamp = millis();
  
  if ((CurrentTimestamp - PreviousTimestamp) > LeaderModeLightTrackingUpdatePeriod)
  {
    uint16_t LeftLDRValue;
    uint16_t RightLDRValue;
    
    uint16_t LDRDiffMagnitude;
    
    int16_t LeftMotorPWMValue;
    int16_t RightMotorPWMValue;
    
    // Measure LDR Circuit Voltages using ADC
    LeftLDRValue  = MeasureLDRCircuitVoltage(LDR_L_PIN);
    RightLDRValue = MeasureLDRCircuitVoltage(LDR_R_PIN);
    
    // Calculate the absolute difference between the LDR Circuit Voltages
    LDRDiffMagnitude = abs(LeftLDRValue - RightLDRValue);
    
    if (LDRDiffMagnitude > LDR_PAIR_DIFF_THRESHOLD)
    {
      if (LeftLDRValue > RightLDRValue)
      {
        LeftMotorPWMValue  = MOTOR_L_LEFT_TURN_PWM_VALUE;
        RightMotorPWMValue = MOTOR_R_LEFT_TURN_PWM_VALUE;
      }
      else
      {
        LeftMotorPWMValue  = MOTOR_L_RIGHT_TURN_PWM_VALUE;
        RightMotorPWMValue = MOTOR_R_RIGHT_TURN_PWM_VALUE;
      }
    }
    else
    {
      if (min(LeftLDRValue, RightLDRValue) > LDR_STOP_LEVEL_THRESHOLD)
      {
        LeftMotorPWMValue  = 0;
        RightMotorPWMValue = 0;
      }
      else
      {
        LeftMotorPWMValue  = MOTOR_L_FORWARD_PWM_VALUE;
        RightMotorPWMValue = MOTOR_R_FORWARD_PWM_VALUE;
      }
    }
    
    // Update the PWM values applied to the left and right motors
    UpdateMotorSpeed(LeftMotorPWMValue, RightMotorPWMValue);
    
    // Store the Current Timestamp for use in the next iteration
    PreviousTimestamp = CurrentTimestamp;
  }
}

// Function to Track Light in Follower-Mode
void TrackLightFollowerMode(uint8_t LockState)
{
  const uint32_t FollowerModeLightTrackingUpdatePeriod = LIGHT_TRACKING_FOLLOWER_MODE_UPDATE_PERIOD_MS;
  
  static uint32_t PreviousTimestamp = millis();
  
  uint32_t CurrentTimestamp;
  
  // Record the time (in milliseconds) since power-up or reset
  CurrentTimestamp = millis();
  
  if ((CurrentTimestamp - PreviousTimestamp) > FollowerModeLightTrackingUpdatePeriod)
  {
    uint16_t LeftLDRValue;
    uint16_t RightLDRValue;
    
    uint16_t LDRDiffMagnitude;
    
    int16_t LeftMotorPWMValue;
    int16_t RightMotorPWMValue;
    
    // If in lock, attempt to maintain lock by following the light source
    if (LockState == 1)
    {
      // Measure LDR Circuit Voltages using ADC
      LeftLDRValue  = MeasureLDRCircuitVoltage(LDR_L_PIN);
      RightLDRValue = MeasureLDRCircuitVoltage(LDR_R_PIN);
      
      // Calculate the absolute difference between the LDR Circuit Voltages
      LDRDiffMagnitude = abs(LeftLDRValue - RightLDRValue);
    
      if (LDRDiffMagnitude > LDR_PAIR_DIFF_THRESHOLD)
      {
        if (LeftLDRValue > RightLDRValue)
        {
          LeftMotorPWMValue  = MOTOR_L_LEFT_TURN_PWM_VALUE;
          RightMotorPWMValue = MOTOR_R_LEFT_TURN_PWM_VALUE;
        }
        else
        {
          LeftMotorPWMValue  = MOTOR_L_RIGHT_TURN_PWM_VALUE;
          RightMotorPWMValue = MOTOR_R_RIGHT_TURN_PWM_VALUE;
        }
      }
      else
      {
        if (min(LeftLDRValue, RightLDRValue) > LDR_STOP_LEVEL_THRESHOLD)
        {
          LeftMotorPWMValue  = 0;
          RightMotorPWMValue = 0;
        }
        else
        {
          LeftMotorPWMValue  = MOTOR_L_FORWARD_PWM_VALUE;
          RightMotorPWMValue = MOTOR_R_FORWARD_PWM_VALUE;
        }
      }
    }
    else
    {
      #if (LIGHT_TRACKING_FOLLOWER_MODE_SEEK_PATTERN == 0)
        // Wait for light source to approach
        LeftMotorPWMValue  = 0;
        RightMotorPWMValue = 0;
      #elif (LIGHT_TRACKING_FOLLOWER_MODE_SEEK_PATTERN == 1)
        // Turn left or right based on whether the Buggy ID is odd or even
        uint8_t BuggyIDIsOdd = (BuggyID % 2);
        
        if (BuggyIDIsOdd)
        {
          LeftMotorPWMValue  = MOTOR_L_LEFT_TURN_PWM_VALUE;
          RightMotorPWMValue = MOTOR_R_LEFT_TURN_PWM_VALUE;
        }
        else
        {
          LeftMotorPWMValue  = MOTOR_L_RIGHT_TURN_PWM_VALUE;
          RightMotorPWMValue = MOTOR_R_RIGHT_TURN_PWM_VALUE;
        }
      #elif (LIGHT_TRACKING_FOLLOWER_MODE_SEEK_PATTERN == 2)
        // Turn left, right or go straight based on the current timestamp
        uint8_t Direction = (CurrentTimestamp % 3);
        
        if (Direction == 0)
        {
          LeftMotorPWMValue  = MOTOR_L_LEFT_TURN_PWM_VALUE;
          RightMotorPWMValue = MOTOR_R_LEFT_TURN_PWM_VALUE;
        }
        else if (Direction == 1)
        {
          LeftMotorPWMValue  = MOTOR_L_RIGHT_TURN_PWM_VALUE;
          RightMotorPWMValue = MOTOR_R_RIGHT_TURN_PWM_VALUE;
        }
        else
        {
          LeftMotorPWMValue  = MOTOR_L_FORWARD_PWM_VALUE;
          RightMotorPWMValue = MOTOR_R_FORWARD_PWM_VALUE;
        }
      #else
        // Wait for light source to approach
        LeftMotorPWMValue  = 0;
        RightMotorPWMValue = 0;
      #endif
    }
    
    // Update the PWM values applied to the left and right motors
    UpdateMotorSpeed(LeftMotorPWMValue, RightMotorPWMValue);
    
    // Store the Current Timestamp for use in the next iteration
    PreviousTimestamp = CurrentTimestamp;
  }
}

// Setup function runs once when board is powered up or reset
void setup()
{
  // Initialise UART
  Serial.begin(UART_BAUDRATE);
  
  // Configure GPIO pins
  pinMode(BUGGY_ID_PIN_3, INPUT);
  pinMode(BUGGY_ID_PIN_2, INPUT);
  pinMode(BUGGY_ID_PIN_1, INPUT);
  pinMode(BUGGY_ID_PIN_0, INPUT);
  pinMode(MOTOR_L1_PIN, OUTPUT);
  pinMode(MOTOR_L2_PIN, OUTPUT);
  pinMode(MOTOR_R1_PIN, OUTPUT);
  pinMode(MOTOR_R2_PIN, OUTPUT);
  pinMode(LOCK_INDICATOR_PIN, OUTPUT);
  pinMode(TRANSMIT_ID_CONTROL_PIN, OUTPUT);
  
  // Set initial value for digital output pins
  digitalWrite(MOTOR_L1_PIN, LOW);
  digitalWrite(MOTOR_L2_PIN, LOW);
  digitalWrite(MOTOR_R1_PIN, LOW);
  digitalWrite(MOTOR_R2_PIN, LOW);
  digitalWrite(LOCK_INDICATOR_PIN, LOW);
  digitalWrite(TRANSMIT_ID_CONTROL_PIN, LOW);
  
  // Read Buggy ID
  BuggyID = ReadBuggyID();
  
  // Output message to console
  Serial.println("B37VB Conga Line Example");
  
  Serial.print("Version: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.println();
  
  Serial.print("Buggy ID: ");
  Serial.println(BuggyID);
  Serial.println();
}

// Loop function runs over and over again forever
void loop()
{
  // Transmit Buggy ID Pulses
  TransmitBuggyIDPulses();
  
  // Determine execution mode from Buggy ID
  if (BuggyID != BUGGY_ID_LEADER_MODE)
  {
    uint8_t LockState;
    
    // Detect Buggy ID Pulses
    LockState = DetectBuggyIDPulses();
    
    // Track Light in Follower-Mode
    TrackLightFollowerMode(LockState);
  }
  else
  {
    // Track Light in Leader-Mode
    TrackLightLeaderMode();
  }
}
