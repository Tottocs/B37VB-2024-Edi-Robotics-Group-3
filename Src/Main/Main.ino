#include <stdint.h>
#define FIRMWARE_VERSION "v1.0.5"

#define UART_BAUDRATE 115200 // Normal 9600 but changed to 115200

#define MOTOR_L1_PIN 5 // Pins on Arduino UNO that connect to motors
#define MOTOR_L2_PIN 6
#define MOTOR_R1_PIN 9
#define MOTOR_R2_PIN 10


#define LDR_L_PIN A2 // Left LDR. Mapped to ATMega328P PCO (Pin 23) on Arduino Uno
#define LDR_R_PIN A1 // Right LDR. Mapped to ATMega328P PC1 (Pin 24) on Arduino Uno
#define LDR_HARD_R_Pin A3 // Hard Right LDR 
#define LDR_HARD_L_Pin A4 // Hard Left LDR

#define ADC_RESOLUTION        10                    // 10-Bit ADC
#define ADC_OUTPUT_RANGE       (1 << ADCResolution) // (2^10 = 1024)
#define ADC_FULLSCALE_VOLTAGE 5.0                   // Full-Scale Voltage

#define MOTOR_DIFF 1.27 // Percentage difference of power between the motors (this needs to be changed if a new robot is used) 

#define LDR_DIFF_THRESHOLD 150 //ADC difference between LDRs
#define LDR_LOCK_THRESHOLD 750 // Using 4.7k Ohms 
#define LOW_THRESHOLD_LDR_VALUE 200 //Ambient Light (GRID=400?)

#define MOTOR_L_FORWARD_PWM_VALUE 100*MOTOR_DIFF //PWM value for the left motor when driving forward
#define MOTOR_R_FORWARD_PWM_VALUE 100          //PWM value for the right motor when driving forward

#define MOTOR_L_REVERSE_PWM_VALUE -150*MOTOR_DIFF //PWM value for the left motor when driving in reverse
#define MOTOR_R_REVERSE_PWM_VALUE -150          //PWM value for the right motor when driving in reverse

#define MOTOR_L_LEFT_TURN_PWM_VALUE 75*MOTOR_DIFF  //PWM value for the left motor when turning left
#define MOTOR_R_LEFT_TURN_PWM_VALUE 175           //PWM value for the right motor when turning left

#define MOTOR_L_RIGHT_TURN_PWM_VALUE 175*MOTOR_DIFF //PWM value for the left motor when turning right
#define MOTOR_R_RIGHT_TURN_PWM_VALUE 75            //PWM value for the right motor when turning right

#define MOTOR_L_HARD_LEFT_TURN_PWM_VALUE -75*MOTOR_DIFF //PWM value for the left motor when making a hard turn left
#define MOTOR_R_HARD_LEFT_TURN_PWM_VALUE 150           //PWM value for the right motor when making a hard turn left

#define MOTOR_L_HARD_RIGHT_TURN_PWM_VALUE 150*MOTOR_DIFF //PWM value for the left motor when making a hard turn right
#define MOTOR_R_HARD_RIGHT_TURN_PWM_VALUE -75           //PWM value for the right motor when making a hard turn left

#define MOTOR_TURN_DURATION_MS 150 // Duration of the turn in milliseconds

#define PWM_VALUE_MAX 255 // Maximum PWM value allowed (Not implemented)
#define PWM_VALUE_MIN -255 // Minimum PWM value allowed (Not implemented)


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
                            MOTOR_L1_PIN,
                            MOTOR_L2_PIN);
  
  // Set Right Motor Control Parameters
  SetMotorControlParameters(RightMotorPWMValue,
                            MOTOR_R1_PIN,
                            MOTOR_R2_PIN);
  
  // Add blocking delay (in ms)
  delay(MOTOR_TURN_DURATION_MS);
}


// Setup function runs once when board is powered up or reset
void setup()
{
  // Initialise UART
  Serial.begin(UART_BAUDRATE);
  
  // Initialise GPIO
  pinMode(MOTOR_L1_PIN, OUTPUT);
  pinMode(MOTOR_L2_PIN, OUTPUT);
  pinMode(MOTOR_R1_PIN, OUTPUT);
  pinMode(MOTOR_R2_PIN, OUTPUT);
  
  // Initialise Motor Driver
  digitalWrite(MOTOR_L1_PIN, LOW);
  digitalWrite(MOTOR_L2_PIN, LOW);
  digitalWrite(MOTOR_R1_PIN, LOW);
  digitalWrite(MOTOR_R2_PIN, LOW);
  
  // Output message to console
  Serial.println("B37VB LDR tracking Basics Demonstration");
  Serial.print("Version: ");
  Serial.println(FIRMWARE_VERSION);
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
  //Serial.println(LeftLDRValue); // To extract value if needed for testing
  RightLDRValue = MeasureLDRCircuitVoltage(LDR_R_PIN);
  //Serial.println(RightLDRValue); // To extract value
  HardLeftLDRValue  = MeasureLDRCircuitVoltage(LDR_HARD_L_Pin);
  //Serial.println(HardLeftLDRValue); // To extract value
  HardRightLDRValue = MeasureLDRCircuitVoltage(LDR_HARD_R_Pin);
  //Serial.println(HardRightLDRValue);

  CentralLDRDiffMagnitude = abs(LeftLDRValue - RightLDRValue);
  //Serial.println(CentralLDRDiffMagnitude);
  RightLDRDiffMagnitude = abs(RightLDRValue - HardRightLDRValue);
  //Serial.println(RightLDRDiffMagnitude);
  LeftLDRDiffMagnitude = abs(LeftLDRValue - HardLeftLDRValue);
  //Serial.println(LeftLDRDiffMagnitude);

  if (min(LeftLDRValue, RightLDRValue) > LDR_LOCK_THRESHOLD /*||
    min(HardLeftLDRValue,LeftLDRValue) > LDR_LOCK_THRESHOLD ||
    min(HardRightLDRValue,RightLDRValue) > LDR_LOCK_THRESHOLD*/)
  {
    LeftMotorPWMValue  = 0;
    RightMotorPWMValue = 0;
    //Serial.println(0);
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                    TurnDuration);       
  }

  else if ((CentralLDRDiffMagnitude > LDR_DIFF_THRESHOLD) && 
      max(CentralLDRDiffMagnitude,
      max(RightLDRDiffMagnitude,LeftLDRDiffMagnitude))
      ==CentralLDRDiffMagnitude /*&& RightLDRDiffMagnitude,LeftLDRDiffMagnitude && 
      min(LeftLDRValue,RightLDRValue)> LOW_THRESHOLD_LDR_VALUE*/)
        
  {
    if (LeftLDRValue > RightLDRValue)
    {
      LeftMotorPWMValue  = MOTOR_L_LEFT_TURN_PWM_VALUE;
      RightMotorPWMValue = MOTOR_R_LEFT_TURN_PWM_VALUE;
      TurnDuration       = MOTOR_TURN_DURATION_MS;
      //Serial.println(1);

    }
    if (LeftLDRValue < RightLDRValue)
    {
      LeftMotorPWMValue  = MOTOR_L_RIGHT_TURN_PWM_VALUE;
      RightMotorPWMValue = MOTOR_R_RIGHT_TURN_PWM_VALUE;
      TurnDuration       = MOTOR_TURN_DURATION_MS;
      //Serial.println(2);

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
          min(HardLeftLDRValue,LeftLDRValue)> LOW_THRESHOLD_LDR_VALUE*/) 
  { 
    LeftMotorPWMValue  = MOTOR_L_HARD_LEFT_TURN_PWM_VALUE;
    RightMotorPWMValue = MOTOR_R_HARD_LEFT_TURN_PWM_VALUE;
    TurnDuration       = MOTOR_TURN_DURATION_MS;
    //Serial.println(3);

       
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                    TurnDuration);
  }
  else if (RightLDRDiffMagnitude > LDR_DIFF_THRESHOLD &&
          max(CentralLDRDiffMagnitude,
          max(RightLDRDiffMagnitude,LeftLDRDiffMagnitude))
          ==RightLDRDiffMagnitude &&
          HardRightLDRValue > RightLDRValue /*&&
          min(HardRightLDRValue,RightLDRValue> LOW_THRESHOLD_LDR_VALUE)*/)
  {

    LeftMotorPWMValue  = MOTOR_L_HARD_RIGHT_TURN_PWM_VALUE;
    RightMotorPWMValue = MOTOR_R_HARD_RIGHT_TURN_PWM_VALUE;
    TurnDuration       = MOTOR_TURN_DURATION_MS;
    //Serial.println(4);
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                    TurnDuration);   
  }

  //else
  //{   
    /*if (max(LeftLDRValue, RightLDRValue) > LDR_LOCK_THRESHOLD ||
        max(HardLeftLDRValue,LeftLDRValue) > LDR_LOCK_THRESHOLD ||
        max(HardRightLDRValue,RightLDRValue) > LDR_LOCK_THRESHOLD)
        
    {
      LeftMotorPWMValue  = 0;
      RightMotorPWMValue = 0;
      //Serial.println(0);

    }*/
  /*else if (max(max(LeftLDRValue, RightLDRValue), 
          max(HardLeftLDRValue, HardRightLDRValue)) 
          < LOW_THRESHOLD_LDR_VALUE)
  {
    LeftMotorPWMValue  = 0;
    RightMotorPWMValue = 0;
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                     TurnDuration);      
  }*/
  else
  {  
    LeftMotorPWMValue  = MOTOR_L_FORWARD_PWM_VALUE;
    RightMotorPWMValue = MOTOR_R_FORWARD_PWM_VALUE;
  
  
    
    UpdateMotorSpeed(LeftMotorPWMValue,
                    RightMotorPWMValue,
                     0);
  }

  
  
}
