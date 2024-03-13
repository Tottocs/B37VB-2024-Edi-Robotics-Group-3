#define FIRMWARE_VERSION "v1.0.0"

#define UART_BAUDRATE 9600

#define MOTOR_L1_PIN 5
#define MOTOR_L2_PIN 6
#define MOTOR_R1_PIN 9
#define MOTOR_R2_PIN 10
#define LDR_L_PIN A0 // Mapped to ATMega328P PCO (Pin 23) on Arduino Uno
#define LDR_R_PIN A1 // Mapped to ATMega328P PC1 (Pin 24) on Arduino Uno

#define ADC_RESOLUTION         10                    // 10-Bit ADC
#define ADC_OUTPUT_RANGE       (1 << ADC_RESOLUTION) // (2^10 = 1024)
#define ADC_FULL_SCALE_VOLTAGE 5.0                   // Full-Scale Voltage

#define UPDATE_PERIOD_MS 1000

#define PWM_VALUE_MAX 255
#define PWM_VALUE_MIN -255

#define LDR_DIFF_THRESHOLD 100
#define LDR_LOCK_THRESHOLD 800

//motor PWM Controls
#define MOTOR_L_FORWARD_PWM_VALUE 150
#define MOTOR_R_FORWARD_PWM_VALUE 150

#define MOTOR_L_REVERSE_PWM_VALUE -150
#define MOTOR_R_REVERSE_PWM_VALUE -150

#define MOTOR_L_LEFT_TURN_PWM_VALUE 100
#define MOTOR_R_LEFT_TURN_PWM_VALUE 200

#define MOTOR_L_RIGHT_TURN_PWM_VALUE 200
#define MOTOR_R_RIGHT_TURN_PWM_VALUE 100

#define MOTOR_TURN_DURATION_MS 250



// Function to Set Motor Control Parameters
void SetMotorControlParameters(int PWMValue,
                               unsigned int HBridgeControlPinA,
                               unsigned int HBridgeControlPinB)
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
void UpdateMotorSpeed(int LeftMotorPWMValue,
                      int RightMotorPWMValue,
                      unsigned long DurationMilliseconds)
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
  delay(DurationMilliseconds);
}

// Function to measure LDR Circuit Voltage
void MeasureLDRCircuitVoltage(int PinNumber)
{
  int   AnalogReadResult;
  float LDRCircuitVoltage;
  
  // Calculate the voltage per ADC output code
  const float VoltagePerADCCode = (ADC_FULL_SCALE_VOLTAGE / ADC_OUTPUT_RANGE);
  
  // Measure the voltage from the LDR circuit using ADC
  AnalogReadResult = analogRead(PinNumber);
  
  // Calculate the circuit voltage from the ADC output code
  LDRCircuitVoltage = AnalogReadResult * VoltagePerADCCode;
  return AnalogReadResult;

  // Output results to console
  Serial.print("ADC Output Code: ");
  Serial.println(AnalogReadResult);
  Serial.print("LDR Circuit Voltage: ");
  Serial.print(LDRCircuitVoltage);
  Serial.println("V");
  Serial.println();
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
  Serial.println("B37VB Motor Control Basics and LDR light tracking Demonstration");
  Serial.print("Version: ");
  Serial.println(FIRMWARE_VERSION);



}

// Loop function runs over and over again forever
void loop()
{

 int LeftLDRValue;
  int RightLDRValue;
  
  unsigned int LDRDiffMagnitude;
  
  int LeftMotorPWMValue;
  int RightMotorPWMValue;
  
  unsigned long TurnDuration;
  
  LeftLDRValue  = MeasureLDRCircuitVoltage(LDR_L_PIN);
  RightLDRValue = MeasureLDRCircuitVoltage(LDR_R_PIN);
  
  LDRDiffMagnitude = abs(LeftLDRValue - RightLDRValue);
  
  if (LDRDiffMagnitude > LDR_DIFF_THRESHOLD)
  {
    if (LeftLDRValue > RightLDRValue)
    {
      LeftMotorPWMValue  = MOTOR_L_LEFT_TURN_PWM_VALUE;
      RightMotorPWMValue = MOTOR_R_LEFT_TURN_PWM_VALUE;
      TurnDuration       = MOTOR_TURN_DURATION_MS;
    }
    else
    {
      LeftMotorPWMValue  = MOTOR_L_RIGHT_TURN_PWM_VALUE;
      RightMotorPWMValue = MOTOR_R_RIGHT_TURN_PWM_VALUE;
      TurnDuration       = MOTOR_TURN_DURATION_MS;
    }
    
    UpdateMotorSpeed(LeftMotorPWMValue,
                     RightMotorPWMValue,
                     TurnDuration);
  }
  else
  {   
    if (min(LeftLDRValue, RightLDRValue) > LDR_LOCK_THRESHOLD)
    {
      LeftMotorPWMValue  = 0;
      RightMotorPWMValue = 0;
    }
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
