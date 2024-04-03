//C code on Arduino UNO

#define FIRMWARE_VERSION "v.1.0"

#define UART_BAUDRATE 9600 //At which speed the information is sent through the Arduino

#define LED_PIN 7 //Pin on the arduino board where the 

#define MOT_L1_PIN 5  //Connected to IN2 pin to the servos 
#define MOT_L2_PIN 6  //Connected to IN1 pin to the servos 
#define MOT_R1_PIN 9  //Connected to IN3 pin to the servos 
#define MOT_R2_PIN 10 //Connected to IN4 pin to the servos 
#define UPDATE_PERIOD 1500 // The servo speed updates every 1500 milliseconds + BLINK_DURATION
#define BLINK_PERIOD 100 //Blinks for 100 milliseconds after each updated 
int InitialLeftServoPWM = 100; //The initial pwm value that will increase to match the right pwm
const int ConstantRightServoPWM = 200; // Three chosen constant values of either 100 ,200 and 255
        // This can be changed to the right or left servo depending on which is lagging behind 

void setup(){
  //initiate
  Serial.begin(UART_BAUDRATE);
  
  //H-Bridge switches pins to output
  pinMode(MOT_L1_PIN, OUTPUT);
  pinMode(MOT_L2_PIN, OUTPUT);
  pinMode(MOT_R1_PIN, OUTPUT);
  pinMode(MOT_R2_PIN, OUTPUT);
  // The pins are set to low, indicating them being off
  digitalWrite(MOT_L1_PIN, LOW);
  digitalWrite(MOT_L2_PIN, LOW);
  digitalWrite(MOT_R1_PIN, LOW);
  digitalWrite(MOT_R2_PIN, LOW);
  //Setting LED pin to output
  pinMode(LED_PIN, OUTPUT);
}

void Set_Motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  //reverse speeds
    analogWrite(IN1_PIN, -pwm); /*if the pwm signal is negative, 
                              the signal gets sent through pin IN1 instead of pin IN2 */
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}
//Inputs pwm for right and left and sets speed
void Set_Motor_Currents(int pwm_L, int pwm_R)
{
  Set_Motor_pwm(pwm_L, MOT_L1_PIN, MOT_L2_PIN);
  Set_Motor_pwm(pwm_R, MOT_R1_PIN, MOT_R2_PIN);

  //Prints status message to the console.
  Serial.print("Set motor R PWM = ");
  Serial.print(pwm_L);
  Serial.print(" motor R PWM = ");
  Serial.println(pwm_R);
}
//Drives forward and changes left servo speed every 1500 millieconds with a 100 millisecond blink
void Forward_Lag_Check(int PWM_L, int PWM_R, int Duration, int BlinkDuration)
{
  for (int LeftVariablePWM=PWM_L;LeftVariablePWM<=255;LeftVariablePWM=+5)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(BlinkDuration);
    digitalWrite(LED_PIN, LOW);
    Set_Motor_Currents(LeftVariablePWM, PWM_R);
    delay(Duration);    

  }
}

void loop()
{
  //Implementing function in loop
  Forward_Lag_Check(InitialLeftServoPWM,ConstantRightServoPWM,UPDATE_PERIOD,BLINK_PERIOD); // sets speed of motors to value entered above for 0.5 sec and keeps repeating
}
