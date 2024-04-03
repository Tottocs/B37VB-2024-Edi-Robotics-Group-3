//C code on Arduino UNO

#define FirmwareVersion "v.1.0"

#define BuadrateSpeed 9600 //At which speed the information is sent through the Arduino

#define MotorLeft1Pin 5 // Pins on the Arduino for that takes that sends PWM values
#define MotorLeft2Pin 6
#define MotorRight1Pin 9
#define MotorRight2Pin 10
#define UpdatePeriod 2000 // The variable speed will change every 2000 millisec

int InitialLeftServoPWM = 100; 
const int ConstantRightServoPWM = 200; // Three chosen constant value of either __ ,__ and __

void setup() {
  //initiate with the Baudrate speed
  Serial.begin(BuadrateSpeed);
  
  //H-Bridge switches
  pinMode(MotorLeft1Pin, OUTPUT);
  pinMode(MotorLeft2Pin, OUTPUT);
  pinMode(MotorRight1Pin, OUTPUT);
  pinMode(MotorRight2Pin, OUTPUT);

  digitalWrite(MotorLeft1Pin, LOW);
  digitalWrite(MotorLeft2Pin, LOW);
  digitalWrite(MotorRight1Pin, LOW);
  digitalWrite(MotorRight2Pin, LOW);
}
//Function that sets up the different motors PWM using Left and Right servo PWM variables
void SetMotorPWM(int PWM, int IN1_Pin, int IN2_Pin)
{
  if (PWM < 0) {  // reverse speeds
    analogWrite(IN1_Pin, -PWM);
    digitalWrite(IN2_Pin, LOW);

  } else { // stop or forward
    digitalWrite(IN1_Pin, LOW);
    analogWrite(IN2_Pin, PWM);
  }
}

void SetMotorCurrents(int PWM_Left, int PWM_Right)
{
  SetMotorPWM(PWM_Left, MotorLeft1Pin, MotorLeft2Pin);
  SetMotorPWM(PWM_Right, MotorRight1Pin, MotorRight2Pin);

  // Print a status message to the console.
  Serial.print("Set motor R PWM = ");
  Serial.print(PWM_Left);
  Serial.print(" motor R PWM = ");
  Serial.println(PWM_Right);
}

void ForwardAndWait(int LeftPWM, int RightPWM, int Duration)
{
  for (int LeftVariablePWM=LeftPWM;LeftVariablePWM<=255;LeftVariablePWM=+5){
    SetMotorCurrents(LeftVariablePWM, RightPWM);
    delay(Duration);
  }
}

void loop()
{
  // sets speed of motors to value entered at the top for 2 sec and keeps repeating
  ForwardAndWait(InitialLeftServoPWM,ConstantRightServoPWM,UpdatePeriod); 
}
