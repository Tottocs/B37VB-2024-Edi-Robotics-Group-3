//C code on Arduino UNO

#define FIRMWARE_VERSION "v.1.0"

#define BITRATE_SPEED 9600

#define MOT_L1_PIN 5 // Changed the names of the pins  R AND L for left and right
#define MOT_L2_PIN 6
#define MOT_R1_PIN 9
#define MOT_R2_PIN 10
#define UPDATE_PERIOD 1500

int leftServoSpeed = 235; // correct vlue 235
int rightServoSpeed = 200;

void setup() {
  //initiate
  Serial.begin(BITRATE_SPEED);
  
  //H-Bridge switches
  pinMode(MOT_L1_PIN, OUTPUT);
  pinMode(MOT_L2_PIN, OUTPUT);
  pinMode(MOT_R1_PIN, OUTPUT);
  pinMode(MOT_R2_PIN, OUTPUT);

  digitalWrite(MOT_L1_PIN, LOW);
  digitalWrite(MOT_L2_PIN, LOW);
  digitalWrite(MOT_R1_PIN, LOW);
  digitalWrite(MOT_R2_PIN, LOW);
}

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

void set_motor_currents(int pwm_L, int pwm_R)
{
  set_motor_pwm(pwm_L, MOT_L1_PIN, MOT_L2_PIN);
  set_motor_pwm(pwm_R, MOT_R1_PIN, MOT_R2_PIN);

  // Print a status message to the console.
  Serial.print("Set motor R PWM = ");
  Serial.print(pwm_L);
  Serial.print(" motor R PWM = ");
  Serial.println(pwm_R);
}

void forward_and_wait(int pwm_L, int pwm_R, int duration)
{
  set_motor_currents(pwm_L, pwm_R);
  delay(duration);
}

void loop()
{

  forward_and_wait(leftServoSpeed,rightServoSpeed,UPDATE_PERIOD); // sets speed of motors to value entered above for 0.5 sec and keeps repeating
}
