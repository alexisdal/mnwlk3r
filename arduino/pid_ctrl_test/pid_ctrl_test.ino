// inspired from https://github.com/qboticslabs/Chefbot_ROS_pkg/blob/master/arduino/chefbot/chefbot.ino


#define pin_motor_left_dir  7  // dir2
#define pin_motor_left_pwm  5  // pwm2
#define pin_motor_right_dir 8  // dir1
#define pin_motor_right_pwm 6  // pwm1

// motor pin out on cytron MDD10A Rev 2.0 
// product page => https://www.cytron.io/p-mdd10a?search=mDD10a&description=1
// manual       => https://docs.google.com/document/d/1ol8nICCTTw5dAHHE_hju08cCVH2GN5_Y3cGC6B4Gbas/edit
// left motor 
// white negative <=> M2B
// red positive   <=> M2A
// right motor
// white negative <=> M1B
// red positive   <=> M1A
// to test this when we click on the test button MAx => each motor must turn counter clockwise
// logic = we keep connection/pinout the same on the board and we want to adjust the code to change direction and encoder values accordingly
//         a positive speed value command from serial must make the wheel turn so that mnwlk3r goes forward   and encoder count increases
//         a negative speed value command from serial must make the wheel turn so that mnwlk3r goes backwards and encoder count decreases


// motors & wheels & encoders => https://www.dfrobot.com/product-1203.html 


float motor_left_speed = 0.0; // now in RPM
float motor_right_speed = 0.0;


///// serial input
#define MAX_BUF 64
char buf[MAX_BUF]; // command received on serial
int8_t bufindex; //index
char c = 0;


//Encoder pins definition

// Left encoder
int Left_Encoder_PinA = 2;
int Left_Encoder_PinB = 9;
volatile long Left_Encoder_Ticks = 0;
//Variable to read current state of left encoder pin
volatile bool LeftEncoderBSet;

//Right Encoder
int Right_Encoder_PinA = 3;
int Right_Encoder_PinB = 10;
volatile long Right_Encoder_Ticks = 0;
//Variable to read current state of right encoder pin
volatile bool RightEncoderBSet;

// with PID lib
#include <PID_v1.h>
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);

#define PID_LOOP_TIME_MS        200                     // PID loop time (milli sec. 100ms <=> 10Hz)
double  right_RPM = 0.0;
long last_update_ms = 0;  


void setup() {
  //Init Serial port with 115200 baud rate
  Serial.begin(115200);
  bufindex = 0;
  setup_motors();
  setup_encoders();
  setup_pid();

}

void loop() {
  read_serial();
  update_pid();
}

void setup_motors() {
  pinMode(pin_motor_left_dir, OUTPUT);
  pinMode(pin_motor_left_pwm, OUTPUT);
  pinMode(pin_motor_right_dir, OUTPUT);
  pinMode(pin_motor_right_pwm, OUTPUT);
  digitalWrite(pin_motor_left_dir, 0);
  analogWrite(pin_motor_left_pwm , 0);
  digitalWrite(pin_motor_right_dir, 0);
  analogWrite(pin_motor_right_pwm, 0);
}




// will read serial and call parsing if applicable
void read_serial() {
  while (Serial.available() > 0) {
    if (bufindex > MAX_BUF - 1) {
      bufindex = 0;
      *buf = 0;
      //Serial.print("ko overflow\n");
      break;
    }

    c = Serial.read();
    //Serial.print(String("'") + String(c)+ String("'\n"));
    buf[bufindex++] = c;

    if (c == '\n') {
      buf[--bufindex] = 0; // remove the final \n
      parse_command(buf);
      *buf = 0;
      bufindex = 0;
    }
  }
}

void parse_command(char * cmd) {
  char *token;
  if (cmd[0] == 's') { // set speed
    cmd[0] = ','; // to skip the 's' in the following strtok()
    token = strtok(cmd, ",");
    motor_left_speed = atof(token);
    token = strtok(NULL, ",");
    motor_right_speed = atof(token);
    set_target_speed(motor_right_speed);
    set_pid_mode(1);
  } else if (cmd[0] == 'k') { // set PID tuning (kp, ki, kd)
    cmd[0] = ',';
    token = strtok(cmd, ",");
    double kp = atof(token);
    token = strtok(NULL, ",");
    double ki = atof(token);
    token = strtok(NULL, ",");
    double kd = atof(token);
    token = strtok(NULL, ",");
    set_pid_tuning(kp, ki, kd);
    set_pid_mode(1);
  } else if (cmd[0] == 'o') { // set PID active 1 / inactive 0
    cmd[0] = ',';
    token = strtok(cmd, ",");
    int m = atoi(token);
    set_pid_mode(m);
  }
}



void moveLeftMotor(float motor_speed) {
  if (motor_speed < -255) motor_speed = -255;
  if (motor_speed > +255) motor_speed = +255;
  // with default pinout, left motor direction must be reversed compared to right motor
  if (motor_speed < 0) {
    digitalWrite(pin_motor_left_dir, 1);
    analogWrite(pin_motor_left_pwm , (int)(-motor_speed));
  } else {
    digitalWrite(pin_motor_left_dir, 0);
    analogWrite(pin_motor_left_pwm , (int)(motor_speed));
  }
}

void moveRightMotor(float motor_speed) {
  if (motor_speed < -255) motor_speed = -255;
  if (motor_speed > +255) motor_speed = +255;
  if (motor_speed < 0) {
    digitalWrite(pin_motor_right_dir, 0);
    analogWrite(pin_motor_right_pwm , (int)(-motor_speed));
  } else {
    digitalWrite(pin_motor_right_dir, 1);
    analogWrite(pin_motor_right_pwm , (int)(motor_speed));
  }
}






void setup_encoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT);      // sets pin A pullup
  pinMode(Left_Encoder_PinB, INPUT);      // sets pin B pullup
  attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_left_encoder, RISING);


  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT);      // sets pin A pullup
  pinMode(Right_Encoder_PinB, INPUT);      // sets pin B pullup

  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_right_encoder, RISING);

}


void do_left_encoder()
{
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  // here we need to decrease counter
  Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;

}
void do_right_encoder()
{
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
}


void setup_pid() 
{
  Input = 0.0;
  // No load RPM (after gearbox): 146rpm@12V
  // 100 RPM target = nice round figure below specs capacity
  set_target_speed(99.0);
  
  myPID.SetSampleTime(PID_LOOP_TIME_MS);
  //myPID.SetOutputLimits(min, max)  
  //myPID.SetOutputLimits(15, 255); // below 15 PWM does nothing (maybe friction due to gearbox ?)
  myPID.SetOutputLimits(0, 255);
  //double Kp=2, Ki=5, Kd=1; // why not?
  //set_pid_tuning(2,5,1); // oscillate
  //set_pid_tuning(0.5,1,0);

  set_pid_mode(0); 
}

void set_pid_mode(int mode) 
{ 
  if (mode == 1) { 
    myPID.SetMode(AUTOMATIC);
  } else { 
    myPID.SetMode(MANUAL);
    Output = 0;
  }
}

void set_pid_tuning(double kp, double ki, double kd) { myPID.SetTunings(kp, ki, kd); }

void set_target_speed(double rpm) 
{
  if (rpm < -146) rpm = -146;
  if (rpm > 146) rpm = 146;
  Setpoint = rpm;
}

void update_pid() 
{
   // inspiration from => https://forum.arduino.cc/index.php?topic=8652.0   (Aug 2010)
  // static variables within functions are local variable except that they survive between calls. init once only
  // pitfall : initialized only first time function is called.
  // https://www.arduino.cc/reference/en/language/variables/variable-scope--qualifiers/static/
  //static long countAnt = 0;            
  //speed_act = ((count - countAnt)*(60*(1000/LOOPTIME)))/(16*29);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
  //countAnt = count;     

  static long Right_Encoder_Ticks_Ant = 0;
  long now = millis();
  if (now - last_update_ms > PID_LOOP_TIME_MS) {
    // Encoder Resolution: 13 PPR (663 PPR for gearbox shaft)
    // Gear ratio: 51:1
    right_RPM = ((Right_Encoder_Ticks - Right_Encoder_Ticks_Ant)*(60*(1000.0/PID_LOOP_TIME_MS)))/(13*51); 
    Right_Encoder_Ticks_Ant = Right_Encoder_Ticks;
    Input = right_RPM;
    myPID.Compute();
    moveRightMotor(Output);
    //Serial.print("r,");
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print(Input);
    //Serial.print(",");
    //Serial.print(Output);

    Serial.print(",");
    Serial.print(Left_Encoder_Ticks);
    Serial.print(",");
    Serial.print(Right_Encoder_Ticks);
    Serial.print("\n");

    /*Serial.print("k,");
    Serial.print(myPID.GetKp());
    Serial.print(",");
    Serial.print(myPID.GetKi());
    Serial.print(",");
    Serial.print(myPID.GetKd());
    Serial.print(",");
    Serial.print(myPID.GetMode());
    Serial.print(",");
    Serial.print(myPID.GetDirection());
    Serial.print("\n");
    */
    last_update_ms = now;
  }
}


