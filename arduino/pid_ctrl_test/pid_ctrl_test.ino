// inspired from https://github.com/qboticslabs/Chefbot_ROS_pkg/blob/master/arduino/chefbot/chefbot.ino


// https://github.com/NicksonYap/digitalWriteFast
#include <digitalWriteFast.h>  


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

// debug is for adjusting fisplay so that we can use the serialplotter
// to adjust the tuning of the PID
// https://bitbucket.org/hyOzd/serialplot/downloads/ 
//uint8_t debug = 1;

//Encoder pins definition

// Left encoder
#define Left_Encoder_PinA  2
#define Left_Encoder_PinB  9
volatile long Left_Encoder_Ticks = 0;
//Variable to read current state of left encoder pin
volatile bool LeftEncoderBSet;

//Right Encoder
#define  Right_Encoder_PinA  3
#define  Right_Encoder_PinB  10
volatile long Right_Encoder_Ticks = 0;
//Variable to read current state of right encoder pin
volatile bool RightEncoderBSet;

//double ticks_per_revolution = 657.0;
int ticks_per_revolution = 657;

/////////////////////////////////////
// with PID lib
#include <PID_v1.h>
//Define Variables we'll be connecting to
double Setpoint_left, Input_left, Output_left;
double Setpoint_right, Input_right, Output_right;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID_left(&Input_left, &Output_left, &Setpoint_left, Kp, Ki, Kd, P_ON_E, DIRECT);
PID myPID_right(&Input_right, &Output_right, &Setpoint_right, Kp, Ki, Kd, P_ON_E, DIRECT);


#define PID_LOOP_TIME_MS        50                     // PID loop time (milli sec. 100ms <=> 10Hz)
double  right_RPM = 0.0;
double  left_RPM = 0.0;
long last_update_ms = 0;  



/////////////////////////////////////
//  odometry within arduino
// references http://andrewjkramer.net/arduino-odometry/
//            https://github.com/1988kramer/motor_control/blob/master/DifferentialDrive.cpp
#include <math.h>  // for cos/sin
double _x = 0.0;
double _y = 0.0;
double _theta = PI;
double wheels_distance_mm = 0.00001;
double degrees_per_millimeter = 0.00001;
  


void setup() {
  //Init Serial port with 115200 baud rate
  Serial.begin(115200);
  bufindex = 0;
  setup_motors();
  setup_encoders();
  //setup_odometry();
  setup_pid();
}

void loop() {
  read_serial();
  //print_encoder();
  update_pid();
}



//void set_debug(int d) { debug = d; }

void setup_motors() {
  pinModeFast(pin_motor_left_dir, OUTPUT);
  pinMode(pin_motor_left_pwm, OUTPUT);
  pinModeFast(pin_motor_right_dir, OUTPUT);
  pinMode(pin_motor_right_pwm, OUTPUT);
  digitalWriteFast(pin_motor_left_dir, 0);
  analogWrite(pin_motor_left_pwm , 0);
  digitalWriteFast(pin_motor_right_dir, 0);
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
    set_target_speed_left(motor_left_speed);
    token = strtok(NULL, ",");
    motor_right_speed = atof(token);
    set_target_speed_right(motor_right_speed);
    set_pid_mode_left(1); set_pid_mode_right(1);
  } else if (cmd[0] == 'k') { // set PID tuning (kp, ki, kd)
    cmd[0] = ',';
    token = strtok(cmd, ",");
    double kp = atof(token);
    token = strtok(NULL, ",");
    double ki = atof(token);
    token = strtok(NULL, ",");
    double kd = atof(token);
    set_pid_tuning(kp, ki, kd);
    set_pid_mode_left(1); set_pid_mode_right(1);
  } else if (cmd[0] == 'o') { // set PID active 1 / inactive 0
    cmd[0] = ',';
    token = strtok(cmd, ",");
    int m = atoi(token);
    set_pid_mode_left(m); set_pid_mode_right(m);
//  } else if (cmd[0] == 'd') { // set debug active (1) / inactive (0) DEFAULT
//    cmd[0] = ',';
//    token = strtok(cmd, ",");
//    int d = atoi(token);
//    set_debug(d);
  }
}



void moveLeftMotor(float motor_speed) {
  if (motor_speed < -255) motor_speed = -255;
  if (motor_speed > +255) motor_speed = +255;
  // with default pinout, left motor direction must be reversed compared to right motor
  if (motor_speed < 0) {
    digitalWriteFast(pin_motor_left_dir, 1);
    analogWrite(pin_motor_left_pwm , (int)(-motor_speed));
  } else {
    digitalWriteFast(pin_motor_left_dir, 0);
    analogWrite(pin_motor_left_pwm , (int)(motor_speed));
  }
}

void moveRightMotor(float motor_speed) {
  if (motor_speed < -255) motor_speed = -255;
  if (motor_speed > +255) motor_speed = +255;
  if (motor_speed < 0) {
    digitalWriteFast(pin_motor_right_dir, 0);
    analogWrite(pin_motor_right_pwm , (int)(-motor_speed));
  } else {
    digitalWriteFast(pin_motor_right_dir, 1);
    analogWrite(pin_motor_right_pwm , (int)(motor_speed));
  }
}






void setup_encoders()
{
  // Quadrature encoders
  // Left encoder
  pinModeFast(Left_Encoder_PinA, INPUT);      // sets pin A pullup
  pinModeFast(Left_Encoder_PinB, INPUT);      // sets pin B pullup
  attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_left_encoder, RISING);


  // Right encoder
  pinModeFast(Right_Encoder_PinA, INPUT);      // sets pin A pullup
  pinModeFast(Right_Encoder_PinB, INPUT);      // sets pin B pullup

  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_right_encoder, RISING);

}


void do_left_encoder()
{
  LeftEncoderBSet = digitalReadFast(Left_Encoder_PinB);   // read the input pin
  // here we need to decrease counter
  Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;

}
void do_right_encoder()
{
  RightEncoderBSet = digitalReadFast(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
}

/*void print_encoder(){
  if (debug == 0) {
    Serial.print("e,");
    Serial.print(Left_Encoder_Ticks);
    Serial.print(",");
    Serial.print(Right_Encoder_Ticks);
    Serial.print("\n");
  }
}*/


void setup_pid() 
{
  Input_right = 0.0;
  Input_left = 0.0;
  // No load RPM (after gearbox): 146rpm@12V
  // 100 RPM target = nice round figure below specs capacity
  set_target_speed_left(0.0);
  set_target_speed_right(0.0);
  
  myPID_right.SetSampleTime(PID_LOOP_TIME_MS);
  //myPID.SetOutputLimits(min, max)  
  //myPID.SetOutputLimits(15, 255); // below 15 PWM does nothing (maybe friction due to gearbox ?)
  myPID_right.SetOutputLimits(-255, 255);

  myPID_left.SetSampleTime(PID_LOOP_TIME_MS);
  myPID_left.SetOutputLimits(-255, 255);
  
  //double Kp=2, Ki=5, Kd=1; // why not?
  //set_pid_tuning(2,5,1); // oscillate
  //set_pid_tuning(0.5,2.05,0.01); // best default tuning values I could come up with
  set_pid_tuning(0.2,3,0.06);
  set_pid_mode_left(0); 
  set_pid_mode_right(0); 
}

void set_pid_mode_left(int mode) 
{ 
  if (mode == 1) { 
    myPID_left.SetMode(AUTOMATIC);
  } else { 
    myPID_left.SetMode(MANUAL);
    Output_left = 0;
  }
}
void set_pid_mode_right(int mode) 
{ 
  if (mode == 1) { 
    myPID_right.SetMode(AUTOMATIC);
  } else { 
    myPID_right.SetMode(MANUAL);
    Output_right = 0;
  }
}

void set_pid_tuning(double kp, double ki, double kd) { 
  myPID_left.SetTunings(kp, ki, kd); 
  myPID_right.SetTunings(kp, ki, kd);
}

void set_target_speed_left(double rpm) 
{
  if (rpm < -146) rpm = -146;
  if (rpm > 146) rpm = 146;
  Setpoint_left = rpm;
}
void set_target_speed_right(double rpm) 
{
  if (rpm < -146) rpm = -146;
  if (rpm > 146) rpm = 146;
  Setpoint_right = rpm;
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
  static long Left_Encoder_Ticks_Ant = 0;
  long now = millis();
  if (now - last_update_ms > PID_LOOP_TIME_MS) {
    // Encoder Resolution: 13 PPR (663 PPR for gearbox shaft)
    // Gear ratio: 51:1
    //right_RPM = ((Right_Encoder_Ticks - Right_Encoder_Ticks_Ant)*(60*(1000.0/PID_LOOP_TIME_MS)))/(13*51); 
    right_RPM = ((Right_Encoder_Ticks - Right_Encoder_Ticks_Ant)*(60*(1000.0/PID_LOOP_TIME_MS)))/(ticks_per_revolution); // calibration says 657 and not manufacturer value 13*51=663 
    Input_right = right_RPM;
    myPID_right.Compute();
    if (Setpoint_right == 0 && (-20 < Output_right || Output_right < 20)) {
      set_pid_mode_right(0);
      moveRightMotor(0);
    } else {
      moveRightMotor(Output_right);
    }      
    
    //if (debug == 0) {
    //  Serial.print("r,");
    //}
    Serial.print(Setpoint_right);
    Serial.print(",");
    Serial.print(Input_right);
    //Serial.print(",");
    //Serial.print(Output_right);


    left_RPM = ((Left_Encoder_Ticks - Left_Encoder_Ticks_Ant)*(60*(1000.0/PID_LOOP_TIME_MS)))/(ticks_per_revolution); // calibration says 657 and not manufacturer value 13*51=663 
    Input_left = left_RPM;
    myPID_left.Compute();
    if (Setpoint_left == 0 && (-20 < Output_left  || Output_left  < 20 ) ) {
      set_pid_mode_left(0);
      moveLeftMotor(0);
    } else {
      moveLeftMotor(Output_left);
    }      
    //Serial.print("r,");
    Serial.print(",");
    Serial.print(Setpoint_left);
    Serial.print(",");
    Serial.print(Input_left);
    //Serial.print(",");
    //Serial.print(Output_left);
    //Serial.print("\n");

    

    //update_diff_drive_position(Left_Encoder_Ticks_Ant, Right_Encoder_Ticks_Ant);
    Right_Encoder_Ticks_Ant = Right_Encoder_Ticks;
    Left_Encoder_Ticks_Ant = Left_Encoder_Ticks;
    //Serial.print("       ,");
    //Serial.print(_x);
    //Serial.print(",");
    //Serial.print(_y);
    //Serial.print(",");
    //Serial.print(_theta);

    

    Serial.print(",");
    Serial.print(Left_Encoder_Ticks);
    Serial.print(",");
    Serial.print(Right_Encoder_Ticks);
    //Serial.print("\n");
    

    /*//Serial.print("k,");
    Serial.print(",");
    Serial.print(myPID_left.GetKp());
    Serial.print(",");
    Serial.print(myPID_left.GetKi());
    Serial.print(",");
    Serial.print(myPID_left.GetKd());
    //Serial.print(",");
    //Serial.print(myPID.GetMode());
    //Serial.print(",");
    //Serial.print(myPID.GetDirection());
    Serial.print("\n");
    */

    
    Serial.print(",");
    Serial.print(now);
    Serial.print("\n");
    last_update_ms = now;
  //} else {
    //Serial.print("x\n");
  }
}

/*
void setup_odometry() {
  // default values
  //set_odometry(double wheel_diameter_mm_, double ticks_per_revolution_, double wheels_distance_mm_) {
  set_odometry(136.0, 657, 328);
}

void set_odometry(double wheel_diameter_mm_, double ticks_per_revolution_, double wheels_distance_mm_) {
  ticks_per_revolution = ticks_per_revolution_;
  wheels_distance_mm = wheels_distance_mm_;
  double wheel_distance_per_revolution_in_mm = wheel_diameter_mm_ * PI; // 136*PI = 427.25 millimeters per 360 degrees
  degrees_per_millimeter = 360.0 / wheel_distance_per_revolution_in_mm; // 360 / 427.25 = 0.842 degrees per mm
  _x = 0.0;
  _y = 0.0;
  _theta = 0.0;
}


void update_diff_drive_position(long Left_Encoder_Ticks_Ant, long Right_Encoder_Ticks_Ant)
{
   // get the angular distance traveled by each wheel since the last update
   double leftDegrees = (Left_Encoder_Ticks - Left_Encoder_Ticks_Ant) / ticks_per_revolution * 360;
   double rightDegrees = (Right_Encoder_Ticks - Right_Encoder_Ticks_Ant) / ticks_per_revolution * 360;

   // convert the angular distances to linear distances
   double dLeft = leftDegrees / degrees_per_millimeter;
   double dRight = rightDegrees / degrees_per_millimeter;

   // calculate the length of the arc traveled by Colin
   double dCenter = (dLeft + dRight) / 2.0;

   Serial.print(",");
   Serial.print(dLeft);
   Serial.print(",");
   Serial.print(dRight);
   Serial.print(",");
   Serial.print(dCenter);


   // calculate Colin's change in angle
   double phi = (dRight - dLeft) / (double)wheels_distance_mm;

   Serial.print(",");
   Serial.print(phi);
   
   // add the change in angle to the previous angle
   _theta += phi;
   // constrain _theta to the range 0 to 2 pi
   if (_theta > 2.0 * PI) _theta -= 2.0 * PI;
   if (_theta < 0.0) _theta += 2.0 * PI;

   // update Colin's x and y coordinates
   _x += dCenter * cos(_theta);
   _y += dCenter * sin(_theta);
}

*/

