
#define pin_motor_left_dir  7
#define pin_motor_left_pwm  5
#define pin_motor_right_dir 8
#define pin_motor_right_pwm 6

float motor_left_speed = 0.0;
float motor_right_speed = 0.0;


///// serial input
#define MAX_BUF 64
char buf[MAX_BUF]; // command received on serial
int8_t bufindex; //index
char c = 0;


void setup() {
  //Init Serial port with 115200 buad rate
  Serial.begin(115200);
  setup_motors();
  bufindex = 0;
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
  motor_left_speed = 0;
  motor_right_speed = 0;
}


void loop() {
  read_serial();
  update_motors();
}

// will read serial and call parsing if applicable
void read_serial() {
  while(Serial.available() > 0){
    if(bufindex > MAX_BUF-1){
      bufindex=0;
      *buf = 0;
      //Serial.print("ko overflow\n");
      break;
    }
    
    c = Serial.read();
    //Serial.print(String("'") + String(c)+ String("'\n"));
    buf[bufindex++] = c;
    
    if(c == '\n'){
      buf[--bufindex] = 0; // remove the final \n
      parse_command(buf);
      *buf = 0;
      bufindex = 0;
    }
  }
}

void parse_command(char * cmd) {
  if (cmd[0] == 's') { // set speed
    char *token;
    cmd[0] = ','; // to skip the 's' in the following strtok()
    token = strtok(cmd, ",");
    motor_left_speed = atof(token);
    token = strtok(NULL, ",");
    motor_right_speed = atof(token);
  } 
}






//Will update both motors
void update_motors() {

  moveLeftMotor(motor_left_speed);
  moveRightMotor(motor_right_speed);

  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\t");
  Serial.print(motor_right_speed);
  Serial.print("\n");
}


void moveLeftMotor(float motor_speed) {
  if (motor_speed < -255) motor_speed = -255;
  if (motor_speed > +255) motor_speed = +255; 
  if (motor_speed < 0) {
    digitalWrite(pin_motor_left_dir, 0);
    analogWrite(pin_motor_left_pwm , (int)(-motor_speed));
  } else {
    digitalWrite(pin_motor_left_dir, 1);
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


