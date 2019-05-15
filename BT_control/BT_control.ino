#include <Wire.h>
#include <SPI.h>

#include <MeMegaPi.h>

#include <Adafruit_PN532.h>

// Robot physical parameters
const double R = 0.03; // m
// Motor 1 - right
const double K1 = 1, p1 = -0.085;
// Motor 2 - left
const double K2 = -1, p2 = 0.085;

// Robot movement parameters
const double max_linear_vel=0.25; // m/s
const double max_angular_vel=2.5; // rad/s

// Const
const double RADS_TO_PWM = 255.0 / 6.3; // TODO callibrate!
const double RADS_TO_RMP = 60.0 / (2.0 * 3.1415);

// Team code
const uint8_t TEAMCODE = 1;

// Localization related variables
// Motor 1 - right wheel
double angle_motor1_pre = 0.0, angle_motor1_cur = 0.0;
double s1 = 0.0;
// Motor 2 - left wheel
double angle_motor2_pre = 0.0, angle_motor2_cur = 0.0;
double s2 = 0.0;
// Robot
double angle_robot_world = 0.0, locX = 0.0, locY = 0.0;

// Communication related variables
uint8_t dCount = 0, syncChar = 'e';
int8_t rec[10] =  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t disp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t next[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// RFID thread related variables
unsigned long int rfid_read_time_prev;
bool flag_new_rfid_result = false;
unsigned long int rfid_read_time_period_ms=500; // ms

// Home detection thread related variables
unsigned long int home_detection_time_prev;
bool flag_new_home_detection_result = false;
unsigned long int home_detection_time_period_ms=500; // ms

// RFID required read variables
uint8_t success, uid[6], uidLength;
uint8_t data[16];
uint8_t key[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


// RFID reader
Adafruit_PN532 rfid_reader(22);
bool rfid_reader_enabled = 0;

// Line Following sensor
MeLineFollower line_following_sensor(PORT_5);

// Motor with encoder 1 - right wheel
MeEncoderOnBoard Encoder_1(SLOT1);

// Motor with encoder 2 - left wheel
MeEncoderOnBoard Encoder_2(SLOT2);

// Motor with encoder 3 - lifter
MeEncoderOnBoard lifter(SLOT3);

// DC motor - gripper
MeMegaPiDCMotor dc(PORT4B);




double saturateVal(double val, double min_val, double max_val)
{
  if(val>max_val)
    val=max_val;
  else if(val<min_val)
    val=min_val;
  return val;    
}

double sliderEasyControl(double val_in, double val_in_dead_min, double val_in_dead_max, double val_in_min, double val_out_min, double val_in_max, double val_out_max)
{
  double val_out = 0;

  double slope_max = val_out_max / (val_in_max - val_in_dead_max);
  double slope_min = val_out_min / (val_in_min - val_in_dead_min);

  if(val_in <= val_in_dead_max && val_in >= val_in_dead_min )
    val_out = 0;
  else if(val_in<val_in_dead_min && val_in>val_in_min)
    val_out = slope_min*(val_in-val_in_dead_min);
  else if(val_in>val_in_dead_max && val_in<val_in_max)
    val_out = slope_max*(val_in-val_in_dead_max);
  else if(val_in<=val_in_min)
    val_out = val_out_min;
  else if(val_in>=val_in_max)
    val_out = val_out_max;
    
  return val_out;
}

void setAngSpeedMotors(double ang_speed_m1, double ang_speed_m2)
{ 
  double right_pwm = RADS_TO_PWM*ang_speed_m1;
  double left_pwm = RADS_TO_PWM*ang_speed_m2;

  //Serial.write("right_pwm: ");
  //Serial.print(right_pwm);
  //Serial.write(" ");
  //Serial.write("left_pwm: ");
  //Serial.print(left_pwm);
  //Serial.write("\n");
  
  // PWM = [-255, 255]
  right_pwm = saturateVal(right_pwm, -255, 255);
  left_pwm = saturateVal(left_pwm, -255, 255);

  //Serial.write("right_pwm: ");
  //Serial.print(right_pwm);
  //Serial.write(" ");
  //Serial.write("left_pwm: ");
  //Serial.print(left_pwm);
  //Serial.write("\n");
  
  //
  Encoder_1.setMotorPwm((int16_t)(right_pwm));
  Encoder_2.setMotorPwm((int16_t)(left_pwm));
  //Encoder_1.setTarPWM((int16_t)(right_pwm));
  //Encoder_2.setTarPWM((int16_t)(left_pwm));

  //
  Encoder_1.loop();
  Encoder_2.loop();

  //Serial.write("pwm: ");
  //Serial.print(right_pwm);
  //Serial.write(" ");
  //Serial.write("rad/s: ");
  //Serial.print(Encoder_1.getCurrentSpeed());
  //Serial.write("\n");

  return;
}

void setVelocityRobot(double linear_vel, double angular_vel)
{
  double ang_speed_m1 = ( (2*p1)/(K1*R*(p1 - p2)) ) * linear_vel + ( (2*p1*p2)/(K1*R*(p1 - p2)) ) * angular_vel;
  double ang_speed_m2 = ( -(2*p2)/(K2*R*(p1 - p2)) ) * linear_vel + ( -(2*p1*p2)/(K2*R*(p1 - p2)) ) * angular_vel;

  //Serial.write("m1: ");
  //Serial.print(ang_speed_m1);
  //Serial.write(" ");
  //Serial.write("m2: ");
  //Serial.print(ang_speed_m2);
  //Serial.write("\n");

  setAngSpeedMotors(ang_speed_m1, ang_speed_m2);

  return;
}

void setVelocityPercRobot(double linear_vel_per, double angular_vel_per)
{
  //Serial.write("linear_vel_per: ");
  //Serial.print(linear_vel_per);
  //Serial.write(" ");
  //Serial.write("angular_vel_per: ");
  //Serial.print(angular_vel_per);
  //Serial.write("\n");
  
  double linear_vel = linear_vel_per/100*max_linear_vel;
  double angular_vel = angular_vel_per/100*max_angular_vel;

  //Serial.write("linear_vel: ");
  //Serial.print(linear_vel);
  //Serial.write(" ");
  //Serial.write("angular_vel: ");
  //Serial.print(angular_vel);
  //Serial.write("\n");

  setVelocityRobot(linear_vel, angular_vel);

  return;
}

int8_t readRfidTag(uint8_t team_code) 
{
  // Check if RFID reader enabled
  if (!rfid_reader_enabled) 
  {
    return -1;
  }
  // Read Passive target ID
  success = rfid_reader.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100);
  if (!success) 
  {
    return -2;
  }
  // Analyze block
  rfid_reader.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 1, key);
  rfid_reader.mifareclassic_ReadDataBlock(4, data);
  //
  int8_t rfid_value = (data[0] & 0x0F);
  if (rfid_value == team_code) 
  {
    return 1;
  } 
  else
  {
    return 0;
  }
  return -10;
}

void operateGripper(int8_t cmd_open, int8_t cmd_close)
{
  int16_t cmd_pwm=0;
  if (cmd_open && cmd_close) 
  {
    cmd_pwm=0;
  } 
  else if (cmd_open) 
  {
    cmd_pwm=-255;
  } 
  else if (cmd_close) 
  {
    cmd_pwm=255;
  } 
  else 
  {
    cmd_pwm=0;
  }
  dc.run(cmd_pwm);

  return;
}

void operateLifter(int8_t cmd_up, int8_t cmd_down)
{
  int16_t cmd_pwm=0;
  if (cmd_up && cmd_down) 
  {
    cmd_pwm=0;
  } 
  else if (cmd_up) 
  {
    cmd_pwm=-255;
  } 
  else if (cmd_down) 
  {
    cmd_pwm=255;
  }
  else 
  {
    cmd_pwm=0;
  }
  lifter.setMotorPwm(cmd_pwm);
  //lifter.setTarPWM(cmd_pwm);

  lifter.loop();
  
  return;
}

void isrProcessEncoder1(void) 
{
  if(digitalRead(Encoder_1.getPortB()) == 0) 
  {
    Encoder_1.pulsePosMinus();
  } 
  else 
  {
    Encoder_1.pulsePosPlus();
  }
}

void isrProcessEncoder2(void) 
{
  if(digitalRead(Encoder_2.getPortB()) == 0) 
  {
    Encoder_2.pulsePosMinus();
  } 
  else 
  {
    Encoder_2.pulsePosPlus();
  }
}

void isrProcessLifter(void) 
{
  if(digitalRead(lifter.getPortB()) == 0) 
  {
    lifter.pulsePosMinus();
  } 
  else 
  {
    lifter.pulsePosPlus();
  }
}

void updateLocation()
{
  // Info: https://robotics.stackexchange.com/questions/106/what-is-a-suitable-model-for-two-wheeled-robots
  
  // Orientation of robot
  angle_motor1_cur = radians(Encoder_1.getCurPos());
  angle_motor2_cur = radians(Encoder_2.getCurPos());

  double delta_angle = -( K1*R/(2*p1)*(angle_motor1_cur-angle_motor1_pre))-K2*R/(2*p2)*(angle_motor2_cur-angle_motor2_pre);
  angle_robot_world += delta_angle;

  //Serial.write("Angle: ");
  //Serial.print(degrees(angle_robot_world));
  //Serial.write(", ");
  //Serial.print(degrees(delta_angle));
  //Serial.write(", LA: ");
  //Serial.print(degrees(angle_motor2_cur));
  //Serial.write(", RA: ");
  //Serial.print(degrees(angle_motor1_cur));
  //Serial.write("\n");

  // Position of the robot
  locX += (0.5 * (K1*R * (angle_motor1_cur-angle_motor1_pre) + K2*R * (angle_motor2_cur-angle_motor2_pre))) * cos(angle_robot_world);
  locX += (0.5 * (K1*R * (angle_motor1_cur-angle_motor1_pre) + K2*R * (angle_motor2_cur-angle_motor2_pre))) * sin(angle_robot_world);

  // Update for the nex iteration
  angle_motor1_pre = angle_motor1_cur;
  angle_motor2_pre = angle_motor2_cur;

  return;
}

void printLocalizationEstimation()
{
  Serial3.write("X (cm): ");
  Serial3.print(locX*100);
  Serial3.write(", Y (cm): ");
  Serial3.print(locY*100);
  Serial3.write(", Ang (deg): ");
  Serial3.print(degrees(angle_robot_world));
  Serial3.write(", LAngle: ");
  Serial3.print(degrees(angle_motor2_cur));
  Serial3.write(", RAngle: ");
  Serial3.print(degrees(angle_motor1_cur));
  Serial3.write("\n");

  return;
}

// TODO - IMPROVE!
int8_t checkHomeBase()
{
  return 0;
  
  // Read measurement line following sensor
  int line_following_sensor_meas = line_following_sensor.readSensors();


  // Print localization estimation
  printLocalizationEstimation();

  
  switch (line_following_sensor_meas) 
  {
    case S1_IN_S2_OUT:
    case S1_OUT_S2_IN:
    case S1_OUT_S2_OUT:
      // Not at home base
      return 0;
      break;
    case S1_IN_S2_IN:
      // At some home base
      Serial3.write("Left: BLACK      Right: BLACK\n"); 
      break;
  }

  return 0;
}

int8_t chksum(int8_t *arr)
{
  int8_t s = 0;
  for (uint8_t i = 0; i < 10; i++) {
    s -= (*(arr + i) - 1);
  }
  return s;
}

void sendUptime()
{
  uint32_t t = millis();
  uint32_t s = (t / 1000) % 60;
  uint32_t m = ((t / 1000) / 60) % 60;
  Serial3.write((m > 9) ? ((m / 10) % 10) + '0' : '0');
  Serial3.write((m % 10) + '0');
  Serial3.write('m');
  Serial3.write((s > 9) ? ((s / 10) % 10) + '0' : '0');
  Serial3.write((s % 10) + '0');
  Serial3.write('s');

  return;
}

void onPressRfidCheck()
{
  int8_t rfid_read_result=readRfidTag(TEAMCODE);
  switch(rfid_read_result)
  {
    case -1:
      Serial3.write("$bRFID: No RFID reader present\n");
      break;
    case -2:
      Serial3.write("$bRFID: No RFID tag present\n");
      break;
    case 0:
      Serial3.write("$bRFID: Not our tag\n");
      break;
    case 1:
      Serial3.write("$gRFID: Our tag, GO!\n");
      break;
    default:
      Serial3.write("$bRFID: RFID unknown error\n");
      break;
  }
  
  return;
}

void onPressRobotCheck()
{
  Serial3.write("Status:\n");
  // Up time
  Serial3.write("$gMegaPi running, uptime ");
  sendUptime();
  Serial3.write(", ");
  // RFID reader
  if (rfid_reader_enabled) 
  {
    Serial3.write("RFID OK\n");
  } 
  else 
  {
    Serial3.write("RFID not initialized\n");
  }

  return;
}

// TODO - IMPROVE!
void onPressHomeBaseCheck()
{
  // Read sensor
  int line_following_sensor_meas = line_following_sensor.readSensors();
  // Print line following sensor measurement
  switch (line_following_sensor_meas) 
  {
    case S1_IN_S2_IN: Serial3.write("Left: BLACK      Right: BLACK\n"); break;
    case S1_IN_S2_OUT: Serial3.write("Left: BLACK      Right: WHITE\n"); break;
    case S1_OUT_S2_IN: Serial3.write("Left: WHITE      Right: BLACK\n"); break;
    case S1_OUT_S2_OUT: Serial3.write("Left: WHITE      Right: WHITE\n"); break;
  }
  
  // Print localization estimation
  printLocalizationEstimation();

  return;
}

void dispatch(uint8_t a)
{
  switch (a) 
  {
    // RFID Read
    case 4:
    {
      onPressRfidCheck();
      break;
    }
    // Home base check
    case 5: 
    {
      onPressHomeBaseCheck();
      break;
    }
    // TODO
    case 6:
      Serial3.write("$nCalibration: Not implemented yet\n");
      break;
    // Robot Status
    case 7:
      onPressRobotCheck();
      break;
  }

  return;
}

// setup() function
void setup()
{
  // Communications - Serial
  Serial.begin(115200);
  
  // Communications - Bluetooth
  Serial3.begin(115200);
  while(!Serial3.available())
  {
    delay(100);
  }

  // Message warning
  Serial3.write("$bROBOT: Initializing robot. Please wait!\n");

  // LED for communication feedback
  pinMode(LED_BUILTIN, OUTPUT);

  // RFID reader
  while(!rfid_reader_enabled)
  {
    rfid_reader.begin();
    if (!rfid_reader.getFirmwareVersion()) 
    {
      rfid_reader_enabled = 0;
      Serial3.write("$bRFID: No RFID reader present\n");
    } 
    else 
    {
      rfid_reader_enabled = 1;
      rfid_reader.SAMConfig();
      Serial3.write("$gRFID: RFID reader configured\n");
    }
  }

  // Motor 1 - Right wheel
  Encoder_1.setMotionMode(DIRECT_MODE);
  //Encoder_1.setMotionMode(PWM_MODE);
  attachInterrupt(Encoder_1.getIntNum(), isrProcessEncoder1, RISING);

  // Motor 2 - Left wheel
  Encoder_2.setMotionMode(DIRECT_MODE);
  //Encoder_2.setMotionMode(PWM_MODE);
  attachInterrupt(Encoder_2.getIntNum(), isrProcessEncoder2, RISING);

  // Motor 3 - Lifter
  lifter.setMotionMode(DIRECT_MODE);
  //lifter.setMotionMode(PWM_MODE);
  attachInterrupt(lifter.getIntNum(), isrProcessLifter, RISING);

  // RFID read time thread initialization
  rfid_read_time_prev=millis();

  // Home detection time thread initialization
  home_detection_time_prev=millis();

  // Message warning
  Serial3.write("$gROBOT: Robot initialized!\n");

  return;
}

// loop() function
void loop() 
{
  // Communication - Bluetooth
  if (Serial3.available())
  {
    int8_t r = Serial3.read();
    if (r == syncChar)
    {
      dCount = 0;
    } 
    else
    {
      next[dCount++] = r;
      if (dCount == 11)
      {
        if (chksum(next) == next[10])
        {
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          for (uint8_t i = 0; i < 10; i++)
          {
            if (!next[i] && disp[i]) 
            {
              disp[i] = 0;
            }
            rec[i] = next[i];
          }
        }
      }
    }
  }
  
  // Lifter
  operateLifter(rec[2], rec[3]);
  
  // Gripper
  operateGripper(rec[0], rec[1]);
  
  // Buttons with one single execution per press
  for (uint8_t i = 4; i <= 7; i++) 
  {
    // Only one execution per press
    if (rec[i] && !disp[i]) 
    {
      disp[i] = 1;
      dispatch(i);
    }
  }
  
  // Cmd robot velocity: linear and angular
  double cmd_linear_vel_per = sliderEasyControl((double)(rec[9]), -10, 10, -100, -100, 100, 100);
  double cmd_angular_vel_per = sliderEasyControl((double)(rec[8]), -10, 10, -100, -100, 100, 100);  
  setVelocityPercRobot(cmd_linear_vel_per, cmd_angular_vel_per);

  // Robot localization estimation
  updateLocation();

  // RFID read tag. At a certain frequency
  unsigned long int rfid_read_time_curr=millis();
  if(rfid_read_time_curr-rfid_read_time_prev > rfid_read_time_period_ms)
  {
    //
    //Serial3.write("Time: ");
    //Serial3.print(millis());
    //Serial3.write("\n");
    
    // Read rfid
    int8_t rfid_read_result=readRfidTag(TEAMCODE);
    switch(rfid_read_result)
    {
      case 0:
        if(flag_new_rfid_result == true)
        {
          Serial3.write("$bRFID: Not our tag\n");
          flag_new_rfid_result = false;
        }
        break;
      case 1:
        if(flag_new_rfid_result == true)
        {
          Serial3.write("$gRFID: Our tag, GO!\n");
          flag_new_rfid_result = false;
        }
        break;
      case -2:
        flag_new_rfid_result = true;
        break;
      case -1:
      default:
        break;
    }
    
    // Update time
    rfid_read_time_prev=rfid_read_time_curr;
  }

  // Home base detection. At a certain frequency
  // TODO
  unsigned long int home_detection_time_curr=millis();
  if(home_detection_time_curr-home_detection_time_prev > home_detection_time_period_ms)
  {
    //
    //Serial3.write("Time: ");
    //Serial3.print(millis());
    //Serial3.write("\n");

    // TODO
    checkHomeBase();

    // Update time
    home_detection_time_prev=home_detection_time_curr;
  }

  // Sleep
  delay(1);

  return;
}
