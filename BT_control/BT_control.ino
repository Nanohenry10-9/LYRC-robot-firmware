#include <Wire.h>
#include <SPI.h>

#include <MeMegaPi.h>

#include <Adafruit_PN532.h>

// Robot physical parameters
const double K1 = 1, K2 = -1;
const double R = 0.0333, p1 = 0.085, p2 = -0.085;
const double P = 0.085; // TODO remove

// Robot movement parameters
const double max_linear_vel=0.5; // m/s
const double max_angular_vel=5.0; // rad/s

// Const
const double RADS_TO_PWM = 255.0 / 2.10;

// Team code
const uint8_t TEAMCODE = 1;

// Localization related variables
double a1 = 0.0, a2 = 0.0, s1 = 0.0, s2 = 0.0;
double angle = 0.0, locX = 0.0, locY = 0.0;

// Communication related variables
uint8_t dCount = 0, syncChar = 'e';
int8_t rec[10] =  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t disp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t next[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// RFID thread related variables
unsigned long int rfid_read_time_prev;
bool flag_new_rfid_result = false;
unsigned long int rfid_read_time_period_ms=500; // ms

// RFID required read variables
uint8_t success, uid[6], uidLength;
uint8_t data[16];
uint8_t key[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


// RFID reader
Adafruit_PN532 rfid_reader(22);
bool rfid_reader_enabled = 0;

// Line Following sensor
MeLineFollower lf(PORT_5);

// Motor with encoder 1 - movement
MeEncoderOnBoard Encoder_1(SLOT1);

// Motor with encoder 2 - movement
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
  double left_pwm = RADS_TO_PWM*ang_speed_m2;
  double right_pwm = RADS_TO_PWM*ang_speed_m1;

  // PWM = [-255, 255]
  right_pwm = saturateVal(right_pwm, -255, 255);
  left_pwm = saturateVal(left_pwm, -255, 255);
  
  //
  Encoder_1.setMotorPwm((int16_t)(right_pwm));
  Encoder_2.setMotorPwm((int16_t)(left_pwm));

  //
  Encoder_1.loop();
  Encoder_2.loop();

  return;
}

void setVelocityRobot(double linear_vel, double angular_vel)
{
  //Serial3.write("linear_vel: ");
  //Serial3.print(linear_vel);
  //Serial3.write(" ");
  //Serial3.write("angular_vel: ");
  //Serial3.print(angular_vel);
  //Serial3.write("\n");

  double ang_speed_m1 = ( (2*p1)/(K1*R*(p1 - p2)) ) * linear_vel + ( -(p1*p2)/(K1*R*(p1 - p2)) ) * angular_vel;
  double ang_speed_m2 = ( -(2*p2)/(K2*R*(p1 - p2)) ) * linear_vel + ( (p1*p2)/(K2*R*(p1 - p2)) ) * angular_vel;

  setAngSpeedMotors(ang_speed_m1, ang_speed_m2);

  return;
}

void setVelocityPercRobot(double linear_vel_per, double angular_vel_per)
{
  //Serial3.write("linear_vel_per: ");
  //Serial3.print(linear_vel_per);
  //Serial3.write(" ");
  //Serial3.write("angular_vel_per: ");
  //Serial3.print(angular_vel_per);
  //Serial3.write("\n");
  
  double linear_vel = linear_vel_per/100*max_linear_vel;
  double angular_vel = angular_vel_per/100*max_angular_vel;

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
  if (cmd_open && cmd_close) 
  {
    dc.run(0);
  } 
  else if (cmd_open) 
  {
    dc.run(-255);
  } 
  else if (cmd_close) 
  {
    dc.run(255);
  } 
  else 
  {
    dc.run(0);
  }

  return;
}

void operateLifter(int8_t cmd_up, int8_t cmd_down)
{
  if (cmd_up && cmd_down) 
  {
    lifter.setMotorPwm(0);
  } 
  else if (cmd_up) 
  {
    lifter.setMotorPwm(-255);
  } 
  else if (cmd_down) 
  {
    lifter.setMotorPwm(255);
  }
  else 
  {
    lifter.setMotorPwm(0);
  }
  //
  lifter.loop();

  return;
}

void isr_process_encoder1(void) 
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

void isr_process_encoder2(void) 
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

void isr_process_encoder3(void) 
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
  a1 = radians(Encoder_1.getCurPos()) * K1;
  a2 = radians(Encoder_2.getCurPos()) * K2;
  s1 = radians(Encoder_1.getCurrentSpeed()) * K1;
  s2 = radians(Encoder_2.getCurrentSpeed()) * K2;
  angle = 0.5 * (((R * a1) / P) - ((R * a2) / P));
  locX += (0.5 * (R * s1 + R * s2)) * cos(angle);
  locY += (0.5 * (R * s1 + R * s2)) * sin(angle);
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
      int ld = lf.readSensors();
      switch (ld) 
      {
        case S1_IN_S2_IN: Serial3.write("Left: BLACK      Right: BLACK\n"); break;
        case S1_IN_S2_OUT: Serial3.write("Left: BLACK      Right: WHITE\n"); break;
        case S1_OUT_S2_IN: Serial3.write("Left: WHITE      Right: BLACK\n"); break;
        case S1_OUT_S2_OUT: Serial3.write("Left: WHITE      Right: WHITE\n"); break;
      }
      Serial3.write("X: ");
      Serial3.print(locX*100);
      Serial3.write("      Y: ");
      Serial3.print(locY*100);
      Serial3.write("      Angle: ");
      Serial3.print(degrees(angle));
      Serial3.write("      LAngle: ");
      Serial3.print(degrees(a2));
      Serial3.write("      RAngle: ");
      Serial3.print(degrees(a1));
      Serial3.write("\n");
    
      break;
    }
    //
    case 6:
      Serial3.write("$nCalibration: Not implemented yet\n");
      break;
    // Status
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

  // LED for communication feedback
  pinMode(LED_BUILTIN, OUTPUT);

  // Motor 1 - Wheel
  Encoder_1.setMotionMode(PWM_MODE);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);

  // Motor 2 - Wheel
  Encoder_2.setMotionMode(PWM_MODE);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

  // Motor 3 - Lifter
  lifter.setMotionMode(PWM_MODE);
  attachInterrupt(lifter.getIntNum(), isr_process_encoder3, RISING);

  // RFID reader
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

  // RFID read time initialization
  rfid_read_time_prev=millis();

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

  //
  updateLocation();

  // Sleep
  delay(1);

  return;
}
