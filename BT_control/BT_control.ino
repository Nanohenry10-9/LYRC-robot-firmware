#include <Wire.h>
#include <SPI.h>

#include <MeMegaPi.h>

#include <Adafruit_PN532.h>


const double K1 = 1, K2 = -1;
const double R = 0.0333, p1 = 0.085, p2 = -0.085;
const double P = 0.085;

const double max_linear_vel=0.5; // m/s
const double max_angular_vel=5.0; // rad/s

const double RADS_TO_PWM = 255.0 / 2.10;


Adafruit_PN532 nfc(22);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard lifter(SLOT3);
MeMegaPiDCMotor dc(PORT4B);
MeLineFollower lf(PORT_5);

double angle = 0.0, locX = 0.0, locY = 0.0, a1 = 0.0, a2 = 0.0, s1 = 0.0, s2 = 0.0;

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


uint8_t success, uid[6], uidLength;
uint8_t data[16], key[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t TEAMCODE = 1;

int8_t ourTag() 
{
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100);
  if (!success) {
    return -1;
  }
  nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 1, key);
  nfc.mifareclassic_ReadDataBlock(4, data);
  return (data[0] & 0x0F) == TEAMCODE;
}

bool rfid = 1;

void isr_process_encoder1(void) {
  if(digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void) {
  if(digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
  }
}

void isr_process_encoder3(void) {
  if(digitalRead(lifter.getPortB()) == 0) {
    lifter.pulsePosMinus();
  } else {
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

uint8_t dCount = 0, syncChar = 'e';
int8_t rec[10] =  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t disp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t next[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void dispatch(uint8_t a)
{
  switch (a) 
  {
    // RFID Read
    case 4:
      if (!rfid) 
      {
        Serial3.write("$bRFID: No RFID reader present\n");
      } 
      else 
      {
        int8_t t = ourTag();
        if (t == -1) 
        {
          Serial3.write("$bRFID: No tag present\n");
        } 
        else if (t == 0) 
        {
          Serial3.write("$bRFID: Not our tag\n");
        } 
        else if (t == 1) 
        {
          Serial3.write("$gRFID: Our tag, GO!\n");
        }
      }
      break;
    case 5: {
        int ld = lf.readSensors();
        switch (ld) {
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
      }
      break;
    case 6:
      Serial3.write("$nCalibration: Not implemented yet\n");
      break;
    case 7:
      Serial3.write("Status:\n");
      Serial3.write("$gMegaPi running, uptime ");
      sendUptime();
      Serial3.write(", ");
      if (rfid) 
      {
        Serial3.write("RFID OK\n");
      } 
      else 
      {
        Serial3.write("RFID not initialized\n");
      }
      break;
  }
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
}

// setup() function
void setup()
{

  Encoder_1.setMotionMode(PWM_MODE);
  Encoder_2.setMotionMode(PWM_MODE);
  
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(lifter.getIntNum(), isr_process_encoder3, RISING);
  
  Serial.begin(115200);
  Serial3.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  nfc.begin();
  if (!nfc.getFirmwareVersion()) 
  {
    rfid = 0;
    //while (1);
  } 
  else 
  {
    nfc.SAMConfig();
  }
}

// loop() function
void loop() 
{
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
  if (rec[2] && rec[3]) {
    lifter.setMotorPwm(0);
  } else if (rec[2]) {
    lifter.setMotorPwm(-255);
  } else if (rec[3]) {
    lifter.setMotorPwm(255);
  } else {
    lifter.setMotorPwm(0);
  }
  //
  lifter.loop();
  
  // Grip
  if (rec[0] && rec[1]) {
    dc.run(0);
  } else if (rec[0]) {
    dc.run(-255);
  } else if (rec[1]) {
    dc.run(255);
  } else {
    dc.run(0);
  }
  
  //
  for (uint8_t i = 4; i <= 7; i++) 
  {
    if (rec[i] && !disp[i]) {
      disp[i] = 1;
      dispatch(i);
    }
  }
  
  // Cmd robot velocity: linear and angular
  double cmd_linear_vel_per = sliderEasyControl((double)(rec[9]), -10, 10, -100, -100, 100, 100);
  double cmd_angular_vel_per = sliderEasyControl((double)(rec[8]), -10, 10, -100, -100, 100, 100);
  setVelocityPercRobot(cmd_linear_vel_per, cmd_angular_vel_per);

  //
  updateLocation();
  delay(1);

  return;
}
