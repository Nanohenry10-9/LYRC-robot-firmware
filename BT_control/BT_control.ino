#include <MeMegaPi.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

Adafruit_PN532 nfc(22);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard lifter(SLOT3);
MeMegaPiDCMotor dc(PORT4B);
MeLineFollower lf(PORT_5);

void setSpeed(int16_t l, int16_t r) {
  Encoder_1.setMotorPwm(r);
  Encoder_2.setMotorPwm(-l);
}

uint8_t success, uid[6], uidLength;
uint8_t data[16], key[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t TEAMCODE = 1;

int8_t ourTag() {
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100);
  if (!success) {
    return -1;
  }
  nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 1, key);
  nfc.mifareclassic_ReadDataBlock(4, data);
  return (data[0] & 0x0F) == TEAMCODE;
}

bool rfid = 1;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  nfc.begin();
  if (!nfc.getFirmwareVersion()) {
    rfid = 0;
    //while (1);
  } else {
    nfc.SAMConfig();
  }
}

int8_t chksum(int8_t *arr) {
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

void loop() {
  if (Serial3.available()) {
    int8_t r = Serial3.read();
    if (r == syncChar) {
      dCount = 0;
    } else {
      next[dCount++] = r;
      if (dCount == 11) {
        if (chksum(next) == next[10]) {
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          for (uint8_t i = 0; i < 10; i++) {
            if (!next[i] && disp[i]) {
              disp[i] = 0;
            }
            rec[i] = next[i];
          }
        }
      }
    }
  }
  if (rec[2] && rec[3]) {
    lifter.setMotorPwm(0);
  } else if (rec[2]) {
    lifter.setMotorPwm(-255);
  } else if (rec[3]) {
    lifter.setMotorPwm(255);
  } else {
    lifter.setMotorPwm(0);
  }
  if (rec[0] && rec[1]) {
    dc.run(0);
  } else if (rec[0]) {
    dc.run(-255);
  } else if (rec[1]) {
    dc.run(255);
  } else {
    dc.run(0);
  }
  for (uint8_t i = 4; i <= 7; i++) {
    if (rec[i] && !disp[i]) {
      disp[i] = 1;
      dispatch(i);
    }
  }
  setSpeed((int16_t)(rec[8] * 2.25), (int16_t)(rec[9] * 2.25));
  delay(1);
}

void dispatch(uint8_t a) {
  switch (a) {
    case 4:
      if (!rfid) {
        Serial3.write("$bRFID: No RFID reader present\n");
      } else {
        int8_t t = ourTag();
        if (t == -1) {
          Serial3.write("$bRFID: No tag present\n");
        } else if (t == 0) {
          Serial3.write("$bRFID: Not our tag\n");
        } else if (t == 1) {
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
      if (rfid) {
        Serial3.write("RFID OK\n");
      } else {
        Serial3.write("RFID not initialized\n");
      }
      break;
  }
}

void sendUptime() {
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

