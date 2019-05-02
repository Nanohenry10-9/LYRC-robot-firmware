#include <MeMegaPi.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

Adafruit_PN532 nfc(22);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard lifter(SLOT3);
MeMegaPiDCMotor dc(PORT4B);

void setSpeed(int l, int r) {
  Encoder_1.setMotorPwm(r);
  Encoder_2.setMotorPwm(-l);
}

void setLifter(int s) {
  lifter.setMotorPwm(s);
}

uint8_t success, uid[6], uidLength;

uint8_t TEAMCODE = 1;

boolean ourTag() {
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  if (!success) {
    return 0;
  }
  uint8_t data[16];
  uint8_t key[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 1, key);
  nfc.mifareclassic_ReadDataBlock(4, data);
  return ((data[0] & 0xF0) >> 4) == TEAMCODE;
}

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  nfc.begin();
  if (!nfc.getFirmwareVersion()) {
    //while (1);
  }
  nfc.SAMConfig();
}

long dCount = 0;

int8_t syncChar = 'e';

int8_t rec[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void loop() {
  if (Serial3.available()) {
    digitalWrite(LED_BUILTIN, 1);
    int8_t r = Serial3.read();
    if (r == syncChar) {
      dCount = 0;
    } else {
      rec[dCount++] = r;
    }
    digitalWrite(LED_BUILTIN, 0);
  }
  if (rec[2] && rec[3]) {
    setLifter(0);
  } else if (rec[3]) {
    setLifter(255);
  } else if (rec[2]) {
    setLifter(-255);
  } else {
    setLifter(0);
  }
  if (rec[0] && rec[1]) {
    dc.run(0);
  } else if (rec[0]) {
    dc.run(-100);
  } else if (rec[1]) {
    dc.run(100);
  } else {
    dc.run(0);
  }
  if (rec[4]) {
    Serial3.print("LMAO test");
  }
  setSpeed((rec[6] / 100.0) * 255.0, (rec[7] / 100.0) * 255.0);
}
