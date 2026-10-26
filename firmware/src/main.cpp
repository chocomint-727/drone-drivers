#include <Arduino.h>

#undef radians
#undef degrees

#include <SPI.h>
#include <Wire.h>
#include "quad_state_model/quad_state_model.hpp"
#include "attitude_estimator/attitude_estimator.hpp"
#include "drone_protocol.hpp"

using glm::ivec3; using glm::fquat;

const int ACCEL_ADDR = 0x53; 
const int GYRO_ADDR = 0x68;
const int MAGN_ADDR = 0x2C;
const int ACCEL_START = 0x32;
const int GYRO_START = 0x1D;
const int MAGN_START = 0x01;
const float ACCEL_SCALE = 0.0383;
const float GYRO_SCALE = (1.0/14.375) * (PI / 180.0);
const float MAGN_SCALE = 100.0f / 3750.0f;
const int SOFT_SDA = 12;
const int SOFT_SCL = 13;

uint8_t buffer[6];
ivec3 rawAccel;
ivec3 rawGyro;
ivec3 rawMagn;
int16_t rawTemp;
vec3 scaledAccel;
vec3 scaledGyro;
vec3 scaledMagn;

uint32_t lastMicros = 0;
float dt;

const glm::fquat identity = glm::fquat(1.0f, 0.0f, 0.0f, 0.0f);
AttitudeEstimator* filter;
QuadModel* drone;

vec3 a_bias;
vec3 g_bias;
vec3 m_bias;

void calibrateIMU(int times);

uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)addr, (size_t)1, true);
  return Wire.read();
}

void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SOFT_SDA, SOFT_SCL); 

  // accel
  writeReg(ACCEL_ADDR, 0x2C, 0x0A);
  writeReg(ACCEL_ADDR, 0x2D, 0x08);
  writeReg(ACCEL_ADDR, 0x31, 0x0B);

  // gyro 
  writeReg(GYRO_ADDR, 0x3E, 0x01);
  writeReg(GYRO_ADDR, 0x16, 0x1B);
  writeReg(GYRO_ADDR, 0x15, 0x04);

  // --- magnetometer (QMC5883P @ 0x2C) ---
  writeReg(MAGN_ADDR, 0x0B, 0x80);   // soft reset -> suspend mode, default registers
  delay(5);
  writeReg(MAGN_ADDR, 0x29, 0x06);   // axis signs (every datasheet example writes this first)
  writeReg(MAGN_ADDR, 0x0B, 0x08);   // ±8 G, set/reset on
  writeReg(MAGN_ADDR, 0x0A, 0xC3);   // normal mode, 200 Hz, OSR1=8, OSR2=8

  rawAccel = ivec3(0);
  rawGyro = ivec3(0);
  rawMagn = ivec3(0);

  a_bias = vec3();
  g_bias = vec3();
  m_bias = vec3();
  Serial.println("Initialized");

  calibrateIMU(10000);
  Serial.printf("Acc bias: (%f, %f, %f)\n", a_bias.x, a_bias.y, a_bias.z);
  Serial.printf("Gyro bias: (%f, %f, %f)\n", g_bias.x, g_bias.y, g_bias.z);
  Serial.printf("Magn bias: (%f, %f, %f)\n", m_bias.x, m_bias.y, m_bias.z);

  filter = new AttitudeEstimator();
  drone = new QuadModel(0.5, identity, mat3(1.0), mat4(1.0));
  delay(5000);
  lastMicros = micros();
}

void loop() {
  // put your main code here, to run repeatedly:

  Wire.beginTransmission(ACCEL_ADDR);
  Wire.write(ACCEL_START); 
  uint8_t err = Wire.endTransmission(true); 
  size_t got = Wire.requestFrom((uint16_t)ACCEL_ADDR, (size_t)6, true);

  if (got == 6) {
    for (int i = 0; i < 6; i++){
      buffer[i] = Wire.read();
    }
    rawAccel.x  = (int16_t)((buffer[1]  << 8) | buffer[0]);
    rawAccel.y  = (int16_t)((buffer[3]  << 8) | buffer[2]);
    rawAccel.z  = (int16_t)((buffer[5]  << 8) | buffer[4]);
  }

  // Serial.printf("requestFrom acc got=%u\n", (unsigned)got);

  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(GYRO_START); 
  err = Wire.endTransmission(true); 
  got = Wire.requestFrom((uint16_t)GYRO_ADDR, (size_t)6, true);

  if (got == 6) {
    for (int i = 0; i < 6; i++){
      buffer[i] = Wire.read();
    }
    rawGyro.x  = (int16_t)((buffer[0]  << 8) | buffer[1]);
    rawGyro.y  = (int16_t)((buffer[2]  << 8) | buffer[3]);
    rawGyro.z  = (int16_t)((buffer[4]  << 8) | buffer[5]);
  }

  // Serial.printf("requestFrom gyro got=%u\n", (unsigned)got);

  Wire.beginTransmission(MAGN_ADDR);
  Wire.write(MAGN_START); 
  err = Wire.endTransmission(false); 
  got = Wire.requestFrom((uint16_t)MAGN_ADDR, (size_t)6, true);

  if (got == 6) {
    for (int i = 0; i < 6; i++){
      buffer[i] = Wire.read();
    }
    rawMagn.x  = (int16_t)((buffer[1]  << 8) | buffer[0]);
    rawMagn.y  = (int16_t)((buffer[3]  << 8) | buffer[2]);
    rawMagn.z  = (int16_t)((buffer[5]  << 8) | buffer[4]);
  }
  
  // Serial.printf("requestFrom mag got=%u\n", (unsigned)got);

  
  scaledAccel.x = rawAccel.x * ACCEL_SCALE;
  scaledAccel.y = rawAccel.y * ACCEL_SCALE;
  scaledAccel.z = rawAccel.z * ACCEL_SCALE;
  
  scaledGyro.x = rawGyro.x * GYRO_SCALE;
  scaledGyro.y = rawGyro.y * GYRO_SCALE;
  scaledGyro.z = rawGyro.z * GYRO_SCALE;

  scaledMagn.x = rawMagn.x * MAGN_SCALE;
  scaledMagn.y = rawMagn.y * MAGN_SCALE;
  scaledMagn.z = rawMagn.z * MAGN_SCALE;

  uint32_t now = micros();
  dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  scaledGyro = scaledGyro - g_bias;
  scaledAccel = scaledAccel - a_bias;
  drone->attitude = filter->madgwickIMU(drone->attitude, scaledGyro, scaledAccel, dt, 0.15);
  
  AttitudePacket pkt;
  pkt.start1 = 0xAA;
  pkt.start2 = 0x55;
  pkt.w = drone->attitude.w;
  pkt.x = drone->attitude.x;
  pkt.y = drone->attitude.y;
  pkt.z = drone->attitude.z;
  Serial.write((uint8_t*)&pkt, sizeof(pkt));
  Serial.flush();

  // Serial.printf("accel: %f %f %f\n", scaledAccel.x, scaledAccel.y, scaledAccel.z);
  // Serial.printf("gyro: %f %f %f\n", scaledGyro.x, scaledGyro.y, scaledGyro.z);
  // Serial.printf("magn: %f %f %f\n", scaledMagn.x, scaledMagn.y, scaledMagn.z);
  // Serial.printf("quaternion: %f %f %f %f\n", filter->attitude.w, filter->attitude.x, filter->attitude.y, filter->attitude.z);

  // delay(10);
}

void calibrateIMU(int times){

  for (int i = 1; i <= times; i++){
    Wire.beginTransmission(ACCEL_ADDR);
    Wire.write(0x32); 
    uint8_t err = Wire.endTransmission(true); 
    size_t got = Wire.requestFrom((uint16_t)ACCEL_ADDR, (size_t)6, true);

    if (got == 6) {
      for (int i = 0; i < 6; i++){
        buffer[i] = Wire.read();
      }
      rawAccel.x  = (int16_t)((buffer[1]  << 8) | buffer[0]);
      rawAccel.y  = (int16_t)((buffer[3]  << 8) | buffer[2]);
      rawAccel.z  = (int16_t)((buffer[5]  << 8) | buffer[4]);
    }

    Wire.beginTransmission(GYRO_ADDR);
    Wire.write(0x1D); 
    err = Wire.endTransmission(true); 
    got = Wire.requestFrom((uint16_t)GYRO_ADDR, (size_t)6, true);

    if (got == 6) {
      for (int i = 0; i < 6; i++){
        buffer[i] = Wire.read();
      }
      rawGyro.x  = (int16_t)((buffer[0]  << 8) | buffer[1]);
      rawGyro.y  = (int16_t)((buffer[2]  << 8) | buffer[3]);
      rawGyro.z  = (int16_t)((buffer[4]  << 8) | buffer[5]);
    }
    
    // Convert to actual physical units
    a_bias.x += rawAccel.x * ACCEL_SCALE;
    a_bias.y += rawAccel.y * ACCEL_SCALE;
    a_bias.z += rawAccel.z * ACCEL_SCALE - 9.80665;
    
    g_bias.x += (rawGyro.x * GYRO_SCALE);
    g_bias.y += (rawGyro.y * GYRO_SCALE);
    g_bias.z += (rawGyro.z * GYRO_SCALE);
  }

  a_bias = a_bias / (float)times;
  g_bias = g_bias / (float)times;
}