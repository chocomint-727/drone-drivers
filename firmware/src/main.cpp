#include <Arduino.h>

#undef radians
#undef degrees

#include <SPI.h>
#include <Wire.h>
#include "quad_state_model/quad_state_model.hpp"
#include "drone_protocol.hpp"

using glm::ivec3; using glm::fquat;

const int ACCEL_ADDR = 0x53; 
const int GYRO_ADDR = 0x68;
const int MAGN_ADDR = 0x1E;
const int ACCEL_START = 0x32;
const int GYRO_START = 0x1D;
const int MAGN_START = 0x03;
const float ACCEL_SCALE = 0.0383;
const float GYRO_SCALE = (1.0/14.375) * (PI / 180.0);
const float MAGN_SCALE = 10.0/109.0;
const int SDA = 33;
const int SCL = 35;

const float ACCEL_SCALE = 16384.0; // For ±2g range
const float GYRO_SCALE = 16.4; // 16.4; // 32.8; // 65.5; // 131.0;

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
QuadModel* drone;

vec3 a_bias;
vec3 g_bias;
vec3 m_bias;

void calibrateIMU(int times);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(SDA, SCL); 

  // accel
  Wire.beginTransmission(ACCEL_ADDR);
  Wire.write(0x2D); 
  Wire.write(0x08);    
  Wire.endTransmission(true);

  Wire.beginTransmission(ACCEL_ADDR);
  Wire.write(0x31); 
  Wire.write(0x0B);    
  Wire.endTransmission(true);

  // gyro 
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x3E);
  Wire.write(0x01); 
  Wire.endTransmission(true);

  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x16);
  Wire.write(0x18); 
  Wire.endTransmission(true);

  // magnetometer 
  Wire.beginTransmission(MAGN_ADDR);
  Wire.write(0x00);
  Wire.write(0b01110100); 
  Wire.endTransmission(true);

  Wire.beginTransmission(MAGN_ADDR);
  Wire.write(0x01);
  Wire.write(0b00100000); 
  Wire.endTransmission(true);

  Wire.beginTransmission(MAGN_ADDR);
  Wire.write(0x02);
  Wire.write(0x00); 
  Wire.endTransmission(true);

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

  drone = new QuadModel(0.5, identity, mat3(1.0), mat4(1.0));
  delay(5000);
  lastMicros = micros();
}

void loop() {
  // put your main code here, to run repeatedly:

  Wire.beginTransmission(ACCEL_ADDR);
  Wire.write(0x32); 
  uint8_t err = Wire.endTransmission(false); 
  size_t got = Wire.requestFrom((uint16_t)ACCEL_ADDR, (size_t)6, true);

  if (got == 6) {
    for (int i = 0; i++; i < 6){
      buffer[i] = Wire.read();
    }
    rawAccel.x  = (int16_t)((buffer[1]  << 8) | buffer[0]);
    rawAccel.y  = (int16_t)((buffer[3]  << 8) | buffer[2]);
    rawAccel.z  = (int16_t)((buffer[5]  << 8) | buffer[4]);
  }

  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x1D); 
  uint8_t err = Wire.endTransmission(false); 
  got = Wire.requestFrom((uint16_t)GYRO_ADDR, (size_t)6, true);

  if (got == 6) {
    for (int i = 0; i++; i < 6){
      buffer[i] = Wire.read();
    }
    rawGyro.x  = (int16_t)((buffer[0]  << 8) | buffer[1]);
    rawGyro.y  = (int16_t)((buffer[2]  << 8) | buffer[3]);
    rawGyro.z  = (int16_t)((buffer[4]  << 8) | buffer[5]);
  }

  Wire.beginTransmission(MAGN_ADDR);
  Wire.write(0x03); 
  uint8_t err = Wire.endTransmission(false); 
  got = Wire.requestFrom((uint16_t)MAGN_ADDR, (size_t)6, true);

  if (got == 6) {
    for (int i = 0; i++; i < 6){
      buffer[i] = Wire.read();
    }
    rawMagn.x  = (int16_t)((buffer[0]  << 8) | buffer[1]);
    rawMagn.y  = (int16_t)((buffer[2]  << 8) | buffer[3]);
    rawMagn.z  = (int16_t)((buffer[4]  << 8) | buffer[5]);
  }
  
  Serial.printf("requestFrom got=%u\n", (unsigned)got);

  
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
  drone->updateAttitude(scaledGyro, scaledAccel, scaledMagn, dt, 0.1, 0.1);
  
  AttitudePacket pkt;
  pkt.start1 = 0xAA;
  pkt.start2 = 0x55;
  pkt.w = drone->attitude.w;
  pkt.x = drone->attitude.x;
  pkt.y = drone->attitude.y;
  pkt.z = drone->attitude.z;
  Serial.write((uint8_t*)&pkt, sizeof(pkt));
  Serial.flush();
  

  delay(10);
}

void calibrateIMU(int times){

  for (int i = 1; i <= times; i++){
    Wire.beginTransmission(ACCEL_ADDR);
    Wire.write(0x32); 
    uint8_t err = Wire.endTransmission(false); 
    size_t got = Wire.requestFrom((uint16_t)ACCEL_ADDR, (size_t)6, true);

    if (got == 6) {
      for (int i = 0; i++; i < 6){
        buffer[i] = Wire.read();
      }
      rawAccel.x  = (int16_t)((buffer[1]  << 8) | buffer[0]);
      rawAccel.y  = (int16_t)((buffer[3]  << 8) | buffer[2]);
      rawAccel.z  = (int16_t)((buffer[5]  << 8) | buffer[4]);
    }

    Wire.beginTransmission(GYRO_ADDR);
    Wire.write(0x1D); 
    uint8_t err = Wire.endTransmission(false); 
    got = Wire.requestFrom((uint16_t)GYRO_ADDR, (size_t)6, true);

    if (got == 6) {
      for (int i = 0; i++; i < 6){
        buffer[i] = Wire.read();
      }
      rawGyro.x  = (int16_t)((buffer[0]  << 8) | buffer[1]);
      rawGyro.y  = (int16_t)((buffer[2]  << 8) | buffer[3]);
      rawGyro.z  = (int16_t)((buffer[4]  << 8) | buffer[5]);
    }
    
    // Convert to actual physical units
    a_bias.x += rawAccel.x * ACCEL_SCALE;
    a_bias.y += rawAccel.y * ACCEL_SCALE;
    a_bias.z += rawAccel.z * ACCEL_SCALE - 1;
    
    g_bias.x += (rawGyro.x * GYRO_SCALE);
    g_bias.y += (rawGyro.y * GYRO_SCALE);
    g_bias.z += (rawGyro.z * GYRO_SCALE);
  }

  a_bias = a_bias / (float)times;
  g_bias = g_bias / (float)times;
}