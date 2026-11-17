/*
 * mpu6050.c -- MPU6050 on I2C1 (PB6 = SCL, PB7 = SDA), address 0x68 (AD0 low).
 * Same register setup as the ESP32 firmware.
 */
#include "mpu6050.h"
#include "i2c.h"
#include "rs_core.h"
#include <stdio.h>

#define MPU_ADDR   (0x68 << 1)
#define I2C_TO_MS  5

static int errStreak = 0;
static uint32_t errTotal = 0, lastMsgMs = 0;

static HAL_StatusTypeDef wr(uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TO_MS);
}

int mpu_init(void) {
  int e = 0;
  e |= wr(0x6B, 0x01) != HAL_OK;   /* wake, PLL with X gyro clock */
  HAL_Delay(10);
  e |= wr(0x19, 9) != HAL_OK;      /* sample rate 1 kHz / (1+9) = 100 Hz */
  e |= wr(0x1A, 3) != HAL_OK;      /* DLPF ~44 Hz */
  e |= wr(0x1B, 0x08) != HAL_OK;   /* gyro +-500 dps */
  e |= wr(0x1C, 0x08) != HAL_OK;   /* accel +-4 g */
  return e;
}

int mpu_whoami(void) {
  uint8_t who = 0;
  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x75, I2C_MEMADD_SIZE_8BIT, &who, 1, I2C_TO_MS) != HAL_OK) return -1;
  return who;
}

uint32_t mpu_err_count(void) { return errTotal; }

static void note_err(void) {
  char l[64];
  errStreak++; errTotal++;
  if (HAL_GetTick() - lastMsgMs > 1000) {
    snprintf(l, sizeof l, "#I2C_ERR hal_err=0x%lx streak=%d", (unsigned long)HAL_I2C_GetError(&hi2c1), errStreak);
    rs_port_write(l); lastMsgMs = HAL_GetTick();
  }
  if (errStreak == 20) {           /* try to recover the bus */
    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(5);
    MX_I2C1_Init();
    mpu_init();
    rs_port_write("#I2C bus re-initialised");
  }
}

int mpu_read(int16_t v[6]) {
  uint8_t b[14];
  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, b, 14, I2C_TO_MS) != HAL_OK) {
    note_err();
    return 0;
  }
  v[0] = (int16_t)(b[0] << 8 | b[1]);  v[1] = (int16_t)(b[2] << 8 | b[3]);
  v[2] = (int16_t)(b[4] << 8 | b[5]);
  v[3] = (int16_t)(b[8] << 8 | b[9]);  v[4] = (int16_t)(b[10] << 8 | b[11]);
  v[5] = (int16_t)(b[12] << 8 | b[13]);
  errStreak = 0;
  return 1;
}
