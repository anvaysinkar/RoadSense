#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

int  mpu_init(void);              /* 0 = OK; 100 Hz, DLPF 44 Hz, +-4 g, +-500 dps */
int  mpu_whoami(void);            /* register 0x75, -1 on bus error */
int  mpu_read(int16_t v[6]);      /* ax ay az gx gy gz counts; 1 = OK */
uint32_t mpu_err_count(void);

#endif
