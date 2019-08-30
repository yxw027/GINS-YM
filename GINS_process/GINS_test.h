#ifndef _GINS_TEST_H
#define _GINS_TESTH
#include <string.h>
#include "GINS_process.h"
#include "GINS_process_lib.h"

#define MAXVAL		50 // max value for spilting
#define MAXLEN		1024
#define IMU_BW_HZ                  40
#define ADI465 0
#define IMU381 0
#define AP100_I90 0
#define GD100 1
/*--------------------------------IMU WALK-----------------------------*/
/*TUpdate(,1)*/
#if ADI465
#define GYRO_WALK (0.3*0.1)
#define VEL_WALK (1200)
#endif
#if IMU381
#define GYRO_WALK (0.3*0.05)
#define VEL_WALK (5000)
#endif
#if AP100_I90
#define GYRO_WALK (0.3*0.1)
#define VEL_WALK (7300)
#endif
#if GD100
#define GYRO_WALK (0.3*0.05)
#define VEL_WALK (3000)
#endif
/*--------------------------------IMU STD-----------------------------*/
#define GYRO_STD_X (0.011*sqrt(IMU_BW_HZ))
#define ACC_STD_X (0.000150*sqrt(IMU_BW_HZ))

#define GYRO_STD_Y GYRO_STD_X
#define GYRO_STD_Z GYRO_STD_X

#define ACC_STD_Y ACC_STD_X
#define ACC_STD_Z ACC_STD_X


/*--------------------------------IMU AXIS-----------------------------*/
#if ADI465
#define CFG_GYRO_X_SCALE  (1)
#define CFG_GYRO_Y_SCALE (1)
#define CFG_GYRO_Z_SCALE  (-1)
#define CFG_ACC_X_SCALE  (1)
#define CFG_ACC_Y_SCALE   (1)
#define CFG_ACC_Z_SCALE  (-1)
#endif
#if IMU381
#define CFG_GYRO_X_SCALE  (-1)
#define CFG_GYRO_Y_SCALE (1)
#define CFG_GYRO_Z_SCALE  (1)
#define CFG_ACC_X_SCALE  (-1)
#define CFG_ACC_Y_SCALE   (1)
#define CFG_ACC_Z_SCALE  (1)
#endif
#if AP100_I90
#define CFG_GYRO_X_SCALE  (1)
#define CFG_GYRO_Y_SCALE (1)
#define CFG_GYRO_Z_SCALE  (-1)
#define CFG_ACC_X_SCALE  (1)
#define CFG_ACC_Y_SCALE   (1)
#define CFG_ACC_Z_SCALE  (-1)
#endif
#if GD100
#define CFG_GYRO_X_SCALE  (-1)
#define CFG_GYRO_Y_SCALE (1)
#define CFG_GYRO_Z_SCALE  (1)
#define CFG_ACC_X_SCALE  (-1)
#define CFG_ACC_Y_SCALE   (1)
#define CFG_ACC_Z_SCALE  (1)
#endif

#define CFG_GYRO_X_ROW 2
#define CFG_GYRO_Y_ROW 1
#define CFG_GYRO_Z_ROW 3
#define CFG_ACC_X_ROW 5
#define CFG_ACC_Y_ROW 4
#define CFG_ACC_Z_ROW 6



#endif