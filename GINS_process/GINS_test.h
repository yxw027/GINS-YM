#ifndef _GINS_TEST_H
#define _GINS_TESTH
#include <string.h>
#include "GINS_process.h"
#include "GINS_process_lib.h"
int GINS_string_decode(char *buff, GINS_raw_t *pRaw);
int Str2Array(const char *str, const char *sep, double *val);
#define MAXVAL		50 // max value for spilting
#define MAXLEN		1024
#define IMU_BW_HZ                  40
/*--------------------------------IMU WALK-----------------------------*/
/*TUpdate(,1)*/
#define GYRO_WALK (0.3*0.05)
#define VEL_WALK (5000)

/*--------------------------------IMU STD-----------------------------*/
#define GYRO_STD_X (0.011*sqrt(IMU_BW_HZ))
#define ACC_STD_X (0.000150*sqrt(IMU_BW_HZ))

#define GYRO_STD_Y GYRO_STD_X
#define GYRO_STD_Z GYRO_STD_X

#define ACC_STD_Y ACC_STD_X
#define ACC_STD_Z ACC_STD_X


/*--------------------------------IMU AXIS-----------------------------*/
#define CFG_GYRO_X_SCALE  (1)
#define CFG_GYRO_Y_SCALE (1)
#define CFG_GYRO_Z_SCALE  (-1)
#define CFG_ACC_X_SCALE  (1)
#define CFG_ACC_Y_SCALE   (1)
#define CFG_ACC_Z_SCALE  (-1)

#define CFG_GYRO_X_ROW 2
#define CFG_GYRO_Y_ROW 1
#define CFG_GYRO_Z_ROW 3
#define CFG_ACC_X_ROW 5
#define CFG_ACC_Y_ROW 4
#define CFG_ACC_Z_ROW 6

#define TEST_RAW_FILE_PATH "E:/P2ÏîÄ¿/CGI410/5.25/610/075429_raw.txt"
#define OUT_OUT_FILE_PATH "E:/GINS_YM_TEST/"
#define OUT_FILE_process  "process.txt"
#define OUT_FILE_raw  "raw.txt"
#define OUT_FILE_kf  "kf.txt"
#define OUT_FILE_rtkplot  "rtkplot.txt"
void File_open(void);
void SaveRst(GINS_Process *gipro, Process_Data *ilcd);
void printf_posLC(FILE *fp, GINS_INS *ins, Process_Data *ilcd);
void printf_kf(FILE *fp, GINS_KF& kf, double gpstimetarget);
void printf_prosess(FILE *fp, GINS_Process *gipro, Process_Data *ilcd);
void printf_posAnt(FILE *fp, GINS_INS *ins, Process_Data *ilcd);
#endif