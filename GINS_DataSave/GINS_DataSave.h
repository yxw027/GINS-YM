#ifndef _GINS_DATASAVE_H
#define _GINS_DATASAVE_H
#include <string.h>
#include "GINS_process.h"
#include "GINS_process_lib.h"
#define gilc_printf printf
#if ADI465
#define TEST_RAW_FILE_PATH "E:/P2项目/CGI410/5.25/610/075429_raw.txt"
#endif
#if IMU381
#define TEST_RAW_FILE_PATH "E:/P2项目/CGI410/5.25/410/075533_raw.txt"
#endif
#if AP100_I90
#define TEST_RAW_FILE_PATH "E:/无人船项目/I90测试数据/071617_gilc_raw.txt"
#endif
#if GD100
#define TEST_RAW_FILE_PATH "G:/GD100项目/测试数据/8.13外场测试/data1/101207_raw.txt"
#endif


#define OUT_FILE_PATH "E:/GINS_YM_TEST/"
#define OUT_FILE_process  "process.txt"
#define OUT_FILE_raw  "raw.txt"
#define OUT_FILE_kf  "kf.txt"
#define OUT_FILE_rtkplot  "rtkplot.txt"
#define OUT_FILE_log  "gins_log.txt"

void File_open(void);
void GINS_log(char* fmt, ...);
void fprintf_log(char* msg);
int Str2Array(const char *str, const char *sep, double *val);

#endif
