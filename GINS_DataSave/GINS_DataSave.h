#ifndef _GINS_DATASAVE_H
#define _GINS_DATASAVE_H
#include <string.h>
#include "GINS_process.h"
#include "GINS_process_lib.h"
#define gilc_printf printf
#define TEST_RAW_FILE_PATH "E:/P2ÏîÄ¿/CGI410/5.25/410/075533_raw.txt"
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
