#ifndef _GINS_FILTER_H
#define _GINS_FILTER_H
#include "GINS_process.h"
#include "GINS_process_lib.h"

void IIR_Lowpass(double *dOut, double *dIn);
void IIR_Lowpass_Oder3(double *dOut, double *dIn, double *dIIR, short nChan);
#define  IIR_NUM	12
#define  MAX_CHAN	6
static double dIIR_Order3_bw_fs_40[] = {
	0.52762438250194321,
	1.5828731475058295,
	1.5828731475058295,
	0.52762438250194321	,
	1,
	1.7600418803431686,
	1.1828932620378305,
	0.27805991763454646
};
#endif