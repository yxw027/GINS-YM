#include "IMU_Filter.h"
void IMU_Filter(Process_Data ilcd)
{
	double imu_raw[6];
	double imu_iir[6];
	Mat_equal(ilcd.acc, 3, 1, &imu_raw[0]);
	Mat_equal(ilcd.gyo, 3, 1, &imu_raw[3]);
	IIR_Lowpass(imu_iir, imu_raw);
	Mat_equal(&imu_iir[0], 3, 1, ilcd.acc);
	Mat_equal(&imu_iir[3], 3, 1, ilcd.gyo);
}

void IIR_Lowpass(double *dOut, double *dIn)
{
	IIR_Lowpass_Oder3(dOut, dIn, dIIR_Order3_bw_fs_40, 6);
	//IIR_Lowpass_Oder5(dOut, dIn, dIIR_Order5_bw_fs_3, 6);
	//IIR_Lowpass_Oder10(dOut, dIn, dIIR_Order10_bw_fs_1_5, 6);
}


int bFirstRun = 1;
double iir_y[MAX_CHAN][IIR_NUM], iir_x[MAX_CHAN][IIR_NUM];
void IIR_Lowpass_Oder3(double *dOut, double *dIn, double *dIIR, short nChan)
{
	int    i, j;
	double dTmp;

	if (bFirstRun)
	{
		bFirstRun = 0;

		for (i = 0; i < nChan; i++)
		{
			for (j = 0; j < IIR_NUM; j++) 
				iir_x[i][j] = iir_y[i][j] = 0;
		}
	}

	for (i = 0; i < nChan; i++)
	{
		iir_x[i][3] = iir_x[i][2];
		iir_x[i][2] = iir_x[i][1];
		iir_x[i][1] = iir_x[i][0];
		iir_x[i][0] = dIn[i];

		dTmp = dIIR[0] * iir_x[i][0] + dIIR[1] * iir_x[i][1] + dIIR[2] * iir_x[i][2] + dIIR[3] * iir_x[i][3];
		dTmp -= dIIR[5] * iir_y[i][1] + dIIR[6] * iir_y[i][2] + dIIR[7] * iir_y[i][3];

		iir_y[i][0] = dTmp;
		iir_y[i][3] = iir_y[i][2];
		iir_y[i][2] = iir_y[i][1];
		iir_y[i][1] = iir_y[i][0];

		dOut[i] = (double)dTmp;
	}
}