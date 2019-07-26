#include "GINS_process_lib.h"
#include "GINS_process.h"
#include "GINS_test.h"
#include "GINS_DataSave.h"
#include <string.h>
#include <stdio.h>
int numins = 0;
FILE *fp_process ;
FILE *fp_kf = NULL;
FILE *fp_raw = NULL;
FILE *fp_rtkplot = NULL;
FILE *s_flog = NULL;
char OUT_PRO[1024] = { 0 };
char OUT_KF[1024] = { 0 };
char OUT_RAW[1024] = { 0 };
char OUT_RTK[1024] = { 0 };
char OUT_log[1024] = { 0 };
void File_open(void)
{
	sprintf(OUT_PRO, "%s%s", OUT_FILE_PATH, OUT_FILE_process);
	fp_process = fopen(OUT_PRO, "wt");
	if (!fp_process)
		printf("open file err! %s\r\n", *fp_process);

	sprintf(OUT_KF, "%s%s", OUT_FILE_PATH, OUT_FILE_kf);
	fp_kf = fopen(OUT_KF, "wt");
	if (!fp_kf)
		printf("open file err! %s\r\n", *fp_kf);

	sprintf(OUT_RAW, "%s%s", OUT_FILE_PATH, OUT_FILE_raw);
	fp_raw = fopen(OUT_RAW, "wt");
	if (!fp_raw)
		printf("open file err! %s\r\n", *fp_raw);

	sprintf(OUT_RTK, "%s%s", OUT_FILE_PATH, OUT_FILE_rtkplot);
	fp_rtkplot = fopen(OUT_RTK, "wt");
	if (!fp_rtkplot)
		printf("open file err! %s\r\n", *fp_rtkplot);


	sprintf(OUT_log, "%s%s", OUT_FILE_PATH, OUT_FILE_log);
	s_flog = fopen(OUT_log, "wt");
	if (!s_flog)
		printf("open file err! %s\r\n", *s_flog);
}
void SaveRst(GINS_Process *gipro, Process_Data *ilcd)
{
	//if (!File_flag)
	//{
	//	File_open();
	//	File_flag = true;
	//}

	if (ilcd->bMEMSavail)
	{
		numins++;
		if (numins == 10) /*10Hz*/
		{
			printf_posLC(fp_process, &gipro->ins, ilcd);
			printf_kf(fp_kf, gipro->kf, ilcd->imutimetarge);
			printf_posAnt(fp_rtkplot, &gipro->ins, ilcd);
			numins = 0;
		}
	}

	if (ilcd->bMEMSavail)
	{
		printf_prosess(fp_raw, gipro, ilcd);
	}
}

void printf_posLC(FILE *fp, GINS_INS *ins, Process_Data *ilcd)
{
	if (!fp) return;
	double ep[6] = { 0 };
	gtime_t gt = gpst2time(ilcd->week, ilcd->imutimetarge - 1); /*RTKlib plot 默认-17秒*/
	time2epoch(gt, ep);

	fprintf(fp, "%4d/%02d/%02d %02d:%02d:%06.3f %15.9f %15.9f %11.4f %4d %4d\n",
		int(ep[0]), int(ep[1]), int(ep[2]), int(ep[3]), int(ep[4]), ep[5], ins->pos[0] * R2D, ins->pos[1] * R2D, ins->pos[2], ilcd->stat, 10);
}

void printf_kf(FILE *fp, GINS_KF& kf, double gpstimetarget)
{
	if (!fp) return;
	fprintf(fp, "%f ", gpstimetarget);
	for (int k = 0; k<kf.ROW; k++)	fprintf(fp, "%19.12lf ", sqrt(kf.Pxk[k*kf.ROW + k])); //状态权对角线元素
	fprintf(fp, "%s", "\n");
}

void printf_prosess(FILE *fp, GINS_Process *gipro, Process_Data *ilcd)
{
	if (!fp) return;
	double heading_ant = atan2(gipro->ins.vnL[0], gipro->ins.vnL[1])*R2D;
	if (heading_ant < 0) heading_ant += 360;
	double heading_gnss = atan2(ilcd->vn[0], ilcd->vn[1])*R2D;
	if (heading_gnss < 0) heading_gnss += 360;

	double gnss_speed = sqrt(pow(ilcd->vn[0], 2) + pow(ilcd->vn[1], 2));
	double ins_speed = gipro->ins.vm_car[1];
	double dUtcTime = (ilcd->ep[3] * 10000 + ilcd->ep[4] * 100 + ilcd->ep[5]);

	fprintf(fp, "%9.3f,%9.3f,%d,%d,%d,%.9f,%.9f,%.9f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.9f,%.9f,%.9f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f ",
		dUtcTime, ilcd->imutimetarge,
		gipro->c, ilcd->nsused, ilcd->stat,
		gipro->ins.pos[0] * R2D, gipro->ins.pos[1] * R2D, gipro->ins.pos[2],
		gipro->ins.vn[0], gipro->ins.vn[1], gipro->ins.vn[2],
		gipro->ins.att[0] * R2D, gipro->ins.att[1] * R2D, gipro->ins.att[2] * R2D,
		ilcd->pos[0] * R2D, ilcd->pos[1] * R2D, ilcd->pos[2],
		ilcd->vn[0], ilcd->vn[1], ilcd->vn[2],
		ilcd->acc[0], ilcd->acc[1], ilcd->acc[2],
		ilcd->gyo[0], ilcd->gyo[1], ilcd->gyo[2]
		);
	fprintf(fp, "%.6f %.6f %.6f %.6f %.6f %.6f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.3f %.3f %d %d %.3f %.3f %.3f \n",
		ilcd->acc_iir[0], ilcd->acc_iir[1], ilcd->acc_iir[2],
		ilcd->gyo_iir[0], ilcd->gyo_iir[1], ilcd->gyo_iir[2],
		gipro->ins.eb[0], gipro->ins.eb[1], gipro->ins.eb[2],
		gipro->ins.db[0], gipro->ins.db[1], gipro->ins.db[2],
		gipro->ins.PRY_Install[0] * R2D,
		gipro->ins.PRY_Install[1] * R2D,
		gipro->ins.PRY_Install[2] * R2D,
		gipro->ins.lever[0],
		gipro->ins.lever[1],
		gipro->ins.lever[2],
		gnss_speed, ins_speed,
		gipro->bStatic, gipro->iDriverMode,
		heading_gnss, heading_ant, ilcd->heading2
		);
}

void printf_posAnt(FILE *fp, GINS_INS *ins, Process_Data *ilcd)
{
	if (!fp) return;
	double ep[6] = { 0 };
	gtime_t gt = gpst2time(ilcd->week, ilcd->imutimetarge - 1);
	time2epoch(gt, ep);
	fprintf(fp, "%4d/%02d/%02d %02d:%02d:%06.3f %15.9f %15.9f %11.4f %4d %4d\n",
		int(ep[0]), int(ep[1]), int(ep[2]), int(ep[3]), int(ep[4]), ep[5], ins->pos[0] * R2D, ins->pos[1] * R2D, ins->pos[2], ilcd->stat, 10);
}


void GINS_log(char* fmt, ...)
{
	char buf[2048] = { 0 };
	va_list ap;
	va_start(ap, fmt);
	vsprintf((char*)buf, fmt, ap);
	va_end(ap);
	fprintf_log(buf);
	gilc_printf(buf);
}

void fprintf_log(char* msg)
{
	if (!s_flog)
		return;
	fprintf(s_flog, msg);
}


int GINS_string_decode(char *buff, GINS_raw_t *pRaw)
{
	double val[MAXVAL] = { 0.0 };
	if (!buff || strlen(buff) > MAXLEN)
	{
		//return GILC_RET__ERR_STRN_PARAM;
	}

	pRaw->bMEMSavail = false;
	pRaw->bGPSavail = false;
	pRaw->bPPSavail = false;
	if (strstr(buff, "GNSS,") || strstr(buff, "GNSS:"))
	{
		int num = Str2Array(buff + 5, ",", val);
		if (num == 20)
		{
			memset(&pRaw->gnssdata, 0, sizeof(pRaw->gnssdata));
			pRaw->gnssdata.second = val[0];
			pRaw->gnssdata.week = (int)val[1];
			pRaw->gnssdata.lat = val[2];
			pRaw->gnssdata.lon = val[3];
			pRaw->gnssdata.alt = val[4];
			pRaw->gnssdata.vx_ecef = val[5];
			pRaw->gnssdata.vy_ecef = val[6];
			pRaw->gnssdata.vz_ecef = val[7];
			pRaw->gnssdata.std_lat = val[8];
			pRaw->gnssdata.std_lon = val[9];
			pRaw->gnssdata.std_alt = val[10];
			pRaw->gnssdata.std_vx_ecef = val[11];
			pRaw->gnssdata.std_vy_ecef = val[12];
			pRaw->gnssdata.std_vz_ecef = val[13];
			pRaw->gnssdata.stat = (int)val[14];
			pRaw->gnssdata.nsused = (int)val[15];
			pRaw->gnssdata.hdop = val[16];
			pRaw->gnssdata.age = val[17];
			pRaw->gnssdata.heading2 = val[20];
		}
		else if (num == 24)
		{
			memset(&pRaw->gnssdata, 0, sizeof(pRaw->gnssdata));
			pRaw->gnssdata.second = val[0];
			pRaw->gnssdata.week = (int)val[1];
			pRaw->gnssdata.lat = val[2];
			pRaw->gnssdata.lon = val[3];
			pRaw->gnssdata.alt = val[4];
			pRaw->gnssdata.vx_ecef = val[5];
			pRaw->gnssdata.vy_ecef = val[6];
			pRaw->gnssdata.vz_ecef = val[7];
			pRaw->gnssdata.std_lat = val[8];
			pRaw->gnssdata.std_lon = val[9];
			pRaw->gnssdata.std_alt = val[10];
			pRaw->gnssdata.std_vx_ecef = val[11];
			pRaw->gnssdata.std_vy_ecef = val[12];
			pRaw->gnssdata.std_vz_ecef = val[13];
			pRaw->gnssdata.pos_type = (int)val[14];
			pRaw->gnssdata.nsused = (int)val[15];
			pRaw->gnssdata.hdop = val[16];
			pRaw->gnssdata.age = val[17];
			pRaw->gnssdata.heading2 = val[18];
			pRaw->gnssdata.ns = (int)val[19];
			pRaw->gnssdata.snsum = (int)val[20];
			pRaw->gnssdata.pdop = val[21];
			pRaw->gnssdata.vdop = val[22];
			pRaw->gnssdata.std_heading2 = val[23];
		}
		else if (num == 35 || num == 40)
		{
			memset(&pRaw->gnssdata, 0, sizeof(pRaw->gnssdata));
			pRaw->gnssdata.second = val[0];
			pRaw->gnssdata.week = (int)val[1];
			pRaw->gnssdata.lat = val[2];
			pRaw->gnssdata.lon = val[3];
			pRaw->gnssdata.alt = val[4];
			pRaw->gnssdata.vx_ecef = val[5];
			pRaw->gnssdata.vy_ecef = val[6];
			pRaw->gnssdata.vz_ecef = val[7];
			pRaw->gnssdata.std_lat = val[8];
			pRaw->gnssdata.std_lon = val[9];
			pRaw->gnssdata.std_alt = val[10];
			pRaw->gnssdata.std_vx_ecef = val[11];
			pRaw->gnssdata.std_vy_ecef = val[12];
			pRaw->gnssdata.std_vz_ecef = val[13];
			pRaw->gnssdata.speed = val[14];
			pRaw->gnssdata.heading1 = val[15];
			pRaw->gnssdata.stat = (int)val[16];
			pRaw->gnssdata.age = val[17];
			pRaw->gnssdata.hdop = val[18];
			pRaw->gnssdata.pdop = val[19];
			pRaw->gnssdata.vdop = val[20];
			pRaw->gnssdata.tdop = val[21];
			pRaw->gnssdata.gdop = val[22];
			pRaw->gnssdata.ns = (int)val[23];
			pRaw->gnssdata.nsused = (int)val[24];
			pRaw->gnssdata.snsum = (int)val[25];
			pRaw->gnssdata.ns2 = (int)val[26];
			pRaw->gnssdata.nsused2 = (int)val[27];
			pRaw->gnssdata.heading2 = val[29];
			pRaw->gnssdata.std_heading2 = val[30];
			pRaw->gnssdata.pitch2 = val[31];
			pRaw->gnssdata.std_pitch2 = val[32];
			pRaw->gnssdata.pos_type = (int)val[33];
		}
		else
		{
			printf("Unknow gnss data num %d\r\n", num);
			return -2;
		}

		pRaw->bGPSavail = true;
		return 1;
	}
	else if (strstr(buff, "IMU,"))
	{
		int num = Str2Array(buff + 4, ",", val);
		if (num == 9 || num == 10 || num == 11)
		{
			memset(&pRaw->memsdate, 0, sizeof(pRaw->memsdate));
			pRaw->imutimetarget = val[0];
			pRaw->memsdate.alldata[0] = val[1];
			pRaw->memsdate.alldata[1] = val[2];
			pRaw->memsdate.alldata[2] = val[3];
			pRaw->memsdate.alldata[3] = val[4];
			pRaw->memsdate.alldata[4] = val[5];
			pRaw->memsdate.alldata[5] = val[6];
			pRaw->bPPSavail = (int)val[8];// 根据标志位判断PPS
		}
		else
		{
			printf("Unknow imu data num %d\r\n", num);
			return -2;
		}
		pRaw->bMEMSavail = true;
		pRaw->bGPSavail = false;
		return 1;
	}
}

int Str2Array(const char *str, const char *sep, double *val)
{
	char *p, _str[1024];
	double d[MAXVAL] = { 0.0 };
	int i, j;

	strcpy(_str, str);
	for (i = 0, p = strtok(_str, sep); p&&i<MAXVAL; p = strtok(NULL, sep), i++) {
		d[i] = atof(p);
	}

	for (j = 0; j<i; j++) val[j] = d[j];
	return i;
}