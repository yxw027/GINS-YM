#include  "GINS_test.h"
typedef struct imu_data {
	int    iAccel[3]; /*x,y,z*/
	int    iGyro[3];  /*x,y,z*/
	int    iMag[3];   /*x,y,z*/
	int    iTemp;
	double dAccel[3];                     //accelerometer  X-Y-Z output (m/s2)
	double dGyro[3];                      //gyroscope X-Y-Z output(rad/s)
	double dMag[3];                       //magnetometer
	double dTemp;
}imu_data_t;

/*天线杆臂默认值*/
#ifndef CFG_fIns2GnssVector_X 
#define CFG_fIns2GnssVector_X 0
#endif
#ifndef CFG_fIns2GnssVector_Y 
#define CFG_fIns2GnssVector_Y 0
#endif
#ifndef CFG_fIns2GnssVector_Z 
#define CFG_fIns2GnssVector_Z 0
#endif
#ifndef CFG_fIns2GnssVectorErr_X 
#define CFG_fIns2GnssVectorErr_X 1
#endif
#ifndef CFG_fIns2GnssVectorErr_Y 
#define CFG_fIns2GnssVectorErr_Y 1
#endif
#ifndef CFG_fIns2GnssVectorErr_Z 
#define CFG_fIns2GnssVectorErr_Z 1
#endif
/*天线基线夹角默认值*/
#ifndef CFG_fIns2GnssAngle_X 
#define CFG_fIns2GnssAngle_X 0
#endif
#ifndef CFG_fIns2GnssAngle_Y 
#define CFG_fIns2GnssAngle_Y 0
#endif
#ifndef CFG_fIns2GnssAngle_Z 
#define CFG_fIns2GnssAngle_Z 0
#endif
#ifndef CFG_fIns2GnssAngleErr_X 
#define CFG_fIns2GnssAngleErr_X 5
#endif
#ifndef CFG_fIns2GnssAngleErr_Y 
#define CFG_fIns2GnssAngleErr_Y 5
#endif
#ifndef CFG_fIns2GnssAngleErr_Z 
#define CFG_fIns2GnssAngleErr_Z 5
#endif
/*INS安装误差默认值*/
#ifndef CFG_fIns2BodyAngle_X 
#define CFG_fIns2BodyAngle_X 0
#endif
#ifndef CFG_fIns2BodyAngle_Y 
#define CFG_fIns2BodyAngle_Y 0
#endif
#ifndef CFG_fIns2BodyAngle_Z 
#define CFG_fIns2BodyAngle_Z 0
#endif
#ifndef CFG_fIns2BodyAngleErr_X 
#define CFG_fIns2BodyAngleErr_X 5
#endif
#ifndef CFG_fIns2BodyAngleErr_Y 
#define CFG_fIns2BodyAngleErr_Y 5
#endif
#ifndef CFG_fIns2BodyAngleErr_Z 
#define CFG_fIns2BodyAngleErr_Z 5
#endif

#ifndef CFG_WORK_MODE 
#define CFG_WORK_MODE GILC_WORK_MODE__CAR_NORMAL
#endif


int main()
{
	int iRet = 0;
	GINS_raw_t stRaw_test = { 0 }, stRawTmp_test = { 0 };
	GINS_result_t stOut = { 0 };
	GINS_cfg_t stCfg = { 0 };
	FILE *fd_data = NULL;
	FILE *fd_result = NULL;
	char buff[1024];
	char outpath[1024] = { 0 };
	char tmppath[1024] = { 0 };
	sprintf(tmppath, "%s", TEST_RAW_FILE_PATH);
	stCfg.gyro_std[0] = GYRO_STD_X;
	stCfg.gyro_std[1] = GYRO_STD_Y;
	stCfg.gyro_std[2] = GYRO_STD_Z;
	stCfg.accle_std[0] = ACC_STD_X;
	stCfg.accle_std[1] = ACC_STD_Y;
	stCfg.accle_std[2] = ACC_STD_Z;
	stCfg.bStdCfgUse = true;

	stCfg.gyro_walk[0] = stCfg.gyro_walk[1] = stCfg.gyro_walk[2] = GYRO_WALK;
	stCfg.vel_walk[0] = stCfg.vel_walk[1] = stCfg.vel_walk[2] = VEL_WALK;
	stCfg.bWalkCfgUse = true;

	/*天线杆臂*/
	stCfg.fIns2GnssVector[0] = CFG_fIns2GnssVector_X;
	stCfg.fIns2GnssVector[1] = CFG_fIns2GnssVector_Y;
	stCfg.fIns2GnssVector[2] = CFG_fIns2GnssVector_Z;
	stCfg.fIns2GnssVectorErr[0] = CFG_fIns2GnssVectorErr_X;
	stCfg.fIns2GnssVectorErr[1] = CFG_fIns2GnssVectorErr_Y;
	stCfg.fIns2GnssVectorErr[2] = CFG_fIns2GnssVectorErr_Z;
	/*天线基线夹角*/
	stCfg.fIns2GnssAngle[0] = CFG_fIns2GnssAngle_X;
	stCfg.fIns2GnssAngle[1] = CFG_fIns2GnssAngle_Y;
	stCfg.fIns2GnssAngle[2] = CFG_fIns2GnssAngle_Z;
	stCfg.fIns2GnssAngleErr[0] = CFG_fIns2GnssAngleErr_X;
	stCfg.fIns2GnssAngleErr[1] = CFG_fIns2GnssAngleErr_Y;
	stCfg.fIns2GnssAngleErr[2] = CFG_fIns2GnssAngleErr_Z;
	/*INS安装夹角*/
	stCfg.fIns2BodyAngle[0] = CFG_fIns2BodyAngle_X;
	stCfg.fIns2BodyAngle[1] = CFG_fIns2BodyAngle_Y;
	stCfg.fIns2BodyAngle[2] = CFG_fIns2BodyAngle_Z;
	stCfg.fIns2BodyAngleErr[0] = CFG_fIns2BodyAngleErr_X;
	stCfg.fIns2BodyAngleErr[1] = CFG_fIns2BodyAngleErr_Y;
	stCfg.fIns2BodyAngleErr[2] = CFG_fIns2BodyAngleErr_Z;
	GINS_YM GINS_test_ym;
	GINS_test_ym.GINS_Init(&stCfg);


	fd_data = fopen(tmppath, "rt");
	if (!fd_data)
	{
		printf("open file err! %s\r\n", tmppath);
#ifdef WIN32
		system("pause");
#endif
		return 0;
	}

	static unsigned long long llu_once = 0, llu_ave = 0, llu_total = 0, llu_cnt = 0;
	while (1)
	{

		if (feof(fd_data))
		{
			printf("file read over!\n");
			if (fd_data)
				fclose(fd_data);
			break;
		}

		fgets(buff, 1024, fd_data);
		if (strlen(buff) < 18)
		{
			printf("file read error: %s\n", buff);
			continue;
		}
		int decode_flag = GINS_string_decode(buff, &stRawTmp_test);
		if (decode_flag == -1)
		{
			printf("GILC_Load param err!\r\n");
			continue;
		}
		else if (decode_flag == -2)
		{
			printf("GILC_Load fail, unknow strning: %s\n", buff);
			continue;
		}


		if (stRawTmp_test.bGPSavail)
		{
			stRaw_test.bGPSavail = stRawTmp_test.bGPSavail;
			stRaw_test.gnssdata = stRawTmp_test.gnssdata;
			stRaw_test.gnss_delay_ms = stRawTmp_test.gnss_delay_ms;
		}

		if (stRawTmp_test.bMEMSavail)
		{
			stRaw_test.bMEMSavail = stRawTmp_test.bMEMSavail;
			stRaw_test.bPPSavail = stRawTmp_test.bPPSavail;
			stRaw_test.imutimetarget = stRawTmp_test.imutimetarget;
			stRaw_test.memsdate = stRawTmp_test.memsdate;
		}

		static double start_time = 0.0;
		if ((stRaw_test.gnssdata.second > start_time + 1 * 60 && stRaw_test.gnssdata.second < start_time + 2 * 60))
		{
			//stRaw.bGPSavail = false;
			//stRaw.bODOavail = false;
		}
		//printf("data:%f,%d\n", stRaw_test.gnssdata.second, stRaw_test.bGPSavail);
		memset(&stOut, 0, sizeof(stOut));
		int GINS_flag = GINS_test_ym.GINS_PROCESS_Lib(&stRaw_test, &stOut);
		stRaw_test.bGPSavail = 0;
		stRaw_test.bMEMSavail = 0;
		stRaw_test.bPPSavail = 0;
	}
	return 0;
}

int GINS_string_decode(char *buff,GINS_raw_t *pRaw)
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