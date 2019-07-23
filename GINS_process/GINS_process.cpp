#include "GINS_process.h"
#include "GINS_test.h"
/*GINS test by ym*/

int GINS_YM::GINS_data_correct(GINS_raw_t *pRaw)
{
	if (pRaw->bGPSavail)
	{
		GINS_GnssRaw_Correct(pRaw);
		stRaw_tmp.bGPSavail = pRaw->bGPSavail;
		stRaw_tmp.gnssdata = pRaw->gnssdata;
		stRaw_tmp.gnss_delay_ms = pRaw->gnss_delay_ms;
	}

	if (pRaw->bMEMSavail)
	{
		GINS_ImuAxis_Correct(pRaw);
		stRaw_tmp.bMEMSavail = pRaw->bMEMSavail;
		stRaw_tmp.memsdate = pRaw->memsdate;
		stRaw_tmp.imutimetarget = pRaw->imutimetarget;
	}

	if (pRaw->bPPSavail)
	{
		stRaw_tmp.bPPSavail = pRaw->bPPSavail;
	}
	return 1;
}


void GINS_YM::GINS_GnssRaw_Correct(GINS_raw_t *pRaw)
{
	static bool bGGAStatUsed = 0;
	if (!bGGAStatUsed && pRaw->gnssdata.stat) //GNSS GGA stat used check
		bGGAStatUsed = 1;

	if (!bGGAStatUsed)
	{
		switch (pRaw->gnssdata.pos_type)
		{
		case 16:
		case 20:
			pRaw->gnssdata.stat = 1;
			break;
		case 17:
		case 18:/*SBAS*/
			pRaw->gnssdata.stat = 2;
			break;
		case 19:
			pRaw->gnssdata.stat = 0;
			break;
		case 32:
		case 33:
		case 34:
			pRaw->gnssdata.stat = 5;
			break;
		case 48:
		case 49:
		case 50:
			pRaw->gnssdata.stat = 4;
			break;
		default:
			//gilc_log("libgilc --- raw gnss state error: week %d, sec %f, state %d\r\n",
				//pRaw->gnssdata.week, pRaw->gnssdata.second, pRaw->gnssdata.pos_type);
			pRaw->gnssdata.stat = 0;
			break;
		}
	}
	else
	{
		switch (pRaw->gnssdata.stat)
		{
		case 0:
		case 1:
		case 2:
		case 4:
		case 5:
		case 9:/*SBAS*/
			break;
		default:
			//gilc_log("libgilc --- raw gnss state error: week %d, sec %f, state %d\r\n",
				//pRaw->gnssdata.week, pRaw->gnssdata.second, pRaw->gnssdata.stat);
			pRaw->gnssdata.stat = 0;
			break;
		}
	}
}


void GINS_YM::GINS_ImuAxis_Correct(GINS_raw_t *pRaw)
{

		pRaw->memsdate.gyro[0] = pRaw->memsdate.alldata[CFG_GYRO_X_ROW - 1] * CFG_GYRO_X_SCALE;
		pRaw->memsdate.gyro[1] = pRaw->memsdate.alldata[CFG_GYRO_Y_ROW - 1] * CFG_GYRO_Y_SCALE;
		pRaw->memsdate.gyro[2] = pRaw->memsdate.alldata[CFG_GYRO_Z_ROW - 1] * CFG_GYRO_Z_SCALE;
		pRaw->memsdate.accel[0] = pRaw->memsdate.alldata[CFG_ACC_X_ROW - 1] * CFG_ACC_X_SCALE;
		pRaw->memsdate.accel[1] = pRaw->memsdate.alldata[CFG_ACC_Y_ROW - 1] * CFG_ACC_Y_SCALE;
		pRaw->memsdate.accel[2] = pRaw->memsdate.alldata[CFG_ACC_Z_ROW - 1] * CFG_ACC_Z_SCALE;
	//Mat_mul(dInstallMat, pRaw->memsdate.gyro, 3, 3, 1, pRaw->memsdate.gyro);
	//Mat_mul(dInstallMat, pRaw->memsdate.accel, 3, 3, 1, pRaw->memsdate.accel);
}



int  GINS_YM::GINS_Rawdata_Quality(GINS_raw_t *pRaw)
{
	if (!(pRaw->bMEMSavail || pRaw->bGPSavail || pRaw->bPPSavail))
	{
		printf("Raw Data error!!! %d,%d,%d\r\n",pRaw->bMEMSavail,pRaw->bGPSavail,pRaw->bPPSavail);
		return 0;
	}

	GI_pd.bMEMSavail = false;
	GI_pd.bGPSavail = false;
	GI_pd.bPPSavail = false;
	/*******************MEMS*********************/
	if (pRaw->bMEMSavail)
	{
		static double IMU_time_pre = 0;
		GI_pd.imutimetarge = pRaw->imutimetarget;
		if (GI_pd.imutimetarge - IMU_time_pre > 0.1)//IMU time error
		{
			//printf("libgilc --- raw imu time error: week %d, sec %f, last %f \r\n",
				//GI_pd.week, GI_pd.imutimetarge, IMU_time_pre);
			if (GI_pd.imutimetarge - IMU_time_pre)
			{
				//GINS_init();
			}
			IMU_time_pre = GI_pd.imutimetarge;
		}
		IMU_time_pre = GI_pd.imutimetarge;

		GI_pd.gyo[0] = pRaw->memsdate.gyro[0] * D2R;
		GI_pd.gyo[1] = pRaw->memsdate.gyro[1] * D2R;
		GI_pd.gyo[2] = pRaw->memsdate.gyro[2] * D2R;
		GI_pd.acc[0] = pRaw->memsdate.accel[0] * glv_g;
		GI_pd.acc[1] = pRaw->memsdate.accel[1] * glv_g;
		GI_pd.acc[2] = pRaw->memsdate.accel[2] * glv_g;

		/*dsf90:IMU数据检验*/
		for (int i = 0; i < 3; i++)
		{
			if (fabs(GI_pd.acc[i]) >= 10 * glv_g)/*加速度上限 10g*/
			{
				printf("libgilc --- raw imu accel error: week %d, sec %f, accel %f %f %f g\r\n",
					GI_pd.week, GI_pd.imutimetarge,
					GI_pd.acc[0] / glv_g, GI_pd.acc[1] / glv_g, GI_pd.acc[2] / glv_g);
				return 0;
			}
			if (fabs(GI_pd.gyo[i]) >= 1000 * D2R)/*陀螺仪上限 1000deg/s*/
			{
				printf("libgilc --- raw imu gyro error: week %d, sec %f, gyro %f %f %f deg/s\r\n",
					GI_pd.week, GI_pd.imutimetarge,
					GI_pd.gyo[0] * R2D, GI_pd.gyo[1] * R2D, GI_pd.gyo[2] * R2D);
				return 0;
			}
		}

		if (pRaw->bPPSavail)
		{
			//int dTime_ms = 0;
			////int imu_period_ms = 20; /*dsf90:2018.9.20,同步时间误差,最大允许20ms*/
			//int imu_period_ms = s_cfgdata.imu_period_ms>InitParam.iPpsTimeErrMax_ms ? s_cfgdata.imu_period_ms : InitParam.iPpsTimeErrMax_ms;
			//dTime_ms = (int)((ilcd.imutimetarge - (int)ilcd.imutimetarge) * 1000);
			//dTime_ms %= GNSS_PERIODE_MS;/*dsf90:默认GNSS 5Hz*/
			//if (dTime_ms < imu_period_ms || dTime_ms>(GNSS_PERIODE_MS - imu_period_ms))
			//{
			//	pIlcData->bPPSavail = pRaw->bPPSavail;
			//}
			//else
			//{
			//	printf("%8.6f: Update PPS flag, last flag %d, new flag %d,err times %dms\r\n", pRaw->imutimetarget, pRaw->bPPSavail, pIlcData->bPPSavail, dTime_ms);
			//}
		}
		GI_pd.bMEMSavail = true;
	}
	/*******************GNSS*********************/
	if (pRaw->bGPSavail)
	{
		static int iGnssSec_pre = 0;
		int iGnssSec = 0;
		double dErrTime = 0;

		iGnssSec = (int)(pRaw->gnssdata.second*1e3);
		if (fabs(iGnssSec - iGnssSec_pre - 200) > 10)
		{
			//printf("GNSS RAW LOST : last %d, now %d------------\r\n", iGnssSec_pre, iGnssSec);
		}
		iGnssSec_pre = iGnssSec;

		GI_pd.gpstimetarge = pRaw->gnssdata.second;
		GI_pd.week = pRaw->gnssdata.week;
		dErrTime = fabs(pRaw->gnssdata.second - GI_pd.imutimetarge);

#ifndef SYNC_PPS_UNUSED
		/*dsf90:时间同步检验*/
		if (!GI_pd.week || dErrTime >0.50)
		{
			printf("libgilc --- raw time error: week %d, sec %f, imu_sec %f, diff time %f s\r\n",
				GI_pd.week, GI_pd.gpstimetarge,
				GI_pd.imutimetarge, dErrTime);
			return 0;
		}
#endif
		//if (s_cfgdata.eGnssPosMode == GILC_GNSSPOS_MODE__LLA_DEG)
		//{
		GI_pd.pos[0] = pRaw->gnssdata.lat*D2R;
		GI_pd.pos[1] = pRaw->gnssdata.lon*D2R;
		//}
		//else if (s_cfgdata.eGnssPosMode == GILC_GNSSPOS_MODE__LLA_RAD)
		//{
		//GI_pd.pos[0] = pRaw->gnssdata.lat;
		//GI_pd.pos[1] = pRaw->gnssdata.lon;
		//}
		//else
		//{
		//	gilc_log("libgilc --- cfg pos mode error, %d\r\n", s_cfgdata.eGnssPosMode);
		//	return GILC_RET__ERR_CFG_POS_MODE;
		//}


		GI_pd.pos[2] = pRaw->gnssdata.alt;

		/*dsf90:经纬高输入检验*/
		/*经纬度上限 180deg；高程上限10000m*/
		if ((fabs(GI_pd.pos[0]) > PI) || (fabs(GI_pd.pos[1]) > PI) || (fabs(GI_pd.pos[2]) >= 10000))
		{
			printf("libgilc --- raw pos error: week %d, sec %f, pos %f rad %f rad %f m\r\n",
				GI_pd.week, GI_pd.gpstimetarge,
				GI_pd.pos[0], GI_pd.pos[1], GI_pd.pos[2]);
			//return GILC_RET__ERR_RAW_POS_PARAM;
		}

		if (1)
		{
			double vel_ecef[3] = { 0 };
			vel_ecef[0] = pRaw->gnssdata.vx_ecef;
			vel_ecef[1] = pRaw->gnssdata.vy_ecef;
			vel_ecef[2] = pRaw->gnssdata.vz_ecef;
			ecef2enu(GI_pd.pos, vel_ecef, GI_pd.vn);
			GI_pd.gnss_speed = sqrt(pow(GI_pd.vn[0], 2) + pow(GI_pd.vn[1], 2));
			GI_pd.heading = atan2(GI_pd.vn[0], GI_pd.vn[1])*R2D;
			if (GI_pd.heading <0)
				GI_pd.heading += 360;
		}
		else if (GI_cfg.GnssVelMode == 2)
		{
			GI_pd.gnss_speed = pRaw->gnssdata.speed;
			GI_pd.heading = pRaw->gnssdata.heading2;
			GI_pd.vn[0] = GI_pd.gnss_speed*sin(GI_pd.heading*D2R);
			GI_pd.vn[1] = GI_pd.gnss_speed*cos(GI_pd.heading*D2R);
			GI_pd.vn[2] = 0;
		}
		else
		{
			//printf("libgilc --- cfg vel mode error, %d\r\n", s_cfgdata.eGnssVelMode);
			return 0;
		}


		/*dsf90:速度输入检验*/
		/*速度上限：200m/s*/
		if (fabs(GI_pd.gnss_speed) > 200 || GI_pd.vn[2] > 20)
		{
			//gilc_log("libgilc --- raw vel error: week %d, sec %f, vel %f %f %f m/s, speed %f m/s\r\n",
			//	pIlcData->week, pIlcData->gpstimetarge,
			//	pIlcData->vn[0], pIlcData->vn[1], pIlcData->vn[2], pIlcData->gnss_speed);
			//return GILC_RET__ERR_RAW_VEL_PARAM;
		}

		double vel_std_enu[3] = { 0 };
		double pos_std_enu[3] = { 0 };
		double heading2_std = pRaw->gnssdata.std_heading2;
		if (1)
		{
			double vel_std_ecef[3] = { 0 };
			vel_std_ecef[0] = pRaw->gnssdata.std_vx_ecef;
			vel_std_ecef[1] = pRaw->gnssdata.std_vy_ecef;
			vel_std_ecef[2] = pRaw->gnssdata.std_vz_ecef;
			Var_XYZ2BLH(GI_pd.pos, vel_std_ecef, vel_std_enu);
		}
		else
		{
			if (pRaw->gnssdata.stat == 4)
			{
				vel_std_enu[0] = 0.05;	vel_std_enu[1] = 0.05;	vel_std_enu[2] = 0.1;
			}
			else if (pRaw->gnssdata.stat == 5)
			{
				vel_std_enu[0] = 0.1;		vel_std_enu[1] = 0.1;		vel_std_enu[2] = 0.2;
			}
			else
			{
				vel_std_enu[0] = 0.2;		vel_std_enu[1] = 0.2;		vel_std_enu[2] = 0.4;
			}
		}
		if (1)
		{
			pos_std_enu[0] = pRaw->gnssdata.std_lat;
			pos_std_enu[1] = pRaw->gnssdata.std_lon;
			pos_std_enu[2] = pRaw->gnssdata.std_alt;
		}
		else
		{
			if (pRaw->gnssdata.stat == 4)
			{
				pos_std_enu[0] = 0.05;	pos_std_enu[1] = 0.05;	pos_std_enu[2] = 0.1;
			}
			else if (pRaw->gnssdata.stat == 5)
			{
				pos_std_enu[0] = 1;		pos_std_enu[1] = 1;		pos_std_enu[2] = 2;
			}
			else
			{
				pos_std_enu[0] = 2;		pos_std_enu[1] = 2;		pos_std_enu[2] = 4;
			}
		}

		/*dsf90:标准差输入检验*/
		/*速度标准差下限：0.01m/s；位置标准差下限：0.01*/
		for (int i = 0; i < 3; i++)
		{
			if (vel_std_enu[i] == 0)
			{
				printf("libgilc --- raw vel std error: week %d, sec %f, vel std %f %f %f m/s\r\n",
					GI_pd.week, GI_pd.gpstimetarge,
					vel_std_enu[0], vel_std_enu[1], vel_std_enu[2]);
				//return GILC_RET__ERR_RAW_VEL_STD_PARAM;
			}
			else if (vel_std_enu[i] < 0.01)
				vel_std_enu[i] = 0.01;

			if (pos_std_enu[i] == 0)
			{
				printf("libgilc --- raw pos std error: week %d, sec %f, pos std %f %f %f m\r\n",
					GI_pd.week, GI_pd.gpstimetarge,
					pos_std_enu[0], pos_std_enu[1], pos_std_enu[2]);
				//return GILC_RET__ERR_RAW_POS_STD_PARAM;
			}
			else if (pos_std_enu[i] < 0.01)
				pos_std_enu[i] = 0.01;
		}
		/*航向标准差下限：0.2deg*/
		if (heading2_std && heading2_std <= 0.5)
		{
			//gilc_log("libgilc --- raw heading2 std error: week %d, sec %f, heading2 std %lf deg\r\n",
			//pIlcData->week, pIlcData->gpstimetarge,	heading2_std);
			//return GILC_RET__ERR_RAW_PARAM;
			heading2_std = 0.5;
		}

		GI_pd.GPV_RK[0 * 6 + 0] = SQ(vel_std_enu[0]);
		GI_pd.GPV_RK[1 * 6 + 1] = SQ(vel_std_enu[1]);
		GI_pd.GPV_RK[2 * 6 + 2] = SQ(vel_std_enu[2]);
		GI_pd.GPV_RK[3 * 6 + 3] = SQ(pos_std_enu[0] / RE);
		GI_pd.GPV_RK[4 * 6 + 4] = SQ(pos_std_enu[1] / RE);
		GI_pd.GPV_RK[5 * 6 + 5] = SQ(pos_std_enu[2]);

		GI_pd.stat = pRaw->gnssdata.stat;
		GI_pd.hdop = pRaw->gnssdata.hdop;
		GI_pd.age = pRaw->gnssdata.age;
		GI_pd.ns = pRaw->gnssdata.ns;
		GI_pd.nsused = pRaw->gnssdata.nsused;
		GI_pd.snsum = pRaw->gnssdata.snsum;
		GI_pd.heading2 = pRaw->gnssdata.heading2;
		DEG_0_360(GI_pd.heading2);
		GI_pd.GA_RK[2] = SQ(heading2_std*D2R);

		/*固定解，方差大小检查*/
		//if (GI_pd.stat == 4)
		//{/*定位精度不达标，修改定位状态为浮动*/
		//	if (pos_std_enu[0] > 0.1 || pos_std_enu[1] > 0.1 || pos_std_enu[2] > 0.2)
		//	{
		//		GI_pd.stat = 5;
		//	}
		//}
		if (GI_pd.stat)
			GI_pd.bGPSavail = true;
	}
	GI_pd.bValid = GI_pd.bGPSavail || GI_pd.bMEMSavail;
	return GI_pd.bValid;/*更新标志*/
}

void GINS_YM::GINS_ekf_data_ready(GINS_cfg_double *GI_cfg)
{


	double kf_P_init_gyobias[3] = { 0,0,0 };
	double kf_P_init_accbias[3] = { 0,0,0 };
	double kf_P_init_installerr[3] = { 0,0,0 };
	double kf_P_init_lever[3] = { 0,0,0};
	double kf_Q_init_att[3] = { 0.0 };
	double kf_Q_init_vel[3] = { 0.0 };
	double kf_Q_init_pos[3] = { 0.0 };
	double kf_Q_init_gyobias[3] = { 0.0 };
	double kf_Q_init_accbias[3] = { 0.0 };

	kf_Q_init_att[0] = GI_cfg->gyro_std[0];
	kf_Q_init_att[1] = GI_cfg->gyro_std[1];
	kf_Q_init_att[2] = GI_cfg->gyro_std[2];

	kf_Q_init_vel[0] = GI_cfg->accle_std[0];
	kf_Q_init_vel[1] = GI_cfg->accle_std[1];
	kf_Q_init_vel[2] = GI_cfg->accle_std[2];


	kf_Q_init_gyobias[0] = GI_cfg->gyro_walk[0];
	kf_Q_init_gyobias[1] = GI_cfg->gyro_walk[1];
	kf_Q_init_gyobias[2] = GI_cfg->gyro_walk[2];

	kf_Q_init_accbias[0] = GI_cfg->vel_walk[0];
	kf_Q_init_accbias[1] = GI_cfg->vel_walk[1];
	kf_Q_init_accbias[2] = GI_cfg->vel_walk[2];
	/*Q INIT*/
	for (int i = 0; i < 3; i++)
	{
		GI_pro.kf.gyro_bias_walk[i] = GI_cfg->gyro_walk[i];
		GI_pro.kf.acc_bias_walk[i] = GI_cfg->vel_walk[i];
		GI_pro.kf.acc_std[i] = GI_cfg->accle_std[i];
		GI_pro.kf.gyro_std[i] = GI_cfg->gyro_std[i];
		GI_pro.kf.kf_Q_init[i + 0] = (kf_Q_init_att[i]);
		GI_pro.kf.kf_Q_init[i + 3] = (kf_Q_init_vel[i]);
		GI_pro.kf.kf_Q_init[i + 6] = (kf_Q_init_pos[i]);
		GI_pro.kf.kf_Q_init[i + 9] = (kf_Q_init_gyobias[i]);
		GI_pro.kf.kf_Q_init[i + 12] = (kf_Q_init_accbias[i]);
	}


	double davp[9] = { 0 };
	davp[0] = 2;
	davp[1] = 2;
	davp[2] = 5;
	davp[3] = sqrt(GI_pd.GPV_RK[0]) * 3;
	davp[4] = sqrt(GI_pd.GPV_RK[7]) * 3;
	davp[5] = sqrt(GI_pd.GPV_RK[14]) * 3;
	davp[6] = sqrt(GI_pd.GPV_RK[21]) * 3;
	davp[7] = sqrt(GI_pd.GPV_RK[28]) * 3;
	davp[8] = sqrt(GI_pd.GPV_RK[35]) * 3;
	/*P INIT*/
	for (int i = 0; i<3; i++)
	{
		GI_pro.kf.kf_P_init[i + 0] = (davp[i]);
		GI_pro.kf.kf_P_init[i + 3] = (davp[i + 3]);
		GI_pro.kf.kf_P_init[i + 6] = (davp[i + 6]);
		GI_pro.kf.kf_P_init[i + 9] = (kf_P_init_gyobias[i]);
		GI_pro.kf.kf_P_init[i + 12] = (kf_P_init_accbias[i]);
		GI_pro.kf.kf_P_init[i + 15] = (kf_P_init_installerr[i]);
		GI_pro.kf.kf_P_init[i + 18] = (kf_P_init_lever[i]);
	}
}


int GINS_YM::GINS_Init(GINS_cfg_t* cfgdata)
{
	memset(&stRaw, 0, sizeof(stRaw));
	memset(&stRaw_tmp, 0, sizeof(stRaw_tmp));
	memset(&GI_pro, 0, sizeof(GI_pro));
	stRaw = { 0 };
	stRaw_tmp = { 0 };
	//GI_pro = { 0 };
	for (int i = 0;i < 3;i++)
	{
		GI_cfg.accle_std[i] = cfgdata->accle_std[i];
		GI_cfg.gyro_std[i] = cfgdata->gyro_std[i];
		GI_cfg.vel_walk[i] = cfgdata->vel_walk[i];
		GI_cfg.gyro_walk[i] = cfgdata->gyro_walk[i];
		eb[i] = 0;
		db[i] = 0;
	}
	bAlign = false;

	GI_pro.ins.eb[3] = { 0 };
	GI_pro.ins.ab[3] = { 0 };

	GI_pro.ins.att[3] = { 0 };
	GI_pro.ins.pos[3] = { 0 };
	GI_pro.ins.vn[3] = { 0 };

	GI_pro.kf_init = false;
	GI_pro.kf.ROW = 21;
	GI_pro.kf.COL = 15;

	GI_align.init();
	//sprintf(cGilcInitMsg, "GILC Init ver: %s %s\r\n", GILC_SOFT_VER, GILC_SOFT_DATE);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC NUMX %d, NUMV %d\r\n", NUMX, NUMV);

	//raw_cnt = 0;
	//dbg_cnt = 0;
	//rst_cnt = 0;

	//GetGPS = false;
	//bAlign = false;
	////gilc_log("init-4\n");
	//dInstallAttCfg[0] = s_cfgdata.fIns2BodyAngle[0] * D2R;
	//dInstallAttCfg[1] = s_cfgdata.fIns2BodyAngle[1] * D2R;
	//dInstallAttCfg[2] = s_cfgdata.fIns2BodyAngle[2] * D2R;
	//a2mat(dInstallAttCfg, dInstallMat);

	//dIns2BodyLever[0] = s_cfgdata.fIns2BodyVector[0];
	//dIns2BodyLever[1] = s_cfgdata.fIns2BodyVector[1];
	//dIns2BodyLever[2] = s_cfgdata.fIns2BodyVector[2];
	////gilc_log("init-5\n");
	//if ((int)(s_cfgdata.fWheelDistance[0] * 1000))
	//	InitParam.dWheelTrack = s_cfgdata.fWheelDistance[0];
	//else
	//	InitParam.dWheelTrack = 1.6;

	//if ((int)(s_cfgdata.fWheelDistance[1] * 1000))
	//	InitParam.dWheelBase = s_cfgdata.fWheelDistance[1];
	//else
	//	InitParam.dWheelBase = 2.7;

	//if (s_cfgdata.imu_period_ms)
	//	InitParam.iImuPeriod_ms = s_cfgdata.imu_period_ms;

	//switch (s_cfgdata.iWorkMode)
	//{
	//case GILC_WORK_MODE__DEFAULT:
	//case GILC_WORK_MODE__CAR_NORMAL:
	//case GILC_WORK_MODE__TRAIN:
	//case GILC_WORK_MODE__SHIP:
	//case GILC_WORK_MODE__PLANE:
	//	InitParam.dGnssVerMin_ForInit = 0.2;
	//	InitParam.dPpsTimeErrMax_ms = 20;
	//	InitParam.dInstallErrStdMax_ForInit = 0.02;
	//	//#if AP100_ADI16445
	//	//			InitParam.dInstallErrStdMax_ForInit = 0.8;
	//	//#endif
	//	InitParam.dKfInitP_AccBais = 0.0001;
	//	InitParam.dKfInitP_GyroBais = 0.001;
	//	InitParam.dKfInitP_InstallErr = 0.05*D2R;
	//	InitParam.dKfInitP_Lever = 0.01;
	//	InitParam.dKfInitP_TimeDelay = 0.001;

	//	InitParam.dKfInitP_OdoKd = 0.05;
	//	InitParam.dKfInitP_OdoKwh = 0.1;
	//	InitParam.dKfInitP_OdoBwh = 0.5;
	//	InitParam.dKfInitP_OdoWh = 0.5;
	//	InitParam.dKfInitP_DualYaw = 0.1;
	//	InitParam.dKfInitP_OdoVel = 0.02;
	//	InitParam.dKfInitP_OdoHeading = 0.1;
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC Work Mode1 %d\r\n", s_cfgdata.iWorkMode);
	//	break;
	//case GILC_WORK_MODE__CAR_SLOW:
	//case GILC_WORK_MODE__TRACTOR:
	//	InitParam.dGnssVerMin_ForInit = 0.3;
	//	InitParam.dPpsTimeErrMax_ms = 50;
	//	InitParam.dInstallErrStdMax_ForInit = 0.01;
	//	InitParam.dKfInitP_AccBais = 0.005;
	//	InitParam.dKfInitP_GyroBais = 0.5;
	//	InitParam.dKfInitP_InstallErr = 0.002;
	//	InitParam.dKfInitP_Lever = 0.005;
	//	InitParam.dKfInitP_TimeDelay = 0.005;

	//	InitParam.dKfInitP_OdoKd = 0.05;
	//	InitParam.dKfInitP_OdoKwh = 0.1;
	//	InitParam.dKfInitP_OdoBwh = 0.5;
	//	InitParam.dKfInitP_OdoWh = 0.5;
	//	InitParam.dKfInitP_DualYaw = 0.1;
	//	InitParam.dKfInitP_OdoVel = 0.02;
	//	InitParam.dKfInitP_OdoHeading = 0.1;
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC Work Mode2 %d\r\n", s_cfgdata.iWorkMode);
	//	break;
	//default:
	//	gilc_log("Unsupport work mode %d\r\n", s_cfgdata.iWorkMode);
	//	return -1;
	//}
	////gilc_log("init-6\n");
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.bFilePathCfgUse      %d\r\n", cfgdata->bFilePathCfgUse);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.bOutFileSaveClose    %d\r\n", cfgdata->bOutFileSaveClose);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.bTmpFileSaveClose    %d\r\n", cfgdata->bTmpFileSaveClose);
	//if (cfgdata->bFilePathCfgUse)
	//{
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.debug_outfile_path   %s\r\n", cfgdata->debug_outfile_path);
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.debug_tmpfile_path   %s\r\n", cfgdata->debug_tmpfile_path);
	//}
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.bStdCfgUse           %d\r\n", cfgdata->bStdCfgUse);
	//if (cfgdata->bStdCfgUse)
	//{
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.gyro_std             %lf %lf %lf\r\n", cfgdata->gyro_std[0], cfgdata->gyro_std[1], cfgdata->gyro_std[2]);
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.accle_std            %lf %lf %lf\r\n", cfgdata->accle_std[0], cfgdata->accle_std[1], cfgdata->accle_std[2]);
	//}
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.bWalkCfgUse          %d\r\n", cfgdata->bWalkCfgUse);
	//if (cfgdata->bWalkCfgUse)
	//{
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.gyro_walk            %lf %lf %lf\r\n", cfgdata->gyro_walk[0], cfgdata->gyro_walk[1], cfgdata->gyro_walk[2]);
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.vel_walk             %lf %lf %lf\r\n", cfgdata->vel_walk[0], cfgdata->vel_walk[1], cfgdata->vel_walk[2]);
	//}
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.bRowScaleCfgUse      %d\r\n", cfgdata->bRowScaleCfgUse);
	//if (cfgdata->bRowScaleCfgUse)
	//{
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.gyro_row             %d %d %d\r\n", cfgdata->gyro_row[0], cfgdata->gyro_row[1], cfgdata->gyro_row[2]);
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.acc_row              %d %d %d\r\n", cfgdata->acc_row[0], cfgdata->acc_row[1], cfgdata->acc_row[2]);
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.gyro_scale           %lf %lf %lf\r\n", cfgdata->gyro_scale[0], cfgdata->gyro_scale[1], cfgdata->gyro_scale[2]);
	//	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.acc_scale            %lf %lf %lf\r\n", cfgdata->acc_scale[0], cfgdata->acc_scale[1], cfgdata->acc_scale[2]);
	//}

	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.bGnssPosStdUse       %d\r\n", cfgdata->bGnssPosStdUse);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.bGnssVelStdUse       %d\r\n", cfgdata->bGnssVelStdUse);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.eGnssVelMode         %d\r\n", cfgdata->eGnssVelMode);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.eGnssPosMode         %d\r\n", cfgdata->eGnssPosMode);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.iOutReferPoint       %d\r\n", cfgdata->iOutReferPoint);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.debug_level          %d\r\n", cfgdata->debug_level);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.fIns2BodyVector      %lf %lf %lf\r\n", cfgdata->fIns2BodyVector[0], cfgdata->fIns2BodyVector[1], cfgdata->fIns2BodyVector[2]);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.fIns2BodyAngle       %lf %lf %lf\r\n", cfgdata->fIns2BodyAngle[0], cfgdata->fIns2BodyAngle[1], cfgdata->fIns2BodyAngle[2]);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.fIns2GnssVector      %lf %lf %lf\r\n", cfgdata->fIns2GnssVector[0], cfgdata->fIns2GnssVector[1], cfgdata->fIns2GnssVector[2]);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.fIns2GnssAngle       %lf %lf %lf\r\n", cfgdata->fIns2GnssAngle[0], cfgdata->fIns2GnssAngle[1], cfgdata->fIns2GnssAngle[2]);
	//sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC cfg.fWheelDistance       %lf %lf\r\n", cfgdata->fWheelDistance[0], cfgdata->fWheelDistance[1]);
	//gilc_log(cGilcInitMsg);
	return 0;
}



int GINS_Process::GINS_P2(Process_Data ilcd)
{

	int ret = 0;
	//滤波器初始化
	if (!kf_init)
	{
		//for (int i = 0; i<3; i++)
		//{
		//	kf.GB[i] = para.GB[i];
		//	kf.AB[i] = para.AB[i];
		//	kf.GW[i] = para.GW[i];
		//	kf.AW[i] = para.AW[i];
		//	kf.GS[i] = para.GS[i];
		//	kf.AS[i] = para.AS[i];
		//}
		//vector<double>temppxk;
		//for (int i = 0;i<kf.ROW;i++)
		//{
		//	temppxk.push_back(sqrt(kf.Pxk[i*kf.ROW + i]));
		//}
		kf.GINS_KF_malloc(NUMX,NUMV,&kf_tmp);//分配内存空间
		kf.kfinit();

		//prePxk.push_back(temppxk);

		kf_init = true;
	}

	if (ilcd.bMEMSavail)
	{
		dt = ilcd.imutimetarge - tpre;		//需要准确的时间戳
		dt_total += dt;
		tpre = ilcd.imutimetarge;
		ins.INS_process(ilcd,dt);
		//ins.Lever(); //臂杆和时间延迟补偿
		kf.upPhi(ins, dt);
		kf.TUpdate(dt);  
		inspre = ins;
	}
	//判断是否有PPS
	if (ilcd.bPPSavail)
	{
		/*保存PPS时刻的ins信息*/
		inspre_forPPS = ins;
	}
	//ret = ZUpdate(ilcd);

	//double ins_speed = sqrt(pow(ins.vm_car[0], 2) + pow(ins.vm_car[1], 2));
	//if (ilcd.bMEMSavail && ins_speed > InitParam.dGnssVerMin_ForInit)
	//	updateKfInitStatus(ilcd);
	return ret;
}





void equalgpos(gpos_t* GP, Process_Data* lcdata)
{
	GP->lat = lcdata->pos[0];
	GP->lon = lcdata->pos[1];
	GP->hig = lcdata->pos[2];
	GP->ve = lcdata->vn[0];
	GP->vn = lcdata->vn[1];
	GP->vu = lcdata->vn[2];
	GP->state = lcdata->stat;
	//GP->yaw=lcdata->yaw_U62R;
	GP->yaw = lcdata->heading2*D2R;
}

void  GINS_Align::init(void)
{
	for (int i = 0;i < 3;i++)
	{
		PRY_Install[i] = 0;
		Att[i] = 0;
		Vn[i] = 0;
		Pos[i] = 0;
		VnL[i] = 0;
		PosL[i] =0;
	}

}


bool GINS_Align::KinmateAlign(Process_Data& ilcd, GINS_Process& gipro)
{
	Ngnss = 10;
	if (!bStatic&&ilcd.bGPSavail)
	{
		//计算航向- acc和gyo的ave和std已算
		if (ilcd.bGPSavail && ilcd.stat == 4)
		{
			gpos_t gnsstemp;
			equalgpos(&gnsstemp, &ilcd);
			gnsstemp.yaw = ilcd.heading;
			DEG_NEG180_180(gnsstemp.yaw);
			gnsstemp.yaw *= -D2R;
			if (heading_v.size()<Ngnss)
			{//GNSS点数不够
				heading_v.push_back(gnsstemp);
				return false;
			}
			else
			{
				heading_v.erase(heading_v.begin());
				heading_v.push_back(gnsstemp);
				bFinshAlign = CalAtt(ilcd, 0);
				return bFinshAlign;
			}
		}
#if 0
		else
		{
			if (ilcd.bGPSavail)
			{
				double speed = norm(ilcd.vn, 3);
				if (fabs(speed)>1.0 && ilcd.nsused > 12)
				{
					double gnssyawvtg = atan2(ilcd.vn[0], ilcd.vn[1]);
					yaw_gnss.push_back(gnssyawvtg);
					nspeed++;
					if (nspeed == 25)
					{
						double std_yaw = GetAveStd(yaw_gnss, 1);
						if (std_yaw<3 * D2R)
						{
							bFinshAlign = CalAtt2(ilcd);/*dsf90:连续20个点航向，std < 3deg, 姿态初始化*/
							return bFinshAlign;
						}
						else
						{
							nspeed--;
							yaw_gnss.erase(yaw_gnss.begin());
							return false;
						}
					}
					return false;
				}
				else
				{
					nspeed = 0;
					yaw_gnss.clear();
					return false;
				}
			}
			else
			{
				return false;
			}
		}
#endif
	}
	return false;
}


bool GINS_Align::CalAtt(Process_Data& ilcd, int opt)
{
	unsigned int i, j, n = 0, m = 0;
	vector<double> yaw;
	double dlon, dlat;
	double rr0[3], rr1[3], drr[3], blh[3], enu[3], enu_ave[3] = { 0.0 };

	for (i = 1;i<Ngnss;i++)
	{
		//yaw.push_back(v_GNSS[i].yaw);
		if (heading_v[i].yaw && ilcd.GA_RK[2] > SQ(0.001*D2R) && ilcd.GA_RK[2] < SQ(1 * D2R))
		{
			yaw.push_back(heading_v[i].yaw);
		}
		else
		{
			dlat = heading_v[i].lat - heading_v[0].lat; //dsf90:根据位置计算航向
			dlon = heading_v[i].lon - heading_v[0].lon;
			double iyaw = -atan2(dlon, dlat);
			yaw.push_back(iyaw);
		}
		n++;
	}

	for (i = 0;i<Ngnss - 1;i++)
	{
		blh[0] = heading_v[i].lat; blh[1] = heading_v[i].lon; blh[2] = heading_v[i].hig;
		pos2ecef(blh, rr0);

		blh[0] = heading_v[i + 1].lat; blh[1] = heading_v[i + 1].lon; blh[2] = heading_v[i + 1].hig;
		pos2ecef(blh, rr1);

		for (j = 0;j<3;j++)
		{
			drr[j] = rr1[j] - rr0[j];
		}

		ecef2enu(blh, drr, enu); //ENU方向位移分量
		if (enu[0] < (50 * GNSS_PERIODE_MS / 1000.0) &&
			enu[1] < (50 * GNSS_PERIODE_MS / 1000.0) &&
			enu[2] < (50 * GNSS_PERIODE_MS / 1000.0)) //max 50m/s
		{
			for (j = 0;j<3;j++)
			{
				enu_ave[j] += enu[j];
			}
			m++;
		}
	}
	for (j = 0;j<3;j++)  /*dsf90:ENU方向平均位移*/
	{
		enu_ave[j] /= m;
	}
	double std_yaw = GetAveStd(yaw, 1);
	double ver_enu = norm(enu_ave, 3)*(1000 / GNSS_PERIODE_MS);
	/*end add*/
	if (std_yaw<3.0)/*dsf90: 航向std< 3deg, 速度>0.5m/s*/
	{
		Att[2] = (heading_v.end() - 1)->yaw;
		double db[3];
		Mat_equal(ilcd.acc, 3, 1, db);
		double pitch = atan2(db[1], sqrt(pow(db[0], 2) + pow(db[2], 2)));
		double roll = atan2(-db[0], db[2]);
		Att[0] = pitch;         /*dsf90,风险:初始姿态设置为起步前的姿态*/
		Att[1] = roll;
		//位置 速度 方差 解状态初始化
		Vn[0] = heading_v.back().ve;
		Vn[1] = heading_v.back().vn;
		Vn[2] = heading_v.back().vu;
		Pos[0] = heading_v.back().lat;
		Pos[1] = heading_v.back().lon;
		Pos[2] = heading_v.back().hig;
		gnssstate = heading_v.back().state;
		return true;
	}
	else
	{
		//gilc_log("CalAtt %f %f\r\n",std_yaw,ver_enu);
		return false;
	}
}


bool GINS_Align::CalAtt2(Process_Data& ilcd)
{
	//double iyaw=-atan2(ilcd.vn[0],ilcd.vn[1]);
	double iyaw = -GetAveStd(yaw_gnss, 0);
	Att[2] = iyaw;
	double db[3];
	Mat_equal(ilcd.acc, 3, 1, db);
	double pitch = atan2(db[1], sqrt(pow(db[0], 2) + pow(db[2], 2)));
	double roll = atan2(-db[0], db[2]);
	Att[0] = pitch;         /*dsf90,风险:初始姿态设置为起步前的姿态*/
	Att[1] = roll;
	//位置 速度 方差 解状态初始化
	Vn[0] = ilcd.vn[0];
	Vn[1] = ilcd.vn[1];
	Vn[2] = ilcd.vn[2];
	Pos[0] = ilcd.pos[0];
	Pos[1] = ilcd.pos[1];
	Pos[2] = ilcd.pos[2];
	gnssstate = ilcd.stat;
	return true;
}

void GINS_Process::loadPPSSyncInsData(Process_Data &ilcd, GINS_INS &ppsins)
{
	bPPSSync = false;
	/*PPS SYNC*/
	if (ilcd.bGPSavail)
	{
		//int imu_period_ms = InitParam.iImuPeriod_ms>(int)InitParam.dPpsTimeErrMax_ms ? InitParam.iImuPeriod_ms : (int)InitParam.dPpsTimeErrMax_ms;
		int imu_period_ms = 20;
		ilcd.bGPSavail = false;
		if (fabs(inspre_forPPS.imutimetarge - ilcd.gpstimetarge) * 1000 < imu_period_ms)
		{	/*dsf90:匹配上次PPS位置*/
			ppsins = inspre_forPPS;
			bPPSSync = true;
			ilcd.bGPSavail = true;
		}
		else if (fabs(inspre.imutimetarge - ilcd.gpstimetarge) * 1000 < imu_period_ms)
		{	/*dsf90:匹配邻近INS位置*/
			ppsins = inspre;
			bPPSSync = true;
			ilcd.bGPSavail = true;
		}

		if (!ilcd.bGPSavail)  //GNSS 
		{
			printf("%8.6f: Update GNSS flag, get pps time %.6f, gps time %.6f, new flag 0\r\n", ilcd.imutimetarge, ppsins.imutimetarge, ilcd.gpstimetarge);
		}
	}

}