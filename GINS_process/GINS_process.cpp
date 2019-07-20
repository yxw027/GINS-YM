#include "GINS_process.h"


int GINS_YM::GINS_data_correct(GINS_raw_t *pRaw)
{
	if (pRaw->bGPSavail && pRaw->gnssdata.second != stRaw_tmp.gnssdata.second)
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

	//if (s_cfgdata.eGnssVelMode == GILC_GNSSVEL_MODE__ECEF)
	//{
	//	double vel_ecef[3] = { 0.0 }, pos[3] = { 0.0 };
	//	double vel_enu[3] = { 0.0 };
	//	vel_ecef[0] = pRaw->gnssdata.vx_ecef;
	//	vel_ecef[1] = pRaw->gnssdata.vy_ecef;
	//	vel_ecef[2] = pRaw->gnssdata.vz_ecef;
	//	pos[0] = pRaw->gnssdata.lat*D2R;
	//	pos[1] = pRaw->gnssdata.lon*D2R;
	//	pos[2] = pRaw->gnssdata.alt;
	//	ecef2enu(pos, vel_ecef, vel_enu);

	//	pRaw->gnssdata.speed = sqrt(pow(vel_enu[0], 2) + pow(vel_enu[1], 2));
	//	pRaw->gnssdata.heading = atan2(vel_enu[0], vel_enu[1])*R2D;

	//	if (pRaw->gnssdata.heading <0)
	//		pRaw->gnssdata.heading += 360;
	//}

}


void GINS_YM::GINS_ImuAxis_Correct(GINS_raw_t *pRaw)
{
	//if (s_cfgdata.bRowScaleCfgUse)
	//{
	//	pRaw->memsdate.gyro[0] = pRaw->memsdate.alldata[s_cfgdata.gyro_row[0] - 1] * s_cfgdata.gyro_scale[0];
	//	pRaw->memsdate.gyro[1] = pRaw->memsdate.alldata[s_cfgdata.gyro_row[1] - 1] * s_cfgdata.gyro_scale[1];
	//	pRaw->memsdate.gyro[2] = pRaw->memsdate.alldata[s_cfgdata.gyro_row[2] - 1] * s_cfgdata.gyro_scale[2];
	//	pRaw->memsdate.accel[0] = pRaw->memsdate.alldata[s_cfgdata.acc_row[0] - 1] * s_cfgdata.acc_scale[0];
	//	pRaw->memsdate.accel[1] = pRaw->memsdate.alldata[s_cfgdata.acc_row[1] - 1] * s_cfgdata.acc_scale[1];
	//	pRaw->memsdate.accel[2] = pRaw->memsdate.alldata[s_cfgdata.acc_row[2] - 1] * s_cfgdata.acc_scale[2];
	//}
	//else
	//{
	//	pRaw->memsdate.gyro[0] = pRaw->memsdate.alldata[CFG_GYRO_X_ROW - 1] * CFG_GYRO_X_SCALE;
	//	pRaw->memsdate.gyro[1] = pRaw->memsdate.alldata[CFG_GYRO_Y_ROW - 1] * CFG_GYRO_Y_SCALE;
	//	pRaw->memsdate.gyro[2] = pRaw->memsdate.alldata[CFG_GYRO_Z_ROW - 1] * CFG_GYRO_Z_SCALE;
	//	pRaw->memsdate.accel[0] = pRaw->memsdate.alldata[CFG_ACC_X_ROW - 1] * CFG_ACC_X_SCALE;
	//	pRaw->memsdate.accel[1] = pRaw->memsdate.alldata[CFG_ACC_Y_ROW - 1] * CFG_ACC_Y_SCALE;
	//	pRaw->memsdate.accel[2] = pRaw->memsdate.alldata[CFG_ACC_Z_ROW - 1] * CFG_ACC_Z_SCALE;
	//}

	//Mmulnm(dInstallMat, pRaw->memsdate.gyro, 3, 3, 1, pRaw->memsdate.gyro);
	//Mmulnm(dInstallMat, pRaw->memsdate.accel, 3, 3, 1, pRaw->memsdate.accel);
}



int  GINS_YM::GINS_Rawdata_Quality(GINS_raw_t *pRaw, Process_Data *pIlcData)
{
	if (!(pRaw->bMEMSavail || pRaw->bGPSavail || pRaw->bPPSavail))
	{
		printf("Raw Data error!!! %d,%d,%d\r\n",pRaw->bMEMSavail,pRaw->bGPSavail,pRaw->bPPSavail);
		//return GILC_RET__ERR_RAW_NO_UPDATE;
	}

	pIlcData->bMEMSavail = false;
	pIlcData->bGPSavail = false;
	pIlcData->bPPSavail = false;
	/*******************MEMS*********************/
	if (pRaw->bMEMSavail)
	{
		static double IMU_time_pre = 0;
		pIlcData->imutimetarge = pRaw->imutimetarget;
		if (pIlcData->imutimetarge - IMU_time_pre > 0.1)//IMU time error
		{
			printf("libgilc --- raw imu time error: week %d, sec %f, last %f \r\n",
				pIlcData->week, pIlcData->imutimetarge, IMU_time_pre);
			if (pIlcData->imutimetarge - IMU_time_pre)
			{
				//GINS_init();
			}
			IMU_time_pre = pIlcData->imutimetarge;
			//return GILC_RET__ERR_RAW_IMU_TIME;
		}
		IMU_time_pre = pIlcData->imutimetarge;

		pIlcData->gyo[0] = pRaw->memsdate.gyro[0] * D2R;
		pIlcData->gyo[1] = pRaw->memsdate.gyro[1] * D2R;
		pIlcData->gyo[2] = pRaw->memsdate.gyro[2] * D2R;
		pIlcData->acc[0] = pRaw->memsdate.accel[0] * glv_g;
		pIlcData->acc[1] = pRaw->memsdate.accel[1] * glv_g;
		pIlcData->acc[2] = pRaw->memsdate.accel[2] * glv_g;

		/*dsf90:IMU数据检验*/
		for (int i = 0; i < 3; i++)
		{
			if (fabs(pIlcData->acc[i]) >= 10 * glv_g)/*加速度上限 10g*/
			{
				printf("libgilc --- raw imu accel error: week %d, sec %f, accel %f %f %f g\r\n",
					pIlcData->week, pIlcData->imutimetarge,
					pIlcData->acc[0] / glv_g, pIlcData->acc[1] / glv_g, pIlcData->acc[2] / glv_g);
				//return GILC_RET__ERR_RAW_IMU_ACCEL_PARAM;
			}
			if (fabs(pIlcData->gyo[i]) >= 1000 * D2R)/*陀螺仪上限 1000deg/s*/
			{
				printf("libgilc --- raw imu gyro error: week %d, sec %f, gyro %f %f %f deg/s\r\n",
					pIlcData->week, pIlcData->imutimetarge,
					pIlcData->gyo[0] * R2D, pIlcData->gyo[1] * R2D, pIlcData->gyo[2] * R2D);
				//return GILC_RET__ERR_RAW_IMU_GYRO_PARAM;
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
		pIlcData->bMEMSavail = true;
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
			printf("GNSS RAW LOST : last %d, now %d------------\r\n", iGnssSec_pre, iGnssSec);
		}
		iGnssSec_pre = iGnssSec;

		pIlcData->gpstimetarge = pRaw->gnssdata.second;
		pIlcData->week = pRaw->gnssdata.week;
		dErrTime = fabs(pRaw->gnssdata.second - pIlcData->imutimetarge);

#ifndef SYNC_PPS_UNUSED
		/*dsf90:时间同步检验*/
		if (!pIlcData->week || dErrTime >0.150)
		{
			printf("libgilc --- raw time error: week %d, sec %f, imu_sec %f, diff time %f s\r\n",
				pIlcData->week, pIlcData->gpstimetarge,
				pIlcData->imutimetarge, dErrTime);
			//return GILC_RET__ERR_RAW_GNSS_TIME_PARAM;
		}
#endif
		//if (s_cfgdata.eGnssPosMode == GILC_GNSSPOS_MODE__LLA_DEG)
		//{
		//	pIlcData->pos[0] = pRaw->gnssdata.lat*D2R;
		//	pIlcData->pos[1] = pRaw->gnssdata.lon*D2R;
		//}
		//else if (s_cfgdata.eGnssPosMode == GILC_GNSSPOS_MODE__LLA_RAD)
		//{
		//	pIlcData->pos[0] = pRaw->gnssdata.lat;
		//	pIlcData->pos[1] = pRaw->gnssdata.lon;
		//}
		//else
		//{
		//	gilc_log("libgilc --- cfg pos mode error, %d\r\n", s_cfgdata.eGnssPosMode);
		//	return GILC_RET__ERR_CFG_POS_MODE;
		//}


		pIlcData->pos[2] = pRaw->gnssdata.alt;

		/*dsf90:经纬高输入检验*/
		/*经纬度上限 180deg；高程上限10000m*/
		if ((fabs(pIlcData->pos[0]) > PI) || (fabs(pIlcData->pos[1]) > PI) || (fabs(pIlcData->pos[2]) >= 10000))
		{
			printf("libgilc --- raw pos error: week %d, sec %f, pos %f rad %f rad %f m\r\n",
				pIlcData->week, pIlcData->gpstimetarge,
				pIlcData->pos[0], pIlcData->pos[1], pIlcData->pos[2]);
			//return GILC_RET__ERR_RAW_POS_PARAM;
		}

		if (GI_cfg.GnssVelMode == 1)
		{
			double vel_ecef[3] = { 0 };
			vel_ecef[0] = pRaw->gnssdata.vx_ecef;
			vel_ecef[1] = pRaw->gnssdata.vy_ecef;
			vel_ecef[2] = pRaw->gnssdata.vz_ecef;
			ecef2enu(pIlcData->pos, vel_ecef, pIlcData->vn);
			pIlcData->gnss_speed = sqrt(pow(pIlcData->vn[0], 2) + pow(pIlcData->vn[1], 2));
			pIlcData->heading = atan2(pIlcData->vn[0], pIlcData->vn[1])*R2D;
			if (pIlcData->heading <0)
				pIlcData->heading += 360;
		}
		else if (GI_cfg.GnssVelMode == 2)
		{
			pIlcData->gnss_speed = pRaw->gnssdata.speed;
			pIlcData->heading = pRaw->gnssdata.heading2;
			pIlcData->vn[0] = pIlcData->gnss_speed*sin(pIlcData->heading*D2R);
			pIlcData->vn[1] = pIlcData->gnss_speed*cos(pIlcData->heading*D2R);
			pIlcData->vn[2] = 0;
		}
		else
		{
			//gilc_log("libgilc --- cfg vel mode error, %d\r\n", s_cfgdata.eGnssVelMode);
			//return GILC_RET__ERR_CFG_VEL_MODE;
		}


		/*dsf90:速度输入检验*/
		/*速度上限：200m/s*/
		if (fabs(pIlcData->gnss_speed) > 200 || pIlcData->vn[2] > 20)
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
			Var_XYZ2BLH(pIlcData->pos, vel_std_ecef, vel_std_enu);
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
					pIlcData->week, pIlcData->gpstimetarge,
					vel_std_enu[0], vel_std_enu[1], vel_std_enu[2]);
				//return GILC_RET__ERR_RAW_VEL_STD_PARAM;
			}
			else if (vel_std_enu[i] < 0.01)
				vel_std_enu[i] = 0.01;

			if (pos_std_enu[i] == 0)
			{
				printf("libgilc --- raw pos std error: week %d, sec %f, pos std %f %f %f m\r\n",
					pIlcData->week, pIlcData->gpstimetarge,
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

		pIlcData->GPV_RK[0 * 6 + 0] = SQ(vel_std_enu[0]);
		pIlcData->GPV_RK[1 * 6 + 1] = SQ(vel_std_enu[1]);
		pIlcData->GPV_RK[2 * 6 + 2] = SQ(vel_std_enu[2]);
		pIlcData->GPV_RK[3 * 6 + 3] = SQ(pos_std_enu[0] / RE);
		pIlcData->GPV_RK[4 * 6 + 4] = SQ(pos_std_enu[1] / RE);
		pIlcData->GPV_RK[5 * 6 + 5] = SQ(pos_std_enu[2]);

		pIlcData->stat = pRaw->gnssdata.stat;
		pIlcData->hdop = pRaw->gnssdata.hdop;
		pIlcData->age = pRaw->gnssdata.age;
		pIlcData->ns = pRaw->gnssdata.ns;
		pIlcData->nsused = pRaw->gnssdata.nsused;
		pIlcData->snsum = pRaw->gnssdata.snsum;
		pIlcData->heading2 = pRaw->gnssdata.heading2;
		DEG_0_360(pIlcData->heading2);
		pIlcData->GA_RK[2] = SQ(heading2_std*D2R);

		/*固定解，方差大小检查*/
		if (pIlcData->stat == 4)
		{/*定位精度不达标，修改定位状态为浮动*/
			if (pos_std_enu[0] > 0.1 || pos_std_enu[1] > 0.1 || pos_std_enu[2] > 0.2)
			{
				pIlcData->stat = 5;
			}
		}
		if (pIlcData->stat)
			pIlcData->bGPSavail = true;
	}

	

	pIlcData->bValid = pIlcData->bGPSavail || pIlcData->bODOavail || pIlcData->bMEMSavail;
	return pIlcData->bValid;/*更新标志*/
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


GINS_YM *s_pstGilc = NULL;
int GINS_PROCESS_Lib(GINS_raw_t* pstRaw, GINS_result_t* pstOut)
{
	if (s_pstGilc)
		return s_pstGilc->GINS_PROCESS_Lib(pstRaw, pstOut);
	else
		//return GILC_RET__RST_NOTHING;
		return 0;
}

int GINS_Process::GINS_P2(Process_Data ilcd)
{
	double *wmcur, *vmcur, wm[3], vm[3];
	int ret = 0;
	//滤波器初始化
	if (!bAlign)
	{
		//for (int i = 0; i<9; i++)
		//{
		//	kf.davp[i] = para.davp[i];
		//}

		//for (int i = 0; i<3; i++)
		//{
		//	kf.GB[i] = para.GB[i];
		//	kf.AB[i] = para.AB[i];
		//	kf.GW[i] = para.GW[i];
		//	kf.AW[i] = para.AW[i];
		//	kf.GS[i] = para.GS[i];
		//	kf.AS[i] = para.AS[i];
		//}
		kf.kfinit();
		//vector<double, malloc_allocator<double> >temppxk;
		//for (int i = 0; i<kf.ROW; i++)
		//{
		//	temppxk.push_back(sqrt(kf.Pxk[i*kf.ROW + i]));
		//}
		//prePxk.push_back(temppxk);
		bAlign = true;
	}

	if (ilcd.bMEMSavail)
	{
		wmcur = ilcd.gyo; vmcur = ilcd.acc;

		/*dsf90:wm = 0.5*(wmpre + wmcur)*dt; wm, 角度变化增量 rad；疑问：dt使用前需先更新！*/
		/*dsf90:vm = 0.5*(vmpre + vmcur)*dt; vm, 速度变化增量 m/s*/
		//均值
		Mat_add(wmpre, wmcur, wm, 3, 1);
		Mat_add(vmpre, vmcur, vm, 3, 1);
		Mat_equal(wmcur, 3, 1, wmpre);
		Mat_equal(vmcur, 3, 1, vmpre);
		Mat_mulb(wm, 3, 1, 0.5, wm);
		Mat_mulb(vm, 3, 1, 0.5, vm);
		Mat_equal(wm, 3, 1, ins.wib);
		Mat_equal(vm, 3, 1, ins.fb);
		Mat_mulb(wm, 3, 1, dt,wm);
		Mat_mulb(vm, 3, 1, dt,vm);
		//需要准确的时间戳
		dt = ilcd.imutimetarge - tpre;
		dt_total += dt;
		tpre = ilcd.imutimetarge;

		ins.Update(wm, vm, dt);
		//ins.Lever(); //臂杆和时间延迟补偿
		kf.upPhi(ins, dt);
		kf.TUpdate(dt);  //考虑GK
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

