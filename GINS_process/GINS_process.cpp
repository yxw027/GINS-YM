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

void GINS_YM::GINS_result(GINS_result_t *presult, int GINS_flag)
{
	for (int i = 0;i < 3;i++)
	{
		presult->acc[i] = GI_pro.ins.fb[i];//去掉零偏后加速度
		presult->gyro[i] = GI_pro.ins.wib[i];//去掉零偏后角速度 
		presult->acc_car[i] = GI_pro.ins.am[i];
		presult->vel_enu[i] = GI_pd.vn[i];
	}
	presult->car_status = GI_pro.bTurn;
	presult->gilc_status =GINS_flag ;
	presult->heading = GI_pd.heading2;
	presult->speed = GI_pd.gnss_speed;
	presult->week = GI_pd.week;
	presult->second = GI_pd.imutimetarge;
	presult->pitch = GI_pro.ins.att_car[0];
	presult->roll = GI_pro.ins.att_car[1];
	presult->yaw = GI_pro.ins.att_car[2];
	if (GI_cfg.iOutReferPoint == 1)//Imu
	{
		presult->lla[0] = GI_pro.ins.pos[0];
		presult->lla[1] = GI_pro.ins.pos[1];
		presult->lla[2] = GI_pro.ins.pos[2];
	}
	if (GI_cfg.iOutReferPoint == 2)//gnss
	{
		presult->lla[0] = GI_pro.ins.posL[0];
		presult->lla[1] = GI_pro.ins.posL[1];
		presult->lla[2] = GI_pro.ins.posL[2];
	}
	presult->std_pry[0] = sqrt(GI_pro.kf.Pxk[0 * GI_pro.kf.ROW + 0])*R2D;
	presult->std_pry[1] = sqrt(GI_pro.kf.Pxk[1 * GI_pro.kf.ROW + 1])*R2D;
	presult->std_pry[2] = sqrt(GI_pro.kf.Pxk[2 * GI_pro.kf.ROW + 2])*R2D;

	presult->std_vel[0] = sqrt(GI_pro.kf.Pxk[3 * GI_pro.kf.ROW + 3]);
	presult->std_vel[1] = sqrt(GI_pro.kf.Pxk[4 * GI_pro.kf.ROW + 4]);
	presult->std_vel[2] = sqrt(GI_pro.kf.Pxk[5 * GI_pro.kf.ROW + 5]);
	presult->std_speed = sqrt(SQR(presult->std_vel[0]) + SQR(presult->std_vel[1]) + SQR(presult->std_vel[2]));

	presult->std_lla[0] = sqrt(GI_pro.kf.Pxk[6 * GI_pro.kf.ROW + 6])*RE_WGS84;
	presult->std_lla[1] = sqrt(GI_pro.kf.Pxk[7 * GI_pro.kf.ROW + 7])*RE_WGS84;
	presult->std_lla[2] = sqrt(GI_pro.kf.Pxk[8 * GI_pro.kf.ROW + 8]);
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
			GINS_log("GINS_RAW_ERROR gnss state error: week %d, sec %f, state %d\r\n",
				pRaw->gnssdata.week, pRaw->gnssdata.second, pRaw->gnssdata.pos_type);
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
			GINS_log("GINS_RAW_ERROR gnss state error: week %d, sec %f, state %d\r\n",
				pRaw->gnssdata.week, pRaw->gnssdata.second, pRaw->gnssdata.stat);
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

int GINS_YM::IMU_static_bias(void)
{
	if (GI_align.bStatic == 1)
	{
		if (IMU_gyro_winx.size() < 500)
		{
			IMU_gyro_winx.push_back(GI_pd.gyo[0]);
			IMU_gyro_winy.push_back(GI_pd.gyo[1]);
			IMU_gyro_winz.push_back(GI_pd.gyo[2]);
			return 0;
		}
		else
		{
			gyro_bias[0] = GetAveStd(IMU_gyro_winx, 0);
			gyro_bias[1] = GetAveStd(IMU_gyro_winy, 0);
			gyro_bias[2] = GetAveStd(IMU_gyro_winz, 0);
			IMU_gyro_winx.clear();
			IMU_gyro_winy.clear();
			IMU_gyro_winz.clear();
			return 1;
		}
	}
	else
	{
		return 0;
	}
}

int  GINS_YM::GINS_Rawdata_Quality(GINS_raw_t *pRaw)
{
	if (!(pRaw->bMEMSavail || pRaw->bGPSavail || pRaw->bPPSavail))
	{
		GINS_log("GINS_RAW_ERROR Raw Data NO avail!!! %d,%d,%d\r\n",pRaw->bMEMSavail,pRaw->bGPSavail,pRaw->bPPSavail);
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
			GINS_log("GINS_RAW_ERROR imu time error: week %d, sec %f, last %f \r\n",
				GI_pd.week, GI_pd.imutimetarge, IMU_time_pre);
			IMU_time_pre = GI_pd.imutimetarge;
		}
		IMU_time_pre = GI_pd.imutimetarge;
		GI_pd.gyo[0] = pRaw->memsdate.gyro[0] * D2R;
		GI_pd.gyo[1] = pRaw->memsdate.gyro[1] * D2R;
		GI_pd.gyo[2] = pRaw->memsdate.gyro[2] * D2R;
		GI_pd.acc[0] = pRaw->memsdate.accel[0] * glv_g;
		GI_pd.acc[1] = pRaw->memsdate.accel[1] * glv_g;
		GI_pd.acc[2] = pRaw->memsdate.accel[2] * glv_g;

		/*IMU数据检验*/
		for (int i = 0; i < 3; i++)
		{
			if (fabs(GI_pd.acc[i]) >= 10 * glv_g)/*加速度上限 10g*/
			{
				GINS_log("GINS_RAW_ERROR imu accel exeed 10g: week %d, sec %f, accel %f %f %f g\r\n",
					GI_pd.week, GI_pd.imutimetarge,
					GI_pd.acc[0] / glv_g, GI_pd.acc[1] / glv_g, GI_pd.acc[2] / glv_g);
				return 0;
			}
			if (fabs(GI_pd.gyo[i]) >= 1000 * D2R)/*陀螺仪上限 1000deg/s*/
			{
				GINS_log("GINS_RAW_ERROR imu gyro exeed 1000deg/s: week %d, sec %f, gyro %f %f %f deg/s\r\n",
					GI_pd.week, GI_pd.imutimetarge,
					GI_pd.gyo[0] * R2D, GI_pd.gyo[1] * R2D, GI_pd.gyo[2] * R2D);
				return 0;
			}
		}

		if (pRaw->bPPSavail)
		{
			int dTime_ms = 0;
			int imu_period_ms = 20;
			dTime_ms = (int)((GI_pd.imutimetarge - (int)GI_pd.imutimetarge) * 1000);//取非整秒数
			dTime_ms %= GNSS_PERIODE_MS;/*默认GNSS 5Hz*/
			if (dTime_ms < imu_period_ms || dTime_ms>(GNSS_PERIODE_MS - imu_period_ms))
			{
				GI_pd.bPPSavail = pRaw->bPPSavail;
			}
			else
			{
				GINS_log("GINS_RAW_ERROR pps time err %8.6f dTime %dms\r\n", pRaw->imutimetarget, dTime_ms);
			}
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
			GINS_log("GINS_RAW_ERROR gnss raw lost : last %d, now %d------------\r\n", iGnssSec_pre, iGnssSec);
		}
		iGnssSec_pre = iGnssSec;

		GI_pd.gpstimetarge = pRaw->gnssdata.second;
		GI_pd.week = pRaw->gnssdata.week;
		dErrTime = fabs(pRaw->gnssdata.second - GI_pd.imutimetarge);

		if (GI_cfg.GnssPosMode == 1)
		{
		GI_pd.pos[0] = pRaw->gnssdata.lat*D2R;
		GI_pd.pos[1] = pRaw->gnssdata.lon*D2R;
		}
		else if (GI_cfg.GnssPosMode == 2)
		{
		GI_pd.pos[0] = pRaw->gnssdata.lat;
		GI_pd.pos[1] = pRaw->gnssdata.lon;
		}
		else
		{
			GINS_log("GINS_RAW_ERROR cfg pos mode error, %d\r\n", GI_cfg.GnssPosMode);
			return 0;
		}
		GI_pd.pos[2] = pRaw->gnssdata.alt;

		/*经纬度上限 180deg；高程上限10000m*/
		if ((fabs(GI_pd.pos[0]) > PI) || (fabs(GI_pd.pos[1]) > PI) || (fabs(GI_pd.pos[2]) >= 10000))
		{
			GINS_log("GINS_RAW_ERROR raw gnss_pos error: week %d, sec %f, pos %f rad %f rad %f m\r\n",
				GI_pd.week, GI_pd.gpstimetarge,
				GI_pd.pos[0], GI_pd.pos[1], GI_pd.pos[2]);
			return 0;
		}

		if (GI_cfg.GnssVelMode==1)
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
			GINS_log("GINS_RAW_ERROR cfg vel mode error, %d\r\n", GI_cfg.GnssVelMode);
			return 0;
		}
		/*速度上限：200m/s*/
		if (fabs(GI_pd.gnss_speed) > 200 || GI_pd.vn[2] > 20)
		{
			GINS_log("GINS_RAW_ERROR raw gnss_vel exceed 200m/s: week %d, sec %f, vel %f %f %f m/s, speed %f m/s\r\n",
				GI_pd.week, GI_pd.gpstimetarge,
				GI_pd.vn[0], GI_pd.vn[1], GI_pd.vn[2], GI_pd.gnss_speed);
			return 0;
		}

		double vel_std_enu[3] = { 0 };
		double pos_std_enu[3] = { 0 };
		double heading2_std= pRaw->gnssdata.std_heading2;
		if (GI_cfg.GnssVelStdUse)
		{
			double vel_std_ecef[3] = { 0 };
			vel_std_ecef[0] = pRaw->gnssdata.std_vx_ecef;
			vel_std_ecef[1] = pRaw->gnssdata.std_vy_ecef;
			vel_std_ecef[2] = pRaw->gnssdata.std_vz_ecef;
			Var_XYZ2BLH(GI_pd.pos, vel_std_ecef, vel_std_enu);
		}
		else
		{
			switch (pRaw->gnssdata.stat)
			{
			case 4:
				vel_std_enu[0] = 0.05;	vel_std_enu[1] = 0.05;	vel_std_enu[2] = 0.1;
			case 5:
				vel_std_enu[0] = 0.1;		vel_std_enu[1] = 0.1;		vel_std_enu[2] = 0.2;
			case 1:
				vel_std_enu[0] = 0.2;		vel_std_enu[1] = 0.2;		vel_std_enu[2] = 0.4;
			case 6:
				vel_std_enu[0] = 0.2;		vel_std_enu[1] = 0.2;		vel_std_enu[2] = 0.4;
			}
		}
		if (GI_cfg.GnssPosStdUse)
		{
			pos_std_enu[0] = pRaw->gnssdata.std_lat;
			pos_std_enu[1] = pRaw->gnssdata.std_lon;
			pos_std_enu[2] = pRaw->gnssdata.std_alt;
		}
		else
		{
			switch (pRaw->gnssdata.stat)
			{
			case 4:
				pos_std_enu[0] = 0.05;	pos_std_enu[1] = 0.05;	pos_std_enu[2] = 0.1;
			case 5:
				pos_std_enu[0] = 1;		pos_std_enu[1] = 1;		pos_std_enu[2] = 2;
			case 1:
				pos_std_enu[0] = 2;		pos_std_enu[1] = 2;		pos_std_enu[2] = 4;
			case 6:
				pos_std_enu[0] = 2;		pos_std_enu[1] = 2;		pos_std_enu[2] = 4;
			}
		}
		/*速度标准差下限：0.01m/s；位置标准差下限：0.01*/
		for (int i = 0; i < 3; i++)
		{
			if (vel_std_enu[i] == 0)
			{
				GINS_log("GINS_RAW_ERROR raw gnss_vel std =0: week %d, sec %f, vel std %f %f %f m/s\r\n",
					GI_pd.week, GI_pd.gpstimetarge,
					vel_std_enu[0], vel_std_enu[1], vel_std_enu[2]);
				return 0;
			}
			else if (vel_std_enu[i] < 0.01)
				vel_std_enu[i] = 0.01;

			if (pos_std_enu[i] == 0)
			{
				GINS_log("GINS_RAW_ERROR raw gnss_pos std =0: week %d, sec %f, pos std %f %f %f m\r\n",
					GI_pd.week, GI_pd.gpstimetarge,
					pos_std_enu[0], pos_std_enu[1], pos_std_enu[2]);
				return 0;
			}
			else if (pos_std_enu[i] < 0.01)
				pos_std_enu[i] = 0.01;
		}
		/*航向标准差下限：0.2deg*/
		if (heading2_std && heading2_std <= 0.001)
		{
			GINS_log("GINS_RAW_ERROR raw heading2 std <=0.5: week %d, sec %f, heading2 std %lf deg\r\n",
				GI_pd.week, GI_pd.gpstimetarge,	heading2_std);
			heading2_std = 0.5;
			return 0;
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
		GI_pd.heading2_std = heading2_std;
		DEG_0_360(GI_pd.heading2);
		GI_pd.GA_RK[2] = SQ(heading2_std*D2R);
		if (GI_pd.stat)
			GI_pd.bGPSavail = true;
	}
	GI_pd.bValid = GI_pd.bGPSavail || GI_pd.bMEMSavail;
	return GI_pd.bValid;/*更新标志*/
}

void GINS_YM::GINS_ekf_data_ready(GINS_cfg_double *GI_cfg)
{

	memset(GI_pro.kf.kf_Q_init, 0, sizeof(GI_pro.kf.kf_Q_init));
	memset(GI_pro.kf.kf_P_init, 0, sizeof(GI_pro.kf.kf_P_init));

	double kf_P_init_gyobias[3] = { 0,0,0 };
	double kf_P_init_accbias[3] = { 0,0,0 };
	double kf_P_init_installerr[3] = { 0,0,0 };
	double kf_P_init_lever[3] = { 0,0,0};
	double kf_Q_init_att[3] = { 0.0 };
	double kf_Q_init_vel[3] = { 0.0 };
	double kf_Q_init_pos[3] = { 0.0 };
	double kf_Q_init_gyobias[3] = { 0.0 };
	double kf_Q_init_accbias[3] = { 0.0 };



	kf_Q_init_pos[0] = 1 / sqrt(3600) / RE;
	kf_Q_init_pos[1] = 1 / sqrt(3600) / RE;
	kf_Q_init_pos[2] = 1 / sqrt(3600);


	kf_Q_init_att[0] = GI_cfg->gyro_std[0] *D2R;
	kf_Q_init_att[1] = GI_cfg->gyro_std[1] * D2R;
	kf_Q_init_att[2] = GI_cfg->gyro_std[2] * D2R;

	kf_Q_init_vel[0] = GI_cfg->accle_std[0] *glv_g;
	kf_Q_init_vel[1] = GI_cfg->accle_std[1] * glv_g;
	kf_Q_init_vel[2] = GI_cfg->accle_std[2] * glv_g;


	kf_Q_init_gyobias[0] = GI_cfg->gyro_walk[0] *D2R / sqrt(3600);
	kf_Q_init_gyobias[1] = GI_cfg->gyro_walk[1] * D2R / sqrt(3600);
	kf_Q_init_gyobias[2] = GI_cfg->gyro_walk[2] * D2R / sqrt(3600);

	kf_Q_init_accbias[0] = GI_cfg->vel_walk[0] *1.0e-6*glv_g/sqrt(3600);
	kf_Q_init_accbias[1] = GI_cfg->vel_walk[1] * 1.0e-6*glv_g / sqrt(3600);
	kf_Q_init_accbias[2] = GI_cfg->vel_walk[2] * 1.0e-6*glv_g / sqrt(3600);


	/*Q INIT*/
	for (int i = 0; i < 3; i++)
	{
		GI_pro.kf.kf_Q_init[i + 0] = (kf_Q_init_att[i]);
		GI_pro.kf.kf_Q_init[i + 3] = (kf_Q_init_vel[i]);
		GI_pro.kf.kf_Q_init[i + 6] = (kf_Q_init_pos[i]);
		GI_pro.kf.kf_Q_init[i + 9] = (kf_Q_init_gyobias[i]);
		GI_pro.kf.kf_Q_init[i + 12] = (kf_Q_init_accbias[i]);
	}
	double davp[9] = { 0 };
	davp[0] = 2*D2R;
	davp[1] = 2 * D2R;
	davp[2] = 5 * D2R;
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
	GINS_log("kf_P_att:%f,%f,%f\n", davp[0], davp[1], davp[2]);
	GINS_log("kf_P_vel:%f,%f,%f\n", davp[3], davp[4], davp[5]);
	GINS_log("kf_P_pos:%f,%f,%f\n", davp[6], davp[7], davp[8]);



}


int GINS_YM::GINS_Init(GINS_cfg_t* cfgdata)
{
	memset(&stRaw, 0, sizeof(stRaw));
	memset(&stRaw_tmp, 0, sizeof(stRaw_tmp));
	memset(&GI_pro, 0, sizeof(GI_pro));
	stRaw = { 0 };
	stRaw_tmp = { 0 };
	bool File_flag = false;

	time_t tt = time(NULL);//这句返回的只是一个时间cuo
	tm* t = localtime(&tt);

	for (int i = 0;i < 3;i++)
	{
		GI_cfg.accle_std[i] = cfgdata->accle_std[i];
		GI_cfg.gyro_std[i] = cfgdata->gyro_std[i];
		GI_cfg.vel_walk[i] = cfgdata->vel_walk[i];
		GI_cfg.gyro_walk[i] = cfgdata->gyro_walk[i];
		dGnss2OutPointLever[i] = 0;
		eb[i] = 0;
		db[i] = 0;
		gyro_bias[i] = 0;
	}
	GI_cfg.GnssPosMode = cfgdata->GnssPosStdUse;
	GI_cfg.GnssVelMode = cfgdata->GnssVelMode;
	GI_cfg.GnssPosStdUse = cfgdata->GnssPosStdUse;
	GI_cfg.GnssVelStdUse = cfgdata->GnssVelStdUse;
	imu_raw_cnt = 0;
	bAlign = false;
	GI_pro.kf_init = false;
	bias_init = false;
	GI_align.init();
	GI_pro.Init();
	GI_pd.Init();
	if (!File_flag)
	{
		File_open();
		File_flag = true;
	}
	GINS_log("GINS_init:%d-%02d-%02d %02d:%02d:%02d\n", t->tm_year + 1900,
		t->tm_mon + 1,
		t->tm_mday,
		t->tm_hour,
		t->tm_min,
		t->tm_sec);
	GINS_log("GYRO_std:%f,%f,%f\n", cfgdata->gyro_std[0], cfgdata->gyro_std[1], cfgdata->gyro_std[2]);
	GINS_log("ACC_std:%f,%f,%f\n", cfgdata->accle_std[0], cfgdata->accle_std[1], cfgdata->accle_std[2]);
	GINS_log("GYRO_walk:%f,%f,%f\n", cfgdata->gyro_walk[0], cfgdata->gyro_walk[1], cfgdata->gyro_walk[2]);
	GINS_log("ACC_walk:%f,%f,%f\n", cfgdata->vel_walk[0], cfgdata->vel_walk[1], cfgdata->vel_walk[2]);
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

void GINS_Process::correctSideslip(void)
{
	/*dsf90:位置结果约束（限位）*/
	double dpos_b[3] = { 0 };
	int pro_flag = 0;

	if (fabs(pospre[0])<1e-6)
	{
		Mat_equal(ins.pos, 3, 1, pospre);
	}
	else
	{
		difpos_b(pospre, ins.pos, ins.att_car, dpos_b);

		/*侧向位移修正*/
		double dpos_x = sin(ins.wib[2] * dt / 2) * (ins.vm_car[1] * dt) * 2;//侧向位移增量 20为经验值
		if (fabs(dpos_b[0]) > fabs(dpos_x) && bGnssLost)
		{
			dpos_b[0] = dpos_x;
			pro_flag = 1;
		}
#if 0	
		/*高程位移修正*/
		//double dpos_z = sin(ins.wib[0] * dt / 2) * (ins.vm_car[1] * dt) * 500;
		double dpos_z = tan(ins.wib[0] * dt / 2) * dpos_b[1] * 500;
		if (fabs(dpos_b[2]) > fabs(dpos_z) && bGnssLost)
		{
			dpos_b[2] = dpos_z;
			pro_flag = 1;
		}
#endif
		if (pro_flag)
			difpos(pospre, ins.pos, ins.att_car, dpos_b);
		Mat_equal(ins.pos, 3, 1, pospre);
	}
}

int GINS_Process::GINS_P2(Process_Data ilcd)
{
	int ret = 0;
	//滤波器初始化
	if (!kf_init)
	{
		kf.GINS_KF_malloc(NUMX,NUMV,&kf_tmp);//分配内存空间
		kf.kfinit();
		kf_init = true;
	}

	if (ilcd.bMEMSavail)
	{
		dt = ilcd.imutimetarge - tpre;		//需要准确的时间戳
		dt_total += dt;
		tpre = ilcd.imutimetarge;
		ins.INS_process(ilcd,dt);//INS推算
		ins.Lever(); //臂杆和时间延迟补偿
		kf.upPhi(ins, dt);//状态转移矩阵更新
		kf.TUpdate(dt);//时间更新
		inspre = ins;
	}
	//判断是否有PPS
	if (ilcd.bPPSavail)
	{
		/*保存PPS时刻的ins信息*/
		inspre_forPPS = ins;
	}
	ret = ZUpdate(ilcd);

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


void GINS_Process::Init(void)
{
	ins = { 0 };
	inspre = { 0 };
	inspre_forPPS = { 0 };
	inspre_forStatic = { 0 };
	kf_init = false;
	bAlign = false;
	bStatic = false;
	bTurn = 1;
	busegnssvn = false;
	bgnssskip = false;
	bGnssLost=false;
	bStaticpre = false;
	bGnssLostNum=0;
	bGnssNum = 0;
}

void GINS_Process::GnssIntegrity(Process_Data &ilcd, double dheading, double dvn[3])
{
	if (ilcd.bGPSavail)	//GNSS
	{
		//根据gnss速度与速度差来判断GNSS速度是否可用
		busegnssvn = busegnssvel_car(ilcd.vn, dvn, dheading);//busegnssvn=1速度可用
		//判断GNSS跳点
		gpos_t gpcur;
		equalgpos(&gpcur, &ilcd);
		if (pregpos.size()<10)
		{
			pregpos.push_back(gpcur);
		}
		else
		{
			bgnssskip = bgnssskip_car(pregpos, ins.vn, c);
			pregpos.clear();
			//pregpos.erase(pregpos.begin());
		}

	}
	if (ilcd.bGPSavail)	//GNSS
	{
		bGnssTimeOut = 0;
		if (bGnssLost)
		{
			printf("%02d:%02d:%06.3f ----gnss get-------%f\r\n", int(ilcd.ep[3]), int(ilcd.ep[4]), ilcd.ep[5],ilcd.imutimetarge);
		}
		bGnssLost = 0;
	}
	else
	{
		bGnssTimeOut += dt;
		if (bGnssTimeOut >= 0.5)
		{
			if (!bGnssLost)
			{
				printf("%02d:%02d:%06.3f ----gnss lost-------%f\r\n", int(ilcd.ep[3]), int(ilcd.ep[4]), ilcd.ep[5],ilcd.imutimetarge);
			}
			bGnssLost = 1;
		}
	}
	if (bGnssLost)
	{
		bGnssLostNum++;
		bGnssNum = 0;
	}
	else
	{
		bGnssLostNum = 0;
		bGnssNum++;
	}
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

void Process_Data::Init(void )
{
	gpstimetarge = 0;
	week = 0;
	ep[6] = { 0 };
	pos[3] = { 0 };
	for (int i = 0;i < 36;i++)
	{
		GPV_RK[i] = 0;
	}

	for (int i = 0;i < 3; i++)
	{
		GA_RK[i] = 0;
		acc_iir[i] = 0;
		gyo_iir[i] = 0;
		mag_iir[i] = 0;
		acc[i] = 0;
		gyo[i] = 0;
		mag[i] = 0;
	}
	heading2 = 0;
	heading2_std = 0;
	imutimetarge = 0;
	gnss_speed = 0;
	heading = 0;
	time = 0;
	num = 0;
	temper = 0;
	lever = 0;
	bMEMSavail = false;
	bPPSavail = false;
	bGPSavail = false;
	imu_raw_cnt = 0;
}