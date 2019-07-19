#include "GINS_process_lib.h"
#include "GINS_process.h"

int GINS_YM::GINS_PROCESS_Lib(GINS_raw_t* pstRaw, GINS_result_t* pstOut)
{

	GINS_data_correct(pstRaw);//输入数据准备：gnss状态检测、imu轴系调整
	int ret =GINS_Rawdata_Quality(pstRaw, &GI_pd);//输入数据质量检测
	gtime_t gt = gpst2time(GI_pd.week, GI_pd.imutimetarge);//imu时间戳时间转换
	time2epoch(gpst2utc(gt), GI_pd.ep);
	//Mat_equal(ilcd.ep, 6, 1, glv.ep);


	/*add by dsf90,2018.6.8: IIR 实时滤波器*/
	double imu_raw[6];
	double imu_iir[6];
	//Mequalm(ilcd.acc, 3, 1, &imu_raw[0]);
	//Mequalm(ilcd.gyo, 3, 1, &imu_raw[3]);
	//iir_filter.IIR_Lowpass(imu_iir, imu_raw);
	//Mequalm(&imu_iir[0], 3, 1, ilcd.acc_iir);
	//Mequalm(&imu_iir[3], 3, 1, ilcd.gyo_iir);
	//Mequalm(&imu_iir[0],3,1,ilcd.acc);
	//Mequalm(&imu_iir[3],3,1,ilcd.gyo);

	//debugfile.SaveRaw(&ilcd);

	if (!GetGPS)
	{
		if (GI_pd.bGPSavail)
		{
			GetGPS = true;
		}
		//ret = GILC_RET__RST_INITING;
	}
	if (GetGPS)
	{
		//kinalign.bStatic = gipro.bStatic = gipro.detect.DetectStatic_car_dsf(ilcd.ep, ilcd.acc, ilcd.gyo, ilcd.vn, gipro.ins.vm_car, ilcd.bGPSavail, gipro.bGnssLost, ilcd.stat);
		bool align;
				//align= GI_align.KinmateAlign(GI_pd, GI_pro);//位置、速度、姿态初始化
			if (align)
			{
				bAlign = true;

				printf("%02d:%02d:%06.3f 启动：%f\r\n",
					int(GI_pd.ep[3]), int(GI_pd.ep[4]), GI_pd.ep[5], GI_pd.imutimetarge);

					//double lever[3] = { s_cfgdata.fIns2GnssVector[0],s_cfgdata.fIns2GnssVector[1],s_cfgdata.fIns2GnssVector[2] };			
				//gipro.ins.Init(kinalign.Att, kinalign.VnL, kinalign.PosL, eb, db, lever, dGnss2OutPointLever, kinalign.PRY_Install);
				

				if (GI_align.Att[2])
				{
					/*航向已初始化，通过杆臂、姿态，近似计算IMU初始位置*/
					//double dLeverAnt2Ins[3] = { 0 };
					//Mat_mul(s_cfgdata.stEkfX_Init.Lever, 3, 1, -1, dLeverAnt2Ins);
					//gipro.ins.Lever(kinalign.PosL, kinalign.VnL, gipro.ins.pos, gipro.ins.vn, dLeverAnt2Ins);
				}
				else
				{
					/*航向未初始化，以天线位置做为IMU初始位置*/
					Mat_equal(GI_align.VnL, 3, 1, GI_pro.ins.vn);
					Mat_equal(GI_align.PosL, 3, 1, GI_pro.ins.pos);
				}

				double cbn[9], vb[3];
				a2mat(GI_pro.ins.PRY_Install, GI_pro.ins.Cmb);//安装误差角转化为姿态矩阵
				a2mat(GI_pro.ins.att, GI_pro.ins.Cnb);//imu姿态转换为姿态矩阵

				Mat_tran(GI_pro.ins.Cnb, 3, 3, cbn);
				Mat_tran(GI_pro.ins.Cmb, 3, 3, GI_pro.ins.Cbm);


				Mat_mul(cbn, GI_pro.ins.vn, 3, 3, 1, vb);
				Mat_mul(GI_pro.ins.Cmb, vb, 3, 3, 1, GI_pro.ins.vm_car);//杆臂转换后的车辆速度

				Mat_mul(GI_pro.ins.Cnb, GI_pro.ins.Cbm, 3, 3, 3, GI_pro.ins.Cnm);//imu姿态杆臂转换
				m2att(GI_pro.ins.Cnm, GI_pro.ins.att_car);//车辆姿态

				GI_pro.preheading = GI_pro.ins.att[2];


				Mat_equal(GI_pd.gyo, 3, 1, GI_pro.wmpre);
				Mat_equal(GI_pd.acc, 3, 1, GI_pro.vmpre);
				GI_pro.tpre = GI_pd.imutimetarge - 0.01;
				GINS_ekf_data_ready(&GI_cfg);//滤波器参数初始

			}
			//ret = GILC_RET__RST_INITING;
		}
	int gilc_flag = 0;
	if (bAlign)
	{
		GI_pro.ins.imutimetarge = GI_pd.imutimetarge;
		//gilc_flag= GI_pro.GIPro_P2(GI_pd);


		//if (!s_cfgdata.bOutFileSaveClose)
		//{
		//	debugfile.SaveRst(stat, &gipro, &ilcd);
		//}
	}
			if (GI_pd.bGPSavail)
			{
				pstOut->bPPSSync = GI_pro.bPPSSync;
			}
	stRaw_tmp.bGPSavail = false;
	stRaw_tmp.bMEMSavail = false;
	stRaw_tmp.bPPSavail = false;
	return gilc_flag;
}