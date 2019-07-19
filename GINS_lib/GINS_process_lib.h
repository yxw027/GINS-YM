/*
* GILC_process_lib.h
*
*  Created on: 7.14, 2019
*      Author: yangmin
*/

#ifndef GILC_PROCESS_LIB_H
#define GILC_PROCESS_LIB_H
#ifdef __cplusplus
extern "C"
{
#endif

//IMU 原始数据结构体
typedef struct IMU_data_double 
	{
		double alldata[10];
		double accel[3];                     //accelerometer  X-Y-Z output (m/s2)
		double gyro[3];                      //gyroscope X-Y-Z output(rad/s)
	}IMU_data_t;

//GNSS原始数据结构体
typedef struct GNSS_data_double {
	/*time*/
	double second;
	int week;
	/*gnss-pos*/
	double lat;  //rad
	double lon;  //rad
	double alt;  //m 
	int pos_type;
	/*gnss-vel*/
	double vx_ecef;   //m/s
	double vy_ecef;   //m/s
	double vz_ecef;   //m/s
	/*gnss-std*/
	double std_lat;  //m
	double std_lon;  //m
	double std_alt;  //m
	double std_vx_ecef;  //m/s
	double std_vy_ecef;  //m/s
	double std_vz_ecef;  //m/s
	double std_heading2;  //deg
	double std_pitch2;  //deg
	/*gnss-satnum*/
	int stat;
	int nsused;//使用卫星数
	int ns;    //可见卫星数
	int snsum; //信噪比和
	double age;
	double hdop;
	double pdop;
	double vdop;
	double tdop;
	double gdop;
	int nsused2;//辅天线使用卫星数
	int ns2;    //辅天线搜星数
	/*gnss-att and speed*/
	double heading1;   //one ant, def
	double heading2; //two-ant,deg
	double pitch2; //deg
	double speed;   //m/s
}GNSS_data_t;
/*GINS 配置文件结构体*/
typedef struct GINS_cfg_double {
	/*DEBUG PARAM*/
	char   debug_outfile_path[128];
	char   debug_tmpfile_path[128];
	bool bFilePathCfgUse;      /*启用文件路径      自定义位置*/
	bool bOutFileSaveClose;    /*关闭结算结果保存*/
	bool bTmpFileSaveClose;    /*关闭结果文件保存*/

							   /*IMU PARAM*/
	int    imu_period_ms;
	double gyro_std[3];
	double accle_std[3];
	double gyro_walk[3];
	double vel_walk[3];
	int gyro_row[3];
	int acc_row[3];
	double gyro_scale[3];
	double acc_scale[3];
	bool bStdCfgUse;           /*启用IMU噪声       自定义配置*/
	bool bWalkCfgUse;          /*启用IMU随机游走   自定义配置*/
	bool bRowScaleCfgUse;      /*启用IMU顺序、比例 自定义配置*/

							   /*GNSS PARAM*/
	bool bGnssPosStdUse;       /*启用GNSS位置噪声观测量*/
	bool bGnssVelStdUse;       /*启用GNSS速度噪声观测量*/
	//gilc_gnssVel_mode_e eGnssVelMode;          /*0:ecef 1:enu 2:speed+heading*/
	//gilc_gnssPos_mode_e eGnssPosMode;
	int GnssVelMode;
	/*CALIBRATE PARAM*/
	//ekf_X_t stEkfX_Init;
	bool bEkfXUse;             /*启用EKF X 初始化配置*/

							   /*INSTALL/CAR PARAM*/
	float fIns2BodyAngle[3];     /*惯导到车辆后轮中心夹角*/
	float fIns2BodyAngleErr[3];  /*惯导到车辆后轮中心夹角-配置误差*/
	float fIns2BodyVector[3];    /*惯导到车辆后轮中心矢量*/
	float fIns2BodyVectorErr[3]; /*惯导到车辆后轮中心矢量-配置误差*/
	float fIns2GnssAngle[3];     /*惯导到GNSS定位主天线夹角*/
	float fIns2GnssAngleErr[3];  /*惯导到GNSS定位主天线夹角-配置误差*/
	float fIns2GnssVector[3];    /*惯导到GNSS定位主天线矢量*/
	float fIns2GnssVectorErr[3]; /*惯导到GNSS定位主天线矢量-配置误差*/
	float fOutPointVector[3];    /*GNSS定位天线到输出目标位置杆臂*/
	float fWheelDistance[2];     /*轮距 [0]:left-right;[2]front-rear*/
	int   iOutReferPoint;        /*输出参考点位置 0:IMU; 1:GNSS ANT; 2:输入目标位置*/
	int   iWorkMode;             /*1:common car 2:Low Speed Car 3:Track 4:farm machinery*/
	int   iReserved;
}GINS_cfg_t;
//算法接口输入数据结构体
typedef struct GINS_raw_double {
	double imutimetarget;    //second in gnss week, sync time by pps
	IMU_data_t memsdate;
	GNSS_data_t gnssdata;
	int  gnss_delay_ms;
	bool bMEMSavail;
	bool bGPSavail;
	bool bPPSavail;
}GINS_raw_t;
//算法结构输出数据结构体
typedef struct GINS_result_double {
	/*time*/
	double second;    //sec of gnss week
	int    week;
	/*IMU data*/
	double acc_car[3];
	double acc[3];
	double gyro[3];
	/*GINS pos and pos-std*/
	double lla[3];     //lat/lon/alt -- deg/deg/m
	double std_lla[3]; //lat/lon/alt -- m/m/m
	/*GINS vel and vel-std*/
	double vel_enu[3]; //m/s
	double std_vel[3]; //m/s
	/*GINS att*/
	double pitch;      //deg
	double roll;       //deg
	double yaw;        //deg
	/*gins install pry-std*/
	double std_pry[3]; //deg/deg/deg
	double speed;      //m/s
	double std_speed;  //m/s
	double heading;    //deg

	/*GINS falg*/
	int    bPPSSync;
	int    car_status;  //0.uninit 1.static 2.direct driver 3.turn right 4.turn left 5. back
	int    gilc_status;      //gilc_ret_e定义  0.uninit 1.resolving 2.convergence 3.ins  
}GINS_result_t;



//算法接口
int GINS_init();
int GINS_PROCESS_Lib(GINS_raw_t* pstRaw, GINS_result_t* pstOut);
#ifdef __cplusplus
}
#endif
#endif