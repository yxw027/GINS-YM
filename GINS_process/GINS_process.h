#ifndef _GINS_PROCESS_H
#define _GINS_PROCESS_H
#include "GINS_ins.h"
#include "GINS_ekf.h"
#include "Att_tran.h"
#include "GINS_process_lib.h"
#include <stdlib.h>
#include <time.h>
#include <string>
#include <vector>
#include <algorithm> 
#include <stdarg.h>
#define NUMX 21
#define NUMV 13
#define RE_WGS84    6378137.0           // earth semimajor axis (WGS84) (m)
#define FE_WGS84    (1.0/298.257223563) // earth flattening (WGS84)
#define PI		    3.14159265358979
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define glv_g0     9.7803267714
#define glv_g      9.80665    
#define GNSS_PERIODE_MS 200
#define RE  6378137.0
#define SQR(x)      ((x)*(x))
#define SQ(X)       ((X)*(X))
#define DEG_0_360(x)       {if ((x) > 360) (x) -= 360;    else if ((x) < 0)	   (x) += 360; }
#define DEG_NEG180_180(x)  {if ((x) > 180) (x) -= 360;    else if ((x) < -180) (x) += 360; }
//template<typename T>
//using malloc_allocator = class std::allocator<T>;
using namespace std;
const static double gpst0[] = { 1980,1, 6,0,0,0 }; /* gps time reference */
const static double gst0[] = { 1999,8,22,0,0,0 }; /* galileo system time reference */
const static double bdt0[] = { 2006,1, 1,0,0,0 }; /* beidou time reference */



typedef struct {        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t 从1970.01.01 0秒到现在的秒数 long int*/
	double sec;         /* fraction of second under 1 s */
} gtime_t;
const static double leaps[][7] = { /* leap seconds {y,m,d,h,m,s,utc-gpst,...} */
	{ 2017,1,1,0,0,0,-18 },
	{ 2015,7,1,0,0,0,-17 },
	{ 2012,7,1,0,0,0,-16 },
	{ 2009,1,1,0,0,0,-15 },
	{ 2006,1,1,0,0,0,-14 },
	{ 1999,1,1,0,0,0,-13 },
	{ 1997,7,1,0,0,0,-12 },
	{ 1996,1,1,0,0,0,-11 },
	{ 1994,7,1,0,0,0,-10 },
	{ 1993,7,1,0,0,0, -9 },
	{ 1992,7,1,0,0,0, -8 },
	{ 1991,1,1,0,0,0, -7 },
	{ 1990,1,1,0,0,0, -6 },
	{ 1988,1,1,0,0,0, -5 },
	{ 1985,7,1,0,0,0, -4 },
	{ 1983,7,1,0,0,0, -3 },
	{ 1982,7,1,0,0,0, -2 },
	{ 1981,7,1,0,0,0, -1 }
};

class CGLV
{
public:
	double Re, f, g0, wie, g;
	double mg, ug, deg, min, sec, hur, ppm, ppmpsh;
	double dph, dpsh, dphpsh, ugpsh, ugpsHz, mpsh, mpspsh, secpsh;
	double ep[6];  // ymdhms
	double kmph;

	CGLV(double Re = 6378137.0, double f = (1.0 / 298.257), double wie0 = 7.2921151467e-5, double g0 = 9.7803267714);
};
class CEarth
{
public:
	double a, b;
	double f, e, e2;
	double wie;

	double sl, sl2, sl4, cl, tl, RMh, RNh, clRNh, f_RMh, f_RNh, f_clRNh;
	double wnie[3], wnen[3], wnin[3], gn[3], gcc[3];

	//CEarth(double a0 = glv.Re, double f0 = glv.f, double g0 = glv.g0);
	//CEarth& operator=(const CEarth& eth);
	//void UpdateP(double pos[]);
	void UpdatePV(double pos[], double vn[]);
};
class GINS_INS
{
public:
	CEarth eth;
	double qnb[4];
	double Cnb[3 * 3];
	/*dsf90 add,2018.4.25,安装误差相关变量, m系: mobile 坐标系*/
	double PRY_Install[3];
	double qmb[4];
	double Cmb[3 * 3];
	double Cbm[3 * 3];
	double vm_car[3];
	double att_car[3];
	double Cnm[3 * 3];
	double tDelay;
	/*end add*/
	double att[3], vn[3], pos[3], vnL[3], posL[3], vnL2[3], posL2[3];
	double eb[3], db[3], lever[3], lever2[3], Kg[3 * 3], Ka[3 * 3];
	double fn[3], an[3], web[3], wnb[3], wib[3], fb[3], ab[3], am[3], wim[3], fm[3];

	double Mpv[3 * 3], Mpvvn[3], MpvCnb[9], CW[9];
	double wm_1[3], vm_1[3];

	double imutimetarge;
	double dual_yaw;  /*双天线航向*/
	double dual_yaw_h;/*双天线航向观测量*/

					  //INS(void) { imutimetarge = 0; };
	void INS(double Att[], double Vn[], double Pos[], double EB[], double DB[], double Lever[]);
	//CSINS& operator=(const CSINS& ins);
	void Init(double Att[], double Vn[], double Pos[], double EB[], double DB[], double Lever[], double Lever2[], double PRY_install[]);
	void Update(double wm[], double vm[], double dtime);
	//void Lever();
	void Lever(double pos1[], double vn1[], double pos2[], double vn2[], double lever[]);
	//void SetOutLever(double Lever2[]);
};



class GINS_KF
{
public:
	int ROW, COL, OPT;
	double* xk;//状态量
	double* Pxk;//状态量协方差
	double* Phi;//状态转移矩阵
	double* Qk;//状态噪声矩阵
	double* Gk;
	double* Hk;//观测矩阵
	double* Rk;//观测噪声矩阵
	double* xkpre;
	double* dpos;
	double* denu;
	int  xflag;
	int  zflag;
	bool bGnssUpdata;
	double kf_Q_init[NUMX];//状态噪声初值
	double kf_P_init[NUMX];//状态协方差初值

	double davp[9], GB[3], AB[3], GW[3], AW[3], GS[3], AS[3];
	double gyro_std[3];
	double acc_std[3];
	double gyro_bias_walk[3];
	double acc_bias_walk[3];


	/*内存空间释放*/
	void kffree();
	/*滤波器初始化,状态转移矩阵的初始，状态噪声的初始*/
	void kfinit();
	/*状态转移矩阵*/
	void upPhi(GINS_INS& ins, double dt);
	/*观测矩阵赋值*/
	void upHk(GINS_INS& ins, double *hk);
	/*时间更新*/
	void TUpdate(double dt, int option = 0);
	/*量测更新*/
	void MUpdate(double ZK[]);
	/*反馈修正*/
	void Feedback(GINS_INS& ins, double scater = 1.0, int option = 0);
};
/*GINS中间数据*/
class Process_Data
{
public:
	double gpstimetarge;
	int week;
	double ep[6];  // ymdhms
	double pos[3]; // gnss pos(blh)
	double undulation;
	double vn[3];  // gnss vn（ENU）
	int stat, ns;   // sol stat/ sat num
	int snsum, nsused;   // sat signal sum/used sat num
	double hdop;   // HDOP
	int m0;
	double age;    // 0/diffage
	double GPV_RK[36];
	double GA_RK[3];
	double yaw_U62R;
	double yaw_var_U62R;
	double lever;
	double heading2;/*双天线航向*/
	double heading2_std;

	// ins data
	int num;
	double time;   // imu pre time
	double imutimetarge;
	double acc[3]; // xyz加速度(mg)
	double gyo[3]; // xyz陀螺(deg/s)
	double mag[3]; // Gauss
	double temper;
	double acc_iir[3]; // xyz加速度(mg),滤波
	double gyo_iir[3]; // xyz陀螺(deg/s),滤波
	double mag_iir[3]; // Gauss,滤波

	double syst;
	double calt;   // position time
	bool bUpdate, bValid;
	bool bGPSavail, bPPSavail;  //GPS available（pos/vel）, PPSavailable
	bool bOnlyVelUp;     //only veloity update,position don't update
	bool bMEMSavail;
	bool bODOavail;      //odometry updata
	double gnss_speed;
	double heading;

	void Init();
	//void Rest();
	//CLCData& operator=(const CLCData& lcdata);
	//void getHMS(double gpstimetarge);
	//void getPOS_deg(double lat, double lon, double hgt);
	//void getPOS_rad(double lat, double lon, double hgt);
	//void imuConvert();
};


/*GINS解算类：INS+EKF*/
class GINS_Process
{
public:
	double dt, tpre, dt_total;
	int Row, Col, Opt;
	GINS_INS ins;
	GINS_INS inspre;
	GINS_INS inspre_forPPS;
	GINS_INS inspre_forStatic;
	double dGnssHeading2_forStatic;
	GINS_KF kf;

	bool bFileSave;
	double dInitTimesMin;

	int iGilcRunMode; /*0:Normal; 1:Calibrate;*/
	int iInitTimes_ms;

	int c;
	bool bAlign;
	bool bStatic;
	int iDriverMode; /*add by dsf90, 2018.5.31*/
	bool bInstallOk; /*add by dsf90, 2018.6.6*/
	double bGnssTimeOut;
	bool bStaticpre;
	int bTurn;
	int bFinshMoveAlign;
	int num_FinshMoveAlign;
	int num_GNSSrestore;
	int num_GNSSmupdate;
	double preheading;

	int num_GPSskip;
	int num_GPSloose;
	int num_ContinueFloat;
	int num_ContinueFix;
	int upmodel;
	double wmpre[3], vmpre[3];

	double pospre[3];

	double dpos_sync[3];
	double dvn_sync[3];
	double dposb_sync[3];
	bool bPPSSync;

	//void Init(void);
	//void IMUcone(Process_Data ilcd, double wmm[3], double vmm[3]);
	//void correctSideslip(void);
	void loadPPSSyncInsData(Process_Data &ilcd, GINS_INS &ppsins);
	//void getDualAntBias(Process_Data &ilcd, double dDifYaw);
	//void getOdoHeadingBias(Process_Data &ilcd, double dDifHeading);
	//void KfInitOver();
	//void updateKfInitStatus(Process_Data &ilcd);
	//void GnssIntegrity(Process_Data &ilcd, double dheading, double dvn[3]);
	//void OdoIntegrity(Process_Data &ilcd);
	int GINS_P2(Process_Data ilcd);
	//void setlever(double leverdt[]);
	int ZUpdate(Process_Data ilcd);
};

struct gpos
{
	double time;
	double lat, lon, hig;
	double ve, vn, vu;
	double yaw;
	int state;
};
typedef struct gpos gpos_t;
void equalgpos(gpos_t* GP, Process_Data* lcdata);

class GINS_Align
{
public:
	bool bFinshAlign;
	bool bStatic;
	double PRY_Install[3]; /*安装误差初始值*/
	double Att[3], Vn[3], Pos[3];//INS推算初始值
	double VnL[3], PosL[3]; /*天线位置速度、位置*/
	int gnssstate;
	double Pgposvn[36];
	unsigned int ngnss, Ngnss;
	int nspeed;
	vector <gpos_t> heading_v;
	vector<double> yaw_gnss;

	bool CalAtt2(Process_Data& ilcd);
	bool CalAtt(Process_Data& ilcd, int opt = 0);              //根据位置计算航向
	bool KinmateAlign(Process_Data& ilcd, GINS_Process& gipro);
};





class GINS_YM
{
public:
	GINS_raw_t stRaw;//gins 原始数据
	GINS_raw_t stRaw_tmp; //gins 临时数据
	GINS_cfg_double GI_cfg;
	GINS_Process GI_pro;//gins解算数据
	GINS_Align GI_align;
	Process_Data GI_pd, pre_GI_pd;//gins中间数据
								  //DebugFile debugfile;
	int raw_cnt;
	int dbg_cnt;
	int rst_cnt;
	bool GetGPS;//gps状态
	bool bAlign;
	double eb[3], db[3];
	char cGilcInitMsg[2048];

	double dInstallMat[9];
	double dInstallAttCfg[3];

	double dGnss2OutPointLever[3];

	//GilcProcess(void) {};

	int GINS_Init(GINS_cfg_t* cfgdata);
	//void GILC_Update_Status(GINS_result_t* pstOut, int iGilcStatus, int iGnssStatus, bool bDualAntAvail);
	void GINS_GnssRaw_Correct(GINS_raw_t *pRaw);
	void GINS_ImuAxis_Correct(GINS_raw_t *pRaw);
	void GINS_ekf_data_ready(GINS_cfg_double *GI_cfg);
	int GINS_Rawdata_Quality(GINS_raw_t *pRaw);
	int GINS_data_correct(GINS_raw_t *pRaw);
	int GINS_PROCESS_Lib(GINS_raw_t* pstRaw, GINS_result_t* pstOut);
};

class test
{
public :
	double data[3];
	int flag;
	void init(void);
};


extern double timediff(gtime_t t1, gtime_t t2);
extern gtime_t timeadd(gtime_t t, double sec);
//extern int str2time(const char *s, int i, int n, gtime_t *t);
extern  gtime_t gpst2utc(gtime_t t);
extern gtime_t epoch2time(const double *ep);
extern void time2epoch(gtime_t t, double *ep);
extern gtime_t gpst2time(int week, double sec);
//extern double time2gpst(gtime_t t, int *week);

#endif
