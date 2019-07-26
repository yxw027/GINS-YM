#ifndef _GINS_PROCESS_H
#define _GINS_PROCESS_H
#include "GINS_ins.h"
#include "GINS_ekf.h"
#include "Att_tran.h"
#include "GINS_process_lib.h"
#include "IMU_Filter.h"
#include "GINS_test.h"
#include "GINS_DataSave.h"
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


#define UPDATE_Z_ATT_X    0x0001
#define UPDATE_Z_ATT_Y    0x0002
#define UPDATE_Z_ATT_Z    0x0004
#define UPDATE_Z_ATT_XYZ  0x0007

#define UPDATE_Z_VER_X    0x0008
#define UPDATE_Z_VER_Y    0x0010
#define UPDATE_Z_VER_Z    0x0020
#define UPDATE_Z_VER_XYZ  0x0038

#define UPDATE_Z_POS_X    0x0040
#define UPDATE_Z_POS_Y    0x0080
#define UPDATE_Z_POS_Z    0x0100
#define UPDATE_Z_POS_XYZ  0x01C0

#define UPDATE_Z_CONS_VER_X   0x0200
#define UPDATE_Z_CONS_VER_Y   0x0400
#define UPDATE_Z_CONS_VER_Z   0x0800
#define UPDATE_Z_CONS_VER_XZ  0x0A00
#define UPDATE_Z_CONS_VER_XYZ 0x0D00


template<typename T>
using malloc_allocator = class std::allocator<T>;
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
	int imu_raw_cnt;
	double hdop;   // HDOP
	int m0;
	double age;    // 0/diffage
	double GPV_RK[36];
	double GA_RK[3];
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

	bool bUpdate, bValid;
	bool bGPSavail, bPPSavail;  //GPS available（pos/vel）, PPSavailable
	bool bMEMSavail;
	double gnss_speed;
	double heading;

	void Init(void);
	//void Rest();
	//CLCData& operator=(const CLCData& lcdata);
	//void getHMS(double gpstimetarge);
	//void getPOS_deg(double lat, double lon, double hgt);
	//void getPOS_rad(double lat, double lon, double hgt);
	//void imuConvert();
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
	double wmpre[3], vmpre[3];
					  //INS(void) { imutimetarge = 0; };
	void INS(double Att[], double Vn[], double Pos[], double EB[], double DB[], double Lever[]);
	//CSINS& operator=(const CSINS& ins);
	int INS_process(Process_Data ilcd,double dt);
	void Init(double Att[], double Vn[], double Pos[], double EB[], double DB[], double Lever[], double Lever2[], double PRY_install[]);
	void Update(double wm[], double vm[], double dtime);
	void Lever();
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


	GINS_KF(int row = NUMX, int col = NUMV, int opt = 156);
	GINS_KF& operator=(const GINS_KF& kftemp);

	void GINS_KF_malloc(int row, int col,GINS_KF *kf_tmp);
	//GINS_KF& operator=(const GINS_KF& kftemp);
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
	/*参数设置为常数*/
	void setRk_constraint(void);
	/*权重处理*/
	void downgrade_car(Process_Data ilcd, double denu[3], double posrk[3]);
	/*降权处理*/
	void downgrade(Process_Data ilcd, int scater, double posrk[3]);
	void resetRk(Process_Data ilcd);
	void MUpadte(GINS_INS& ins, double ZK[], int zflag = 0xffff);

};
/*GINS中间数据*/

void MUpdate_Variable(int ROW, int COL, double Hk[], double Rk[], double ZK[], double xk[], double Pxk[], int zflag);
void MUpdate_True(int ROW, int COL, double Hk[], double Rk[], double ZK[], double xk[], double Pxk[]);

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
	GINS_KF kf_tmp;
	bool bFileSave;
	double dInitTimesMin;

	int iGilcRunMode; /*0:Normal; 1:Calibrate;*/
	int iInitTimes_ms;

	int c;
	bool kf_init;
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
	bool busegnssvn;
	bool bgnssskip;
	int num_GPSskip;
	int num_GPSloose;
	int num_ContinueFloat;
	int num_ContinueFix;
	int upmodel;
	bool bGnssLost;
	int  bGnssLostNum;
	int bGnssNum;
	double pospre[3];

	double dpos_sync[3];
	double dvn_sync[3];
	double dposb_sync[3];
	bool bPPSSync;
	vector<gpos, malloc_allocator<gpos> >pregpos;

	void Init(void);
	void correctSideslip(void);
	void loadPPSSyncInsData(Process_Data &ilcd, GINS_INS &ppsins);
	//void getDualAntBias(Process_Data &ilcd, double dDifYaw);
	//void KfInitOver();
	//void updateKfInitStatus(Process_Data &ilcd);
	void GnssIntegrity(Process_Data &ilcd, double dheading, double dvn[3]);
	int GINS_P2(Process_Data ilcd);
	//void setlever(double leverdt[]);
	int ZUpdate(Process_Data ilcd);
};



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
	void init(void);
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
	int imu_raw_cnt;
	int dbg_cnt;
	int rst_cnt;
	bool GetGPS;//gps状态
	bool bAlign;
	double eb[3], db[3];
	char cGilcInitMsg[2048];
	vector<double> IMU_gyro_winx;
	vector<double> IMU_gyro_winy;
	vector<double> IMU_gyro_winz;
	double gyro_bias[3];
	bool bias_init;
	double dInstallMat[9];
	double dInstallAttCfg[3];

	double dGnss2OutPointLever[3];

	//GilcProcess(void) {};

	int GINS_Init(GINS_cfg_t* cfgdata);
	int IMU_static_bias(void);
	void GINS_result(GINS_result_t *presult,int GINS_flag);
	//void GILC_Update_Status(GINS_result_t* pstOut, int iGilcStatus, int iGnssStatus, bool bDualAntAvail);
	void GINS_GnssRaw_Correct(GINS_raw_t *pRaw);
	void GINS_ImuAxis_Correct(GINS_raw_t *pRaw);
	void GINS_ekf_data_ready(GINS_cfg_double *GI_cfg);
	int GINS_Rawdata_Quality(GINS_raw_t *pRaw);
	int GINS_data_correct(GINS_raw_t *pRaw);
	int GINS_PROCESS_Lib(GINS_raw_t* pstRaw, GINS_result_t* pstOut);
};
extern double timediff(gtime_t t1, gtime_t t2);
extern gtime_t timeadd(gtime_t t, double sec);
//extern int str2time(const char *s, int i, int n, gtime_t *t);
extern  gtime_t gpst2utc(gtime_t t);
extern gtime_t epoch2time(const double *ep);
extern void time2epoch(gtime_t t, double *ep);
extern gtime_t gpst2time(int week, double sec);
//extern double time2gpst(gtime_t t, int *week);
void IMU_Filter(Process_Data ilcd);
bool busegnssvel_car(double gnssvel[3], double dvel[3], double dheading);
bool bgnssskip_car(vector<gpos_t, malloc_allocator<gpos_t> > v_gps, double iposcur[3], int numins);


void SaveRst(GINS_Process *gipro, Process_Data *ilcd);
void printf_posLC(FILE *fp, GINS_INS *ins, Process_Data *ilcd);
void printf_kf(FILE *fp, GINS_KF& kf, double gpstimetarget);
void printf_prosess(FILE *fp, GINS_Process *gipro, Process_Data *ilcd);
void printf_posAnt(FILE *fp, GINS_INS *ins, Process_Data *ilcd);
int GINS_string_decode(char *buff, GINS_raw_t *pRaw);

#endif
