#include "GINS_ekf.h"
void GINS_KF::kfinit()
{
	memset(xk, 0, sizeof(double)*ROW * 1);
	memset(xkpre, 0, sizeof(double)*ROW * 1);
	memset(dpos, 0, sizeof(double) * 3 * 1);
	memset(denu, 0, sizeof(double) * 3 * 1);
	memset(Phi, 0, sizeof(double)*ROW*ROW);
	memset(Hk, 0, sizeof(double)*COL*ROW);
	memset(Rk, 0, sizeof(double)*COL * 1);
	memset(Pxk, 0, sizeof(double)*ROW*ROW);
	memset(Qk, 0, sizeof(double)*ROW*ROW);
	memset(Gk, 0, sizeof(double)*ROW*ROW);

	for (int i = 0; i<ROW; i++)
	{
		Pxk[i*ROW + i] = SQR(kf_P_init[i]);//状态协方差初始化
	}


	for (int i = 0; i<ROW; i++)
	{
		Qk[i*ROW + i] = SQR(kf_Q_init[i]);//状态噪声初始化
	}
/*
	Gk[0 * ROW + 0] = -1; Gk[0 * ROW + 1] = 0; Gk[0 * ROW + 2] = 0;
	Gk[1 * ROW + 0] = 0; Gk[1 * ROW + 1] = -1; Gk[1 * ROW + 2] = 0;
	Gk[2 * ROW + 0] = 0; Gk[2 * ROW + 1] = 0; Gk[2 * ROW + 2] = -1;
	Gk[3 * ROW + 3] = 1; Gk[3 * ROW + 4] = 0; Gk[3 * ROW + 5] = 0;
	Gk[4 * ROW + 3] = 0; Gk[4 * ROW + 4] = 1; Gk[4 * ROW + 5] = 0;
	Gk[5 * ROW + 3] = 0; Gk[5 * ROW + 4] = 0; Gk[5 * ROW + 5] = 1;
	Gk[9 * ROW + 9] = 1.0;
	Gk[10 * ROW + 10] = 1.0;
	Gk[11 * ROW + 11] = 1.0;
	Gk[12 * ROW + 12] = 1.0;
	Gk[13 * ROW + 13] = 1.0;
	Gk[14 * ROW + 14] = 1.0;
	*/
}

void GINS_KF::kffree()
{
	if (xk != NULL)
	{
		free(xk); xk = NULL;
	}
	if (xkpre != NULL)
	{
		free(xkpre); xkpre = NULL;
	}
	if (dpos != NULL)
	{
		free(dpos); dpos = NULL;
	}
	if (denu != NULL)
	{
		free(denu); denu = NULL;
	}
	if (Phi != NULL)
	{
		free(Phi); Phi = NULL;
	}
	if (Hk != NULL)
	{
		free(Hk); Hk = NULL;
	}
	if (Pxk != NULL)
	{
		free(Pxk); Pxk = NULL;
	}
	if (Qk != NULL)
	{
		free(Qk); Qk = NULL;
	}
	//if (Gk != NULL)
	//{
	//	free(Gk); Gk = NULL;
	//}
	if (Rk != NULL)
	{
		free(Rk); Rk = NULL;
	}
}

void GINS_KF::TUpdate(double dt, int option)
{
	if (!option)
	{
		double* temp = (double*)__ml_zero(sizeof(double)*ROW * 1);
		double* temp1 = (double*)__ml_zero(sizeof(double)*ROW*ROW);
		double* temp2 = (double*)__ml_zero(sizeof(double)*ROW*ROW);

		/*dsf90:xk  = Phi X xk*/
		Mat_mul(Phi, xk, ROW, ROW, 1, temp);
		Mat_equal(temp, ROW, 1, xk);
		Mat_mul(Phi, Pxk, ROW, ROW, ROW, temp1);
		Mat_tran(Phi, ROW, ROW, temp2);
		Mat_mul(temp1, temp2, ROW, ROW, ROW, Pxk);
#if 1
		/*dsf90:Pxk = Phi X Pxk X Mt(Phi) + Qk*dt*/
		Mat_mulb(Qk, ROW, ROW, dt, temp1);
		Mat_add(Pxk, temp1, Pxk, ROW, ROW);
#else   
		/*dsf90:修正：Pxk = Phi X Pxk X Mt(Phi) + Qk*dt^2*/
		Mat_mul(Qk, ROW, ROW, dt*dt, temp1);
		Mat_add(Pxk, temp1, ROW, ROW);
#endif
		free(temp);  temp = NULL;
		free(temp1); temp1 = NULL;
		free(temp2); temp2 = NULL;
	}
	else  // 考虑GK
	{
		double* temp = (double*)__ml_zero(sizeof(double)*ROW * 1);
		double* temp1 = (double*)__ml_zero(sizeof(double)*ROW*ROW);
		double* temp2 = (double*)__ml_zero(sizeof(double)*ROW*ROW);
		double* temp3 = (double*)__ml_zero(sizeof(double)*ROW*ROW);

		/*dsf90:xk  = Phi X xk*/
		Mat_mul(Phi, xk, ROW, ROW, 1, temp);
		Mat_equal(temp, ROW, 1, xk);
		Mat_mul(Phi, Pxk, ROW, ROW, ROW, temp1);
		Mat_tran(Phi, ROW, ROW, temp2);
		Mat_mul(temp1, temp2, ROW, ROW, ROW, Pxk);
#if 0
		/*dsf90:Pxk = Phi X Pxk X Mt(Phi) + Gk X (Qk*dt) X Gk*/
		Mmuln(Qk, ROW, ROW, dt, temp1);
		Mmulnm(Gk, temp1, ROW, ROW, ROW, temp2);
		Mtn(Gk, ROW, ROW, temp1);
		Mmulnm(temp2, Gk, ROW, ROW, ROW, temp3);
#else
		/*dsf90:修正：Pxk = Phi X Pxk X Mt(Phi) + dt^2 * Gk X Qk X Mt(Gk)*/
		Mat_mulb(Qk, ROW, ROW, (dt*dt), temp1);
		Mat_mul(Gk, temp1, ROW, ROW, ROW, temp2);
		Mat_tran(Gk, ROW, ROW, temp1);
		Mat_mul(temp2, temp1, ROW, ROW, ROW, temp3);
#endif
		Mat_add(Pxk, temp3, Pxk, ROW, ROW);

		free(temp);   temp = NULL;
		free(temp1);  temp1 = NULL;
		free(temp2);  temp2 = NULL;
		free(temp3);  temp3 = NULL;
	}
}


void GINS_KF::MUpdate(double ZK[])
{
	double* xkk_1 = (double*)__ml_zero(sizeof(double)*ROW * 1);
	double* Pxkk_1 = (double*)__ml_zero(sizeof(double)*ROW*ROW);
	double* Pxykk_1 = (double*)__ml_zero(sizeof(double)*ROW*COL);
	double* Pykk_1 = (double*)__ml_zero(sizeof(double)*COL*COL);
	double* Kk = (double*)__ml_zero(sizeof(double)*ROW*COL);
	double* ykk_1 = (double*)__ml_zero(sizeof(double)*COL * 1);
	double* dxk = (double*)__ml_zero(sizeof(double)*ROW * 1);
	double* Hkt = (double*)__ml_zero(sizeof(double)*ROW*COL);
	double* invPy = (double*)__ml_zero(sizeof(double)*COL*COL);
	double* KPyKt = (double*)__ml_zero(sizeof(double)*ROW*ROW);

	/*dsf90:Phi = F*/
	/*dsf90:xk	= X*/
	/*dsf90:Pxk	= P*/
	/*dsf90:Hk	= H*/
	/*dsf90:ZK	= Z*/
	/*dsf90:Rk	= R*/

	Mat_equal(xk, ROW, 1, xkk_1);
	Mat_equal(Pxk, ROW, ROW, Pxkk_1);

	/*dsf90:Pxykk_1	= Pxk X Mt(Hk)*/
	/*dsf90:Pykk_1	= Hk X Pxk X Mt(Hk) + Rk*/
	/*dsf90:ykk_1	= ZK - Hk X xk*/
	/*dsf90:dxk	    = Kk X (ZK - Hk X xk)*/

	/*dsf90:Kk	    = Pxk X Mt(Hk) X inv(Hk X Pxk X Mt(Hk) + Rk)*/
	/*dsf90:xk  	= xk + Kk X (ZK - Hk X xk)*/

	/*dsf90:Hkt	= Mt(Kk)*/
	/*dsf90:Pxykk_1	= Kk X (Hk X Pxk X Mt(Hk) + Rk)*/
	/*dsf90:KPyKt	= Kk X (Hk X Pxk X Mt(Hk) + Rk) X Mt(Hk)*/
	/*dsf90:Pxk  	= Pxk - Kk X (Hk X Pxk X Mt(Hk) + Rk) X Mt(Hk)*/
	/*dsf90:Pxk  	= 0.5*(Pxk + Mt(Pxk))*/

	Mat_tran(Hk, COL, ROW, Hkt);
	Mat_mul(Pxkk_1, Hkt, ROW, ROW, COL, Pxykk_1);
	Mat_mul(Hk, Pxykk_1, COL, ROW, COL, Pykk_1);
	Mat_add(Pykk_1, Rk, Pykk_1, COL, COL);
	Mat_inv(Pykk_1, COL, invPy);
	Mat_mul(Pxykk_1, invPy, ROW, COL, COL, Kk);
	Mat_mul(Hk, xkk_1, COL, ROW, 1, ykk_1);
	Mat_min(ZK, ykk_1, ykk_1, COL, 1);
	Mat_mul(Kk, ykk_1, ROW, COL, 1, dxk);
	Mat_add(xkk_1, dxk, xk, ROW, 1);

	free(xkk_1);
	free(Pxkk_1);
	free(Pxykk_1);
	free(Pykk_1);
	free(Kk);
	free(ykk_1);
	free(dxk);
	free(Hkt);
	free(invPy);
	free(KPyKt);
	xkk_1 = NULL;
	Pxkk_1 = NULL;
	Pxykk_1 = NULL;
	Pykk_1 = NULL;
	Kk = NULL;
	ykk_1 = NULL;
	dxk = NULL;
	Hkt = NULL;
	invPy = NULL;
	KPyKt = NULL;
}


void GINS_KF::Feedback(GINS_INS& ins, double scater, int option)
{
	double datt[3], dvn[3], dpos[3], deb[3], ddb[3], dPRY[3], dlever[3];
	//double dtelay;
	double* xk_f = (double*)__ml_zero(sizeof(double)*ROW);
	Mat_mulb(xk, ROW, 1, scater, xk_f);

	datt[0] = xk_f[0]; datt[1] = xk_f[1]; datt[2] = xk_f[2];
	dvn[0] = xk_f[3];  dvn[1] = xk_f[4];  dvn[2] = xk_f[5];
	dpos[0] = xk_f[6]; dpos[1] = xk_f[7]; dpos[2] = xk_f[8];
	deb[0] = xk_f[9];  deb[1] = xk_f[10]; deb[2] = xk_f[11];
	ddb[0] = xk_f[12]; ddb[1] = xk_f[13]; ddb[2] = xk_f[14];
	if (ROW >= 18)
	{
		dPRY[0] = xk_f[15]; dPRY[1] = xk_f[16]; dPRY[2] = xk_f[17];
	}
	if (ROW >= 21)
	{
		dlever[0] = xk_f[18]; dlever[1] = xk_f[19]; dlever[2] = xk_f[20];
	}


	qdelphi(ins.qnb, datt);
	Mat_min(ins.vn, dvn, ins.vn, 3, 1);//
	Mat_min(ins.pos, dpos, ins.pos, 3, 1);//
	Mat_add(ins.eb, deb, ins.eb, 3, 1); // 陀螺零偏
	Mat_add(ins.db, ddb, ins.db, 3, 1);
	if (ROW >= 18 && (xflag&(7 << 15)))
	{
#if 1
		qdelphi(ins.qmb, dPRY);
		q2att(ins.qmb, ins.PRY_Install);
		q2mat(ins.qmb, ins.Cmb);
#else /*dsf90:效果一致*/
		Madd(ins.PRY_Install, dPRY, 3, 1);
		a2qua(ins.PRY_Install, ins.qmb);
		a2mat(ins.PRY_Install, ins.Cmb);
#endif
		double cbn[9], vb[3];
		Mat_tran(ins.Cnb, 3, 3, cbn);
		Mat_mul(cbn, ins.vn, 3, 3, 1, vb);
		Mat_mul(ins.Cmb, vb, 3, 3, 1, ins.vm_car);

		Mat_tran(ins.Cmb, 3, 3, ins.Cbm);
		Mat_mul(ins.Cnb, ins.Cbm, 3, 3, 3, ins.Cnm);
		m2att(ins.Cnm, ins.att_car);
	}

	if (ROW >= 21 && (xflag&(7 << 18)))
		Mat_add(ins.lever, dlever, ins.lever,3, 1);
	/*
	if(ROW>=23 && (xflag&(1<<22)))
	ins.tDelay+=dtelay;
	*/
	//ins.Lever();

	for (int i = 0; i<ROW; i++)
	{
		xk[i] -= xk_f[i];
	}

	free(xk_f);
}


void GINS_KF::upPhi(GINS_INS& ins, double dt)
{
	Mat_equal(Phi, ROW, ROW, 0);

	double sl, cl, tl, secl, secl2, f_RMh, f_RNh, f_RMh2, f_RNh2, f_clRNh;
	sl = sin(ins.pos[0]); cl = cos(ins.pos[0]);
	tl = sl / cl; secl = 1 / cl;
	secl2 = sl*sl;
	f_RMh = 1.0 / ins.eth.RMh; f_RNh = 1.0 / ins.eth.RNh;
	f_RMh2 = f_RMh*f_RMh; f_RNh2 = f_RNh*f_RNh;
	f_clRNh = 1.0 / ins.eth.clRNh;

	double vn[3] = { 0.0 };
	Mat_equal(ins.vn, 3, 1, vn);
	double vE_clRNh, vE_RNh2, vN_RMh2;
	vE_clRNh = vn[0] * f_clRNh; vE_RNh2 = vn[0] * f_RNh2; vN_RMh2 = vn[1] * f_RMh2;

	double Mp1[3 * 3] = { 0.0 };
	Mp1[1 * 3 + 0] = -ins.eth.wnie[2];
	Mp1[2 * 3 + 0] = ins.eth.wnie[1];

	double Mp2[3 * 3] = { 0.0 };
	Mp2[0 * 3 + 2] = vN_RMh2;
	Mp2[1 * 3 + 2] = -vE_RNh2;
	Mp2[2 * 3 + 0] = vE_clRNh*secl;  Mp2[2 * 3 + 2] = -vE_RNh2*tl;

	double Avn[3 * 3] = { 0.0 };
	Avn[0 * 3 + 1] = -vn[2]; Avn[0 * 3 + 2] = vn[1];
	Avn[1 * 3 + 0] = vn[2];                     Avn[1 * 3 + 2] = -vn[0];
	Avn[2 * 3 + 0] = -vn[1]; Avn[2 * 3 + 1] = vn[0];

	double Awn[3 * 3] = { 0.0 };
	Awn[0 * 3 + 1] = -(ins.eth.wnie[2] + ins.eth.wnin[2]); Awn[0 * 3 + 2] = ins.eth.wnie[1] + ins.eth.wnin[1];
	Awn[1 * 3 + 0] = ins.eth.wnie[2] + ins.eth.wnin[2];                                                   Awn[1 * 3 + 2] = -(ins.eth.wnie[0] + ins.eth.wnin[0]);
	Awn[2 * 3 + 0] = -(ins.eth.wnie[1] + ins.eth.wnin[1]); Awn[2 * 3 + 1] = ins.eth.wnie[0] + ins.eth.wnin[0];

	double Maa[3 * 3] = { 0.0 };
	Maa[0 * 3 + 1] = ins.eth.wnin[2];  Maa[0 * 3 + 2] = -ins.eth.wnin[1];
	Maa[1 * 3 + 0] = -ins.eth.wnin[2];                              Maa[1 * 3 + 2] = ins.eth.wnin[0];
	Maa[2 * 3 + 0] = ins.eth.wnin[1];  Maa[2 * 3 + 1] = -ins.eth.wnin[0];

	double Mav[3 * 3] = { 0.0 };
	Mav[0 * 3 + 1] = -f_RMh;
	Mav[1 * 3 + 0] = f_RNh;
	Mav[2 * 3 + 0] = f_RNh*tl;

	double Map[3 * 3] = { 0.0 };
	Mat_add(Mp1, Mp2, Map, 3, 3);

	double Mva[3 * 3] = { 0.0 };
	Mva[0 * 3 + 1] = -ins.fn[2]; Mva[0 * 3 + 2] = ins.fn[1];
	Mva[1 * 3 + 0] = ins.fn[2];                         Mva[1 * 3 + 2] = -ins.fn[0];
	Mva[2 * 3 + 0] = -ins.fn[1]; Mva[2 * 3 + 1] = ins.fn[0];

	double temp[3 * 3] = { 0.0 };
	double Mvv[3 * 3] = { 0.0 };
	Mat_mul(Avn, Mav, 3, 3, 3, temp);
	Mat_min(temp, Awn, Mvv, 3, 3);

	double Mvp[3 * 3] = { 0.0 };
	Mat_add(Mp1, Map, temp, 3, 3);
	Mat_mul(Avn, temp, 3, 3, 3, Mvp);

	double g0 = 9.7803267714;
	double scl = sl*cl;

	Mvp[2 * 3 + 0] = Mvp[2 * 3 + 0] - g0*(5.27094e-3 * 2 * scl + 2.32718e-5 * 4 * secl2*scl);
	Mvp[2 * 3 + 2] = Mvp[2 * 3 + 2] + 3.086e-6;

	double Mpv[3 * 3] = { 0.0 };

	Mat_equal(ins.Mpv, 3, 3, Mpv);

	double Mpp[3 * 3] = { 0.0 };
	Mpp[0 * 3 + 2] = -vN_RMh2;
	Mpp[1 * 3 + 0] = vE_clRNh*tl;  Mpp[1 * 3 + 2] = -vE_RNh2*secl;

	Phi[0 * ROW + 1] = Maa[0 * 3 + 1]; Phi[0 * ROW + 2] = Maa[0 * 3 + 2];
	Phi[1 * ROW + 0] = Maa[1 * 3 + 0];                         Phi[1 * ROW + 2] = Maa[1 * 3 + 2];
	Phi[2 * ROW + 0] = Maa[2 * 3 + 0]; Phi[2 * ROW + 1] = Maa[2 * 3 + 1];

	Phi[0 * ROW + 4] = Mav[0 * 3 + 1];
	Phi[1 * ROW + 3] = Mav[1 * 3 + 0];
	Phi[2 * ROW + 3] = Mav[2 * 3 + 0];

	Phi[0 * ROW + 8] = Map[0 * 3 + 2];
	Phi[1 * ROW + 6] = Map[1 * 3 + 0];                           Phi[1 * ROW + 8] = Map[1 * 3 + 2];
	Phi[2 * ROW + 6] = Map[2 * 3 + 0];                           Phi[2 * ROW + 8] = Map[2 * 3 + 2];

	Phi[0 * ROW + 9] = -ins.Cnb[0 * 3 + 0]; Phi[0 * ROW + 10] = -ins.Cnb[0 * 3 + 1]; Phi[0 * ROW + 11] = -ins.Cnb[0 * 3 + 2];
	Phi[1 * ROW + 9] = -ins.Cnb[1 * 3 + 0]; Phi[1 * ROW + 10] = -ins.Cnb[1 * 3 + 1]; Phi[1 * ROW + 11] = -ins.Cnb[1 * 3 + 2];
	Phi[2 * ROW + 9] = -ins.Cnb[2 * 3 + 0]; Phi[2 * ROW + 10] = -ins.Cnb[2 * 3 + 1]; Phi[2 * ROW + 11] = -ins.Cnb[2 * 3 + 2];

	Phi[3 * ROW + 1] = Mva[0 * 3 + 1]; Phi[3 * ROW + 2] = Mva[0 * 3 + 2];
	Phi[4 * ROW + 0] = Mva[1 * 3 + 0];                         Phi[4 * ROW + 2] = Mva[1 * 3 + 2];
	Phi[5 * ROW + 0] = Mva[2 * 3 + 0]; Phi[5 * ROW + 1] = Mva[2 * 3 + 1];

	Phi[3 * ROW + 3] = Mvv[0 * 3 + 0]; Phi[3 * ROW + 4] = Mvv[0 * 3 + 1]; Phi[3 * ROW + 5] = Mvv[0 * 3 + 2];
	Phi[4 * ROW + 3] = Mvv[1 * 3 + 0]; Phi[4 * ROW + 4] = Mvv[1 * 3 + 1]; Phi[4 * ROW + 5] = Mvv[1 * 3 + 2];
	Phi[5 * ROW + 3] = Mvv[2 * 3 + 0]; Phi[5 * ROW + 4] = Mvv[2 * 3 + 1]; Phi[5 * ROW + 5] = Mvv[2 * 3 + 2];

	Phi[3 * ROW + 6] = Mvp[0 * 3 + 0]; Phi[3 * ROW + 7] = Mvp[0 * 3 + 1]; Phi[3 * ROW + 8] = Mvp[0 * 3 + 2];
	Phi[4 * ROW + 6] = Mvp[1 * 3 + 0]; Phi[4 * ROW + 7] = Mvp[1 * 3 + 1]; Phi[4 * ROW + 8] = Mvp[1 * 3 + 2];
	Phi[5 * ROW + 6] = Mvp[2 * 3 + 0]; Phi[5 * ROW + 7] = Mvp[2 * 3 + 1]; Phi[5 * ROW + 8] = Mvp[2 * 3 + 2];

	Phi[3 * ROW + 12] = ins.Cnb[0 * 3 + 0]; Phi[3 * ROW + 13] = ins.Cnb[0 * 3 + 1]; Phi[3 * ROW + 14] = ins.Cnb[0 * 3 + 2];
	Phi[4 * ROW + 12] = ins.Cnb[1 * 3 + 0]; Phi[4 * ROW + 13] = ins.Cnb[1 * 3 + 1]; Phi[4 * ROW + 14] = ins.Cnb[1 * 3 + 2];
	Phi[5 * ROW + 12] = ins.Cnb[2 * 3 + 0]; Phi[5 * ROW + 13] = ins.Cnb[2 * 3 + 1]; Phi[5 * ROW + 14] = ins.Cnb[2 * 3 + 2];

	Phi[6 * ROW + 4] = Mpv[0 * 3 + 1];
	Phi[7 * ROW + 3] = Mpv[1 * 3 + 0];
	Phi[8 * ROW + 5] = Mpv[2 * 3 + 2];

	Phi[6 * ROW + 8] = Mpp[0 * 3 + 2];
	Phi[7 * ROW + 6] = Mpp[1 * 3 + 0];                             Phi[7 * ROW + 8] = Mpp[1 * 3 + 2];

#if 0		//加计和陀螺的一阶马尔科夫模型 相关时间200s 反相关系数 与遗忘滤波相互作用了
	double t = 100.0;
	Phi[9 * ROW + 9] = -1.0 / t;
	Phi[10 * ROW + 10] = -1.0 / t;
	Phi[11 * ROW + 11] = -1.0 / t;
	Phi[12 * ROW + 12] = -1.0 / t;
	Phi[13 * ROW + 13] = -1.0 / t;
	Phi[14 * ROW + 14] = -1.0 / t;
#endif
	/*dsf90:Phi = I + F*T = I + Phi*dt */
	double *eye = (double*)__ml_zero(sizeof(double)*ROW*ROW);
	Mat_unit(eye, ROW);
	Mat_mulb(Phi, ROW, ROW, dt, Phi);
	Mat_add(Phi, eye, Phi, ROW, ROW);
	free(eye);
}


void GINS_KF::upHk(GINS_INS& ins, double *hk)
{
	/*
	hk[0*ROW+0]=-cos(ins.att[2]);
	hk[0*ROW+1]= sin(ins.att[2]);
	hk[0*ROW+2]= 0;
	hk[1*ROW+0]= sin(ins.att[2])/cos(ins.att[0]);
	hk[1*ROW+1]=-cos(ins.att[2])/cos(ins.att[0])+2*sin(ins.att[0])*sin(ins.att[1])*cos(ins.att[1])*sin(ins.att[2])/cos(ins.att[0]);
	hk[1*ROW+2]= 0;
	hk[2*ROW+0]=-sin(ins.att[0])*sin(ins.att[2])/cos(ins.att[0]);
	hk[2*ROW+1]= sin(ins.att[0])*cos(ins.att[2])/cos(ins.att[0]);
	hk[2*ROW+2]=-1;*/

	hk[0 * ROW + 0] = 1; hk[1 * ROW + 1] = 1; hk[2 * ROW + 2] = 1;
	hk[3 * ROW + 3] = 1; hk[4 * ROW + 4] = 1; hk[5 * ROW + 5] = 1;
	hk[6 * ROW + 6] = 1; hk[7 * ROW + 7] = 1; hk[8 * ROW + 8] = 1;

#if 0
	double askwnb[3 * 3] = { 0.0 };
	double askwnblb[3] = { 0.0 };
	double askaskwnblb[3 * 3] = { 0.0 };
	double CnbAskAskWnbLb[3 * 3] = { 0.0 };

	askew(ins.wnb, askwnb);
	Mmulnm(askwnb, ins.lever, 3, 3, 1, askwnblb);
	askew(askwnblb, askaskwnblb);
	Mmulnm(ins.Cnb, askaskwnblb, 3, 3, 3, CnbAskAskWnbLb);

	/*dsf90:速度观测——姿态*/
	for (int i = 0; i<3; i++)
	{
		{	hk[3 * ROW + 0 + i] = -CnbAskAskWnbLb[0 * 3 + i]; }
		{	hk[4 * ROW + 0 + i] = -CnbAskAskWnbLb[1 * 3 + i]; }
		{	hk[5 * ROW + 0 + i] = -CnbAskAskWnbLb[2 * 3 + i]; }
	}

	double asklb[3 * 3] = { 0.0 };
	double cnbasklb[3 * 3] = { 0.0 };
	double MpvCnbAskLb[3 * 3] = { 0.0 };

	askew(ins.lever, asklb);
	Mmulnm(ins.Cnb, asklb, 3, 3, 3, cnbasklb);
	Mmulnm(ins.Mpv, cnbasklb, 3, 3, 3, MpvCnbAskLb);

	/*dsf90:位置观测——姿态*/
	for (int i = 0; i<3; i++)
	{
		{	hk[6 * ROW + 0 + i] = -MpvCnbAskLb[0 * 3 + i]; }
		{	hk[7 * ROW + 0 + i] = -MpvCnbAskLb[1 * 3 + i]; }
		{	hk[8 * ROW + 0 + i] = -MpvCnbAskLb[2 * 3 + i]; }
	}
#endif	

	/*dsf90:速度观测*/
	for (int i = 0; i<3; i++)
	{
		{	hk[3 * ROW + 18 + i] = -ins.CW[0 * 3 + i]; }
		{	hk[4 * ROW + 18 + i] = -ins.CW[1 * 3 + i]; }
		{	hk[5 * ROW + 18 + i] = -ins.CW[2 * 3 + i]; }
	}

	double Mpvcnb[3 * 3] = { 0.0 };
	Mat_mul(ins.Mpv, ins.Cnb, 3, 3, 3, Mpvcnb);
	/*dsf90:位置观测*/
	for (int i = 0; i<3; i++)
	{
		{	hk[6 * ROW + 18 + i] = -Mpvcnb[0 * 3 + i]; }
		{	hk[7 * ROW + 18 + i] = -Mpvcnb[1 * 3 + i]; }
		{	hk[8 * ROW + 18 + i] = -Mpvcnb[2 * 3 + i]; }
	}

	double cbn[9], cmn[9], vb[3], askvn[9], cmnAskvn[9], askvb[9], cmbAskvb[9];
	Mat_tran(ins.Cnb, 3, 3, cbn);
	Mat_mul(ins.Cmb, cbn, 3, 3, 3, cmn);
	askew(ins.vn, askvn);
	Mat_mul(cmn, askvn, 3, 3, 3, cmnAskvn);

	Mat_mul(cbn, ins.vn, 3, 3, 1, vb);
	askew(vb, askvb);
	Mat_mul(ins.Cmb, askvb, 3, 3, 3, cmbAskvb);

	/*dsf90:载体约束/里程计轮速*/
	for (int i = 0; i<3; i++)
	{
		{	hk[9 * ROW + i] = -cmnAskvn[0 * 3 + i]; hk[9 * ROW + i + 3] = cmn[0 * 3 + i]; hk[9 * ROW + 15 + i] = cmbAskvb[0 * 3 + i]; }
		{	hk[10 * ROW + i] = -cmnAskvn[1 * 3 + i]; hk[10 * ROW + i + 3] = cmn[1 * 3 + i]; hk[10 * ROW + 15 + i] = cmbAskvb[1 * 3 + i]; }
		{	hk[11 * ROW + i] = -cmnAskvn[2 * 3 + i]; hk[11 * ROW + i + 3] = cmn[2 * 3 + i]; hk[11 * ROW + 15 + i] = cmbAskvb[2 * 3 + i]; }
	}

	/*dsf90:载体速度观测（里程计航向）*/
	hk[12 * ROW + 11] = -1;
}