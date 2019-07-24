#include "GINS_ins.h"
CGLV glv;
void GINS_INS::Init(double Att[], double Vn[], double Pos[], double EB[], double DB[], double Lever[], double Lever2[], double PRY_install[])
{
	Mat_equal(Att, 3, 1, att);
	Mat_equal(Vn, 3, 1, vnL);
	Mat_equal(Pos, 3, 1, posL);
	Mat_equal(EB, 3, 1, eb);
	Mat_equal(DB, 3, 1, db);
	Mat_equal(Lever, 3, 1, lever);
	Mat_equal(Lever2, 3, 1, lever2);
	Mat_equal(PRY_install, 3, 1, PRY_Install);

	a2qua(PRY_install, qmb);
	a2mat(PRY_install, Cmb);

	a2qua(Att, qnb);
	a2mat(Att, Cnb);
	eth.UpdatePV(posL, vnL);
	Mat_equal(eth.gn, 3, 1, fn);

	//tDelay = 0.0;
	for (int i = 0; i<9; i++)
	{
		Mpv[i] = 0.0;
		MpvCnb[i] = 0.0;
		CW[i] = 0.0;
		Ka[i] = 0.0;
		Kg[i] = 0.0;
	}
	for (int i = 0; i<3; i++)
	{
		fn[i] = -fn[i];
		an[i] = 0.0;
		web[i] = 0.0;
		wnb[i] = 0.0;
		Mpvvn[i] = 0.0;
		wm_1[i] = 0.0;
		vm_1[i] = 0.0;
		Ka[i * 3 + i] = 1.0;
		Kg[i * 3 + i] = 1.0;
	}
	Mpv[0 * 3 + 1] = 1.0 / eth.RMh;
	Mpv[1 * 3 + 0] = 1.0 / eth.clRNh;
	Mpv[2 * 3 + 2] = 1.0;
	Mat_mul(Mpv, Vn, 3, 3, 1, Mpvvn);
	Mat_mul(Mpv, Cnb, 3, 3, 1, MpvCnb);
	double Cnbt[9];
	Mat_tran(Cnb, 3, 3, Cnbt);
	Mat_mul(Cnbt, eth.wnin, 3, 3, 1, wib);
	Mat_mul(Cnbt, fn, 3, 3, 1, fb);
}



void GINS_INS::Update(double wm[], double vm[], double dtime)
{
	double t2 = dtime*0.5;
	/*dsf90:wib = wib - eb; wib, b系角速率*/
	/*dsf90:fb	= fb - db; fb, b系加速度*/
	Mat_min(wib, eb, wib, 3, 1);//陀螺去零偏
	Mat_min(fb, db, fb,3, 1);//加计去零偏

	Mat_mulb(wib, 3, 1, dtime, wm);
	Mat_mulb(fb, 3, 1, dtime, vm);

	eth.UpdatePV(pos, vn);
	double Cnbt[3 * 3], Ctwie[3 * 1];
	Mat_equal(Cnb, 3, 3, Cnbt);
	Mat_tran(Cnbt, 3, 3, Cnbt);

	Mat_mul(Cnbt, eth.wnin, 3, 3, 1, Ctwie);
	Mat_min(wib, Ctwie, wnb, 3, 1);

	/*dsf90:速度更新*/
	/*dsf90:fn  = Cnb X fb; fn, n系实际加速度*/
	/*dsf90:an  = rv2m(-0.5*dt*wnin) X fn + gcc; fn, n系实际加速度*/
	/*dsf90:vn1 = vn + (rv2m(-0.5*dt*wnin) X (Cnb X fb) + gcc)*dt;*/
	/*dsf90:教材版 vn1 = vn + (Cnb X fb + gcc)*dt*/
	Mat_mul(Cnb, fb, 3, 3, 1, fn);                /*dsf90: 等效 qmulv(qnb,fb,fn);*/
	double ant[3], vn1[3];
	Mat_add(fn, eth.gcc, an, 3, 1);

	Mat_mulb(an, 3, 1, dtime,ant);
	Mat_add(vn, ant, vn1, 3, 1);


	Mpv[0 * 3 + 1] = 1.0 / eth.RMh;
	Mpv[1 * 3 + 0] = 1.0 / eth.clRNh;
	Mpv[2 * 3 + 2] = 1.0;
	double vnvn1[3], Mvnt[3];
	Mat_add(vn, vn1, vnvn1, 3, 1);
	Mat_mulb(vnvn1, 3, 1, 0.5, vnvn1);

	/*dsf90:位置更新*/
	/*dsf90:vn = vn1*/
	/*dsf90:Mpvvn =  0.5 * Mpv X (vn+vn1)*/
	Mat_mul(Mpv, vnvn1, 3, 3, 1, Mpvvn);
	Mat_mulb(Mpvvn, 3, 1, dtime, Mvnt);
	/*dsf90:pos = pos + 0.5 * dt * Mpv X (vn+vn1)*/
	Mat_add(pos, Mvnt, pos,3, 1);
	Mat_equal(vn1, 3, 1, vn);

	/*dsf90:姿态更新*/
	/*dsf90:qnb = qupdt2(qnb,wib*dt,wnin*dt);*/
	/*dsf90:教材版 qnb = qupdt(qnb,wnb*dt) = qupdt(qnb,((wib-Cbn X wnin)*dt))*/
	double wnindt[3], qnb1[4];
	//double q[4];
#if 1
	Mat_mulb(eth.wnin, 3, 1, dtime, wnindt);
	qupdt2(qnb, wm, wnindt, qnb1);
#endif
	Mat_equal(qnb1, 4, 1, qnb);

	q2att(qnb, att);
	q2mat(qnb, Cnb);
	/*dsf90:更新车辆速度*/
	double cbn[9], vb[3];
	Mat_tran(Cnb, 3, 3, cbn);
	Mat_mul(cbn, vn1, 3, 3, 1, vb);
	Mat_mul(Cmb, vb, 3, 3, 1, vm_car);

	Mat_tran(Cmb, 3, 3, Cbm);
	Mat_mul(Cnb, Cbm, 3, 3, 3, Cnm);
	m2att(Cnm, att_car);
	/*dsf90:更新IMU、车辆加速度*/
	Mat_mul(cbn, an, 3, 3, 1, ab);
	Mat_mul(Cmb, ab, 3, 3, 1, am);

	Mat_mul(Cmb, fb, 3, 3, 1, fm);
	Mat_mul(Cmb, wib, 3, 3, 1, wim);
}


void GINS_INS::Lever(double pos1[], double vn1[], double pos2[], double vn2[], double lever[])
{
	double askeww[3 * 3] = { 0.0 };
	double temp31[3] = { 0.0 };

	/*dsf90:wbnb    = wbib - Cbn X wnin;*/
	/*dsf90:CW	   = Cnb X (wbnb X);*/
	askew(wnb, askeww);
	Mat_mul(Cnb, askeww, 3, 3, 3, CW);
	/*dsf90:MpvCnb = Mpv X Cnb;*/
	Mat_mul(Mpv, Cnb, 3, 3, 3, MpvCnb);

	/*dsf90:vnL    = vn + CW X lb;*/
	Mat_mul(CW, lever, 3, 3, 1, temp31);
	Mat_add(vn1, temp31, vn2, 3, 1);
	//Mminn(vn,temp31,vnL,3,1);

	/*dsf90:posL    = pos + Mpv X Cnb X lb;*/
	Mat_mul(MpvCnb, lever, 3, 3, 1, temp31);
	Mat_add(pos1, temp31, pos2, 3, 1);
	//Mminn(pos,temp31,posL,3,1);
}

void GINS_INS::INS(double Att[], double Vn[], double Pos[], double EB[], double DB[], double Lever[])
{
	Mat_equal(Att, 3, 1, att);
	Mat_equal(Vn, 3, 1, vn);
	Mat_equal(Pos, 3, 1, pos);
	Mat_equal(EB, 3, 1, eb);
	Mat_equal(DB, 3, 1, db);
	Mat_equal(Lever, 3, 1, lever);

	a2qua(Att, qnb);
	a2mat(Att, Cnb);
	eth.UpdatePV(pos, vn);
	Mat_equal(eth.gn, 3, 1, fn);
	tDelay = 0.0;
	for (int i = 0; i<9; i++)
	{
		Mpv[i] = 0.0;
		MpvCnb[i] = 0.0;
		CW[i] = 0.0;
		Ka[i] = 0.0;
		Kg[i] = 0.0;
	}
	for (int i = 0; i<3; i++)
	{
		PRY_Install[i] = 0;

		fn[i] = -fn[i];
		an[i] = 0.0;
		web[i] = 0.0;
		wnb[i] = 0.0;
		Mpvvn[i] = 0.0;
		vnL[i] = 0.0;
		posL[i] = 0.0;
		wm_1[i] = 0.0;
		vm_1[i] = 0.0;
		Ka[i * 3 + i] = 1.0;
		Kg[i * 3 + i] = 1.0;
	}
	Mpv[0 * 3 + 1] = 1.0 / eth.RMh;
	Mpv[1 * 3 + 0] = 1.0 / eth.clRNh;
	Mpv[2 * 3 + 2] = 1.0;
	Mat_mul(Mpv, Vn, 3, 3, 1, Mpvvn);
	Mat_mul(Mpv, Cnb, 3, 3, 1, MpvCnb);
	double Cnbt[9];
	Mat_tran(Cnb, 3, 3, Cnbt);
	Mat_mul(Cnbt, eth.wnin, 3, 3, 1, wib);
	Mat_mul(Cnbt, fn, 3, 3, 1, fb);

	Mat_equal(Cnb, 3, 3, Cnm);
	Mat_equal(att, 3, 1, att_car);

	//Kd = 1;
	//Kwh = 1;
}


void CEarth::UpdatePV(double pos[], double vn[])
{
	sl = sin(pos[0]); cl = cos(pos[0]); tl = sl / cl;
	sl2 = sl*sl; sl4 = sl2*sl2;
	double sq = 1 - e2*sl*sl, sq2 = sqrt(sq);

	/*dsf90: RMh = Rm + h = Re*(1-e*e)/(1-(e*sin(lat)*(e*sin(lat))^(3/2) + h*/
	/*dsf90: RNh = Rn + h = Re/(1-(e*sin(lat)*(e*sin(lat))^(1/2) + h*/
	RMh = RE*(1 - e2) / sq / sq2 + pos[2]; f_RMh = 1.0 / RMh;
	RNh = RE / sq2 + pos[2]; f_RNh = 1.0 / RNh;
	clRNh = cl*RNh; f_clRNh = 1.0 / clRNh;

	gn[2] = -(glv.g0*(1 + 5.27094e-3*sl2 + 2.32718e-5*sl4) - 3.086e-6*pos[2]);
	//glv.g=-gn[2];/*add by dsf90,2018.6.26*/

	/*dsf90: wnie,地球自转速率；wnen,位移速率；*/
	wnie[0] = 0.0;
	wnie[1] = glv.wie*cl;
	wnie[2] = glv.wie*sl;
	wnen[0] = -vn[1] / RMh;
	wnen[1] = vn[0] / RNh;
	wnen[2] = vn[0] / RNh*tl;
	/*dsf90: wnin = wnie + wnen*/
	Mat_add(wnie, wnen, wnin, 3, 1);

	double w[3] = { 0.0 };
	Mat_add(wnie, wnin, w, 3, 1);
	/*dsf90:gcc = gn - (wnen+2wnie) X vn; w,vn 叉积 ;n系*/
	gcc[0] = w[1] * vn[2] - w[2] * vn[1];
	gcc[1] = w[2] * vn[0] - w[0] * vn[2];
	gcc[2] = w[0] * vn[1] - w[1] * vn[0];
	Mat_min(gn, gcc, gcc, 3, 1);
}

CGLV::CGLV(double Re, double f, double wie0, double g0)
{
	this->Re = Re; this->f = f; this->wie = wie0; this->g0 = g0;
	g = glv_g;
	mg = 1.0e-3*g;
	ug = 1.0e-6*g;
	deg = PI / 180.0;
	min = deg / 60.0;
	sec = min / 60.0;
	ppm = 1.0e-6;
	hur = 3600.0;
	dph = deg / hur;
	dpsh = deg / sqrt(hur);
	dphpsh = dph / sqrt(hur);
	ugpsHz = ug / sqrt(1.0);
	ugpsh = ug / sqrt(hur);
	mpsh = 1 / sqrt(hur);
	mpspsh = 1 / 1 / sqrt(hur);
	ppmpsh = ppm / sqrt(hur);
	secpsh = sec / sqrt(hur);
	kmph = 3.6;
}

int GINS_INS::INS_process(Process_Data ilcd, double dt)
{
	double *wmcur, *vmcur, wm[3], vm[3];
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

	Mat_equal(wm, 3, 1, wib);
	Mat_equal(vm, 3, 1, fb);

	Mat_mulb(wm, 3, 1, dt, wm);
	Mat_mulb(vm, 3, 1, dt, vm);
	Update(wm, vm, dt);
	return 1;
}