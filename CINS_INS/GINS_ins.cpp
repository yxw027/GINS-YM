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
	//double ebdt[3],dbdt[3],kwm[3],kvm[3];
	//Mmuln(eb,3,1,dtime,ebdt); 
	//Mmuln(db,3,1,dtime,dbdt); 
	//Mmulnm(Kg,wm,3,3,1,kwm);
	//Mmulnm(Ka,vm,3,3,1,kvm);
	//Mminn(kwm,ebdt,wm,3,1); 
	//Mminn(kvm,dbdt,vm,3,1); 

	/*dsf90:wib = wib - eb; wib, b系角速率*/
	/*dsf90:fb	= fb - db; fb, b系加速度*/
	Mat_min(wib, eb, wib, 3, 1);
	Mat_min(fb, db, fb,3, 1);

	Mat_mul(wib, wm, 3, 1, dtime, wib);
	Mat_mul(fb, vm, 3, 1, dtime,  fb);

#if (!GILC_SIMPLIFY_USED)	
	/*dsf90:速度、位置预测更新*/
	/*dsf90:vn01 = vn + 0.5*dt*an ; vn01 本时间段内的中点速度*/
	double ant2[3], vn01[3], vn01t2[3], Mvt[3], pos01[3];
	Mmuln(an, 3, 1, t2, ant2);
	Maddn(vn, ant2, vn01, 3, 1);
	/*dsf90:pos01 = pos + 0.5*dt*Mpv X vn01; pos01 本时间段内的中点位置*/
	Mmuln(vn01, 3, 1, t2, vn01t2);
	Mmulnm(Mpv, vn01t2, 3, 3, 1, Mvt);
	Maddn(pos, Mvt, pos01, 3, 1);

	/*dsf90:使用位置、速度更新值，更新地球模型*/
	eth.UpdatePV(pos01, vn01);
#else	/*dsf90:简化版,差异小于1mm*/
	eth.UpdatePV(pos, vn);
#endif

	double Cnbt[3 * 3], Ctwie[3 * 1];
	Mat_equal(Cnb, 3, 3, Cnbt);
	Mat_tran(Cnbt, 3, 3, Cnbt);

#if (!GILC_SIMPLIFY_USED)	
	/*dsf90:web = wib - Cbn X wnie；*/
	Mmulnm(Cnbt, eth.wnie, 3, 3, 1, Ctwie);
	Mminn(wib, Ctwie, web, 3, 1);
#else
	/*dsf90:wnb = wib - Cbn X wnin；*/
	Mat_mul(Cnbt, eth.wnin, 3, 3, 1, Ctwie);
	Mat_min(wib, Ctwie, wnb, 3, 1);
#endif

#if (!GILC_SIMPLIFY_USED)	
	/*dsf90:Cnb_new = Cwm2m = Cnb X rv2m(0.5*wib*dt);时间段中点角度增量*/
	/*dsf90:win_new = Ctwnin = Cbn_new X wnin;*/
	/*dsf90:wnb = wib - Cbn_new X wnin;疑问？wnb未使用*/
	//double wnb[3];
	double wm2m[3 * 3], wm2[3 * 1], Cwm2m[3 * 3], Cwm2mt[3 * 3], Ctwnin[3 * 1];
	Mmuln(wm, 3, 1, 0.5, wm2);
	rv2m(wm2, wm2m);
	Mmulnm(Cnb, wm2m, 3, 3, 3, Cwm2m);
	Mtn(Cwm2m, 3, 3, Cwm2mt);
	Mmulnm(Cwm2mt, eth.wnin, 3, 3, 1, Ctwnin);
	Mminn(wib, Ctwnin, wnb, 3, 1);
#else /*dsf90:简化*/

#endif

	/*dsf90:速度更新*/
	/*dsf90:fn  = Cnb X fb; fn, n系实际加速度*/
	/*dsf90:an  = rv2m(-0.5*dt*wnin) X fn + gcc; fn, n系实际加速度*/
	/*dsf90:vn1 = vn + (rv2m(-0.5*dt*wnin) X (Cnb X fb) + gcc)*dt;*/
	/*dsf90:教材版 vn1 = vn + (Cnb X fb + gcc)*dt*/
	Mat_mul(Cnb, fb, 3, 3, 1, fn);                /*dsf90: 等效 qmulv(qnb,fb,fn);*/
	double ant[3], vn1[3];
#if (!GILC_SIMPLIFY_USED)	
	double rtvfn[3];
	double wnint[3];
	Mmuln(eth.wnin, 3, 1, -t2, wnint);
	rotv(wnint, fn, rtvfn);
	Maddn(rtvfn, eth.gcc, an, 3, 1);
#else /*dsf90:简化版，教材版，差异小于1mm*/
	Mat_add(fn, eth.gcc, an, 3, 1);
#endif	
	Mat_mul(an, ant, 3, 1, dtime,an);
	Mat_add(vn, ant, vn1, 3, 1);

	/*dsf90:位置更新*/
	/*dsf90:Mpvvn =  0.5 * Mpv X (vn+vn1)*/
	/*dsf90:pos = pos + 0.5 * dt * Mpv X (vn+vn1)*/
	/*dsf90:vn = vn1*/
	Mpv[0 * 3 + 1] = 1.0 / eth.RMh;
	Mpv[1 * 3 + 0] = 1.0 / eth.clRNh;
	Mpv[2 * 3 + 2] = 1.0;
	double vnvn1[3], Mvnt[3];
	Mat_add(vn, vn1, vnvn1, 3, 1);
	Mat_mulb(vnvn1, 3, 1, 0.5, vnvn1);
	Mat_mul(Mpv, vnvn1, 3, 3, 1, Mpvvn);
	Mat_mulb(Mpvvn, 3, 1, dtime, Mvnt);
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
#elif 0 /*dsf90:等效展开，误差小于1mm*/
	Mmuln(eth.wnin, 3, 1, -dtime, wnindt);
	qupdt(qnb, wm, qnb1);
	qdelphi(qnb1, wnindt, qnb1);
#elif 0 /*dsf90:等效展开，误差小于1mm*/
	Mmuln(eth.wnin, 3, 1, -dtime, wnindt);
	memcpy(qnb1, qnb, sizeof(qnb1));
	rv2q(wm, q);
	qmul(qnb1, q);
	rv2q(wnindt, q);
	qmul(q, qnb1);
	memcpy(qnb1, q, sizeof(qnb1));
#else	/*dsf90:简化版，教材版，角增量加减后更新，差异约1mm*/
	/*dsf90:wnb = wib - Cbn X wnin;*/
	double wnbdt[3];
	double Cbn[3 * 3], Wbin[3];
	double wnb[3];
	Mtn(Cnb, 3, 3, Cbn);
	Mmulnm(Cbn, eth.wnin, 3, 3, 1, Wbin);
	Mminn(wib, Wbin, wnb, 3, 1);
	Mmuln(wnb, 3, 1, dtime, wnbdt);
	qupdt(qnb, wnbdt, qnb1);
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
	RMh = a*(1 - e2) / sq / sq2 + pos[2]; f_RMh = 1.0 / RMh;
	RNh = a / sq2 + pos[2]; f_RNh = 1.0 / RNh;
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

