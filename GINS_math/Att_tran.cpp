#include "Att_tran.h"
void m2qua_ned(double m[], double q[])
{
	double s[5] = { 0.0 };
	s[4] = m[0 * 3 + 0] + m[1 * 3 + 1] + m[2 * 3 + 2];
	s[0] = 1.0 + s[4];
	s[1] = 1.0 + 2.0*m[0 * 3 + 0] - s[4];
	s[2] = 1.0 + 2.0*m[1 * 3 + 1] - s[4];
	s[3] = 1.0 + 2.0*m[2 * 3 + 2] - s[4];
	int index = 0;
	double max = s[0];
	for (int k = 1; k<4; k++)
	{
		if (s[k]>max)
		{
			index = k;
			max = s[k];
		}
	}
	switch (index)
	{
	case 0:
		q[0] = 0.5*sqrt(s[0]);
		q[1] = 0.25*(m[2 * 3 + 1] - m[1 * 3 + 2]) / q[0];
		q[2] = 0.25*(m[0 * 3 + 2] - m[2 * 3 + 0]) / q[0];
		q[3] = 0.25*(m[1 * 3 + 0] - m[0 * 3 + 1]) / q[0];
		break;
	case 1:
		q[1] = 0.5*sqrt(s[1]);
		q[2] = 0.25*(m[1 * 3 + 0] + m[0 * 3 + 1]) / q[1];
		q[3] = 0.25*(m[0 * 3 + 2] + m[2 * 3 + 0]) / q[1];
		q[0] = 0.25*(m[2 * 3 + 1] - m[1 * 3 + 2]) / q[1];
		break;
	case 2:
		q[2] = 0.5*sqrt(s[2]);
		q[3] = 0.25*(m[2 * 3 + 1] + m[1 * 3 + 2]) / q[2];
		q[0] = 0.25*(m[0 * 3 + 2] - m[2 * 3 + 0]) / q[2];
		q[1] = 0.25*(m[1 * 3 + 0] + m[0 * 3 + 1]) / q[2];
		break;
	case 3:
		q[3] = 0.5*sqrt(s[3]);
		q[0] = 0.25*(m[1 * 3 + 0] - m[0 * 3 + 1]) / q[3];
		q[1] = 0.25*(m[0 * 3 + 2] + m[2 * 3 + 0]) / q[3];
		q[2] = 0.25*(m[2 * 3 + 1] + m[1 * 3 + 2]) / q[3];
		break;
	}
}

void q2mat_ned(double qua[], double m[])
{
	double q11, q12, q13, q14, q22, q23, q24, q33, q34, q44;
	q11 = qua[0] * qua[0]; q12 = qua[0] * qua[1]; q13 = qua[0] * qua[2]; q14 = qua[0] * qua[3];
	q22 = qua[1] * qua[1]; q23 = qua[1] * qua[2]; q24 = qua[1] * qua[3];
	q33 = qua[2] * qua[2]; q34 = qua[2] * qua[3];
	q44 = qua[3] * qua[3];
	m[0 * 3 + 0] = q11 + q22 - q33 - q44; m[0 * 3 + 1] = 2 * (q23 - q14);     m[0 * 3 + 2] = 2 * (q24 + q13);
	m[1 * 3 + 0] = 2 * (q23 + q14);     m[1 * 3 + 1] = q11 - q22 + q33 - q44; m[1 * 3 + 2] = 2 * (q34 - q12);
	m[2 * 3 + 0] = 2 * (q24 - q13);     m[2 * 3 + 1] = 2 * (q34 + q12);     m[2 * 3 + 2] = q11 - q22 - q33 + q44;
}

void m2att_ned(double m[], double a[])
{
	a[0] = atan2(m[2 * 3 + 1], m[2 * 3 + 2]);
	a[1] = asin(-m[2 * 3 + 0]);
	a[2] = atan2(m[1 * 3 + 0], m[0 * 3 + 0]);
}

void a2mat_ned(double att[], double m[])
{
	double phi = att[0];
	double theta = att[1];
	double psi = att[2];
	double cpsi = cos(psi); double spsi = sin(psi);
	double cthe = cos(theta); double sthe = sin(theta);
	double cphi = cos(phi); double sphi = sin(phi);
	double C1[9] = { 0.0 };
	double C2[9] = { 0.0 };
	double C3[9] = { 0.0 };
	double m1[9] = { 0.0 };
	double m2[9] = { 0.0 };
	C1[0 * 3 + 0] = cpsi;  C1[0 * 3 + 1] = spsi;
	C1[1 * 3 + 0] = -spsi; C1[1 * 3 + 1] = cpsi;
	C1[2 * 3 + 2] = 1.0;
	C2[0 * 3 + 0] = cthe;                  C2[0 * 3 + 2] = -sthe;
	C2[1 * 3 + 1] = 1.0;
	C2[2 * 3 + 0] = sthe;                  C2[2 * 3 + 2] = cthe;
	C3[0 * 3 + 0] = 1.0;
	C3[1 * 3 + 1] = cphi; C3[1 * 3 + 2] = sphi;
	C3[2 * 3 + 1] = -sphi; C3[2 * 3 + 2] = cphi;
	Mat_mul(C3, C2, 3, 3, 3, m1);
	Mat_mul(m1, C1, 3, 3, 3, m2);
	Mat_tran(m2, 3, 3, m);
}

/*dsf90:旋转顺序Z,X,Y*/
void a2mat(double att[], double m[])
{
	double si, sj, sk, ci, cj, ck;
	si = sin(att[0]); sj = sin(att[1]); sk = sin(att[2]);
	ci = cos(att[0]); cj = cos(att[1]); ck = cos(att[2]);
	m[0 * 3 + 0] = cj*ck - si*sj*sk;
	m[0 * 3 + 1] = -ci*sk;
	m[0 * 3 + 2] = sj*ck + si*cj*sk;
	m[1 * 3 + 0] = cj*sk + si*sj*ck;
	m[1 * 3 + 1] = ci*ck;
	m[1 * 3 + 2] = sj*sk - si*cj*ck;
	m[2 * 3 + 0] = -ci*sj;
	m[2 * 3 + 1] = si;
	m[2 * 3 + 2] = ci*cj;
}
/*******************************
功能：欧拉角转四元数
输入：att[3] 三轴欧拉角度（姿态角），单位rad(弧度)
输出：qua[4] 四元数
返回：无
********************************/
void a2qua(double att[], double qua[])
{
	double att2[3] = { 0.0 };
	att2[0] = att[0] * 0.5;
	att2[1] = att[1] * 0.5;
	att2[2] = att[2] * 0.5;

	double sp, sr, sy, cp, cr, cy;
	sp = sin(att2[0]); sr = sin(att2[1]); sy = sin(att2[2]);
	cp = cos(att2[0]); cr = cos(att2[1]); cy = cos(att2[2]);

	qua[0] = cp*cr*cy - sp*sr*sy;
	qua[1] = sp*cr*cy - cp*sr*sy;
	qua[2] = cp*sr*cy + sp*cr*sy;
	qua[3] = cp*cr*sy + sp*sr*cy;
}

void m2att(double mat[], double att[])
{
	att[0] = asin(mat[2 * 3 + 1]);
	att[1] = atan2(-mat[2 * 3 + 0], mat[2 * 3 + 2]);
	att[2] = atan2(-mat[0 * 3 + 1], mat[1 * 3 + 1]);
}

void m2qua(double m[], double qua[])
{
	qua[0] = 1 + m[0 * 3 + 0] + m[1 * 3 + 1] + m[2 * 3 + 2];
	qua[1] = 1 + m[0 * 3 + 0] - m[1 * 3 + 1] - m[2 * 3 + 2];
	qua[2] = 1 - m[0 * 3 + 0] + m[1 * 3 + 1] - m[2 * 3 + 2];
	qua[3] = 1 - m[0 * 3 + 0] - m[1 * 3 + 1] + m[2 * 3 + 2];

	int sign[4];
	sign[0] = 1;
	if ((m[2 * 3 + 1] - m[1 * 3 + 2]) >= 0)
	{
		sign[1] = 1;
	}
	else
	{
		sign[1] = -1;
	}
	if ((m[0 * 3 + 2] - m[2 * 3 + 0]) >= 0)
	{
		sign[2] = 1;
	}
	else
	{
		sign[2] = -1;
	}
	if ((m[1 * 3 + 0] - m[0 * 3 + 1]) >= 0)
	{
		sign[3] = 1;
	}
	else
	{
		sign[3] = -1;
	}

	qua[0] = sign[0] * sqrt(fabs(qua[0])) / 2;
	qua[1] = sign[1] * sqrt(fabs(qua[1])) / 2;
	qua[2] = sign[2] * sqrt(fabs(qua[2])) / 2;
	qua[3] = sign[3] * sqrt(fabs(qua[3])) / 2;
}

void q2att(double qua[], double att[])
{
	double q11, q12, q13, q14, q22, q23, q24, q33, q34, q44;
	double c12, c22, c31, c32, c33;
	q11 = qua[0] * qua[0]; q12 = qua[0] * qua[1]; q13 = qua[0] * qua[2]; q14 = qua[0] * qua[3];
	q22 = qua[1] * qua[1]; q23 = qua[1] * qua[2]; q24 = qua[1] * qua[3];
	q33 = qua[2] * qua[2]; q34 = qua[2] * qua[3];
	q44 = qua[3] * qua[3];
	c12 = 2 * (q23 - q14);
	c22 = q11 - q22 + q33 - q44;
	c31 = 2 * (q24 - q13); c32 = 2 * (q34 + q12); c33 = q11 - q22 - q33 + q44;
	att[0] = asin(c32);
	att[1] = atan2(-c31, c33);
	att[2] = atan2(-c12, c22);
}

void q2mat(double qua[], double mat[])
{
	double q11, q12, q13, q14, q22, q23, q24, q33, q34, q44;
	q11 = qua[0] * qua[0]; q12 = qua[0] * qua[1]; q13 = qua[0] * qua[2]; q14 = qua[0] * qua[3];
	q22 = qua[1] * qua[1]; q23 = qua[1] * qua[2]; q24 = qua[1] * qua[3];
	q33 = qua[2] * qua[2]; q34 = qua[2] * qua[3];
	q44 = qua[3] * qua[3];
	mat[0 * 3 + 0] = q11 + q22 - q33 - q44; mat[0 * 3 + 1] = 2 * (q23 - q14); mat[0 * 3 + 2] = 2 * (q24 + q13);
	mat[1 * 3 + 0] = 2 * (q23 + q14); mat[1 * 3 + 1] = q11 - q22 + q33 - q44; mat[1 * 3 + 2] = 2 * (q34 - q12);
	mat[2 * 3 + 0] = 2 * (q24 - q13); mat[2 * 3 + 1] = 2 * (q34 + q12); mat[2 * 3 + 2] = q11 - q22 - q33 + q44;
}

void askew(double v[], double m[])
{
	m[0 * 3 + 0] = 0.0;   m[0 * 3 + 1] = -v[2]; m[0 * 3 + 2] = v[1];
	m[1 * 3 + 0] = v[2];  m[1 * 3 + 1] = 0.0;   m[1 * 3 + 2] = -v[0];
	m[2 * 3 + 0] = -v[1]; m[2 * 3 + 1] = v[0];  m[2 * 3 + 2] = 0.0;
}



void qupdt2(double qnb0[], double rv_ib[], double rv_in[], double qnb1[])
{
	double n2, n, n_2, rv_ib0, s, qb1, qb2, qb3, qb4, rv_in0, nq;
	n2 = rv_ib[0] * rv_ib[0] + rv_ib[1] * rv_ib[1] + rv_ib[2] * rv_ib[2];
	if (n2<1.0e-8)
	{
		rv_ib0 = 1 - n2*(1.0 / 8.0 - n2 / 384.0);
		s = 0.5 - n2*(1.0 / 48.0 - n2 / 3840.0);
	}
	else
	{
		n = sqrt(n2); n_2 = n / 2.0;
		rv_ib0 = cos(n_2); s = sin(n_2) / n;
	}
	rv_ib[0] = s*rv_ib[0]; rv_ib[1] = s*rv_ib[1]; rv_ib[2] = s*rv_ib[2];
	// qnb1 = qmul(qnb0, q);
	qb1 = qnb0[0] * rv_ib0 - qnb0[1] * rv_ib[0] - qnb0[2] * rv_ib[1] - qnb0[3] * rv_ib[2];
	qb2 = qnb0[0] * rv_ib[0] + qnb0[1] * rv_ib0 + qnb0[2] * rv_ib[2] - qnb0[3] * rv_ib[1];
	qb3 = qnb0[0] * rv_ib[1] + qnb0[2] * rv_ib0 + qnb0[3] * rv_ib[0] - qnb0[1] * rv_ib[2];
	qb4 = qnb0[0] * rv_ib[2] + qnb0[3] * rv_ib0 + qnb0[1] * rv_ib[1] - qnb0[2] * rv_ib[0];
	// rv2q(-rv_in)
	n2 = rv_in[0] * rv_in[0] + rv_in[1] * rv_in[1] + rv_in[2] * rv_in[2];
	if (n2<1.0e-8)
	{
		rv_in0 = 1 - n2*(1.0 / 8.0 - n2 / 384.0); s = -0.5 + n2*(1.0 / 48.0 - n2 / 3840.0);
	}
	else
	{
		n = sqrt(n2); n_2 = n / 2.0;
		rv_in0 = cos(n_2); s = -sin(n_2) / n;
	}
	rv_in[0] = s*rv_in[0]; rv_in[1] = s*rv_in[1]; rv_in[2] = s*rv_in[2];
	// qnb1 = qmul(q, qnb1);  
	qnb1[0] = rv_in0 * qb1 - rv_in[0] * qb2 - rv_in[1] * qb3 - rv_in[2] * qb4;
	qnb1[1] = rv_in0 * qb2 + rv_in[0] * qb1 + rv_in[1] * qb4 - rv_in[2] * qb3;
	qnb1[2] = rv_in0 * qb3 + rv_in[1] * qb1 + rv_in[2] * qb2 - rv_in[0] * qb4;
	qnb1[3] = rv_in0 * qb4 + rv_in[2] * qb1 + rv_in[0] * qb3 - rv_in[1] * qb2;
	//normalization
	n2 = qnb1[0] * qnb1[0] + qnb1[1] * qnb1[1] + qnb1[2] * qnb1[2] + qnb1[3] * qnb1[3];
	if (n2>1.000001 || n2<0.999999)
	{
		nq = 1.0 / sqrt(n2);
		qnb1[0] = qnb1[0] * nq; qnb1[1] = qnb1[1] * nq; qnb1[2] = qnb1[2] * nq; qnb1[3] = qnb1[3] * nq;
	}
}
void rv2q(double rv[], double q[])
{
	double temp, f;
	temp = sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
	if (temp>1.0e-40)
		f = sin(temp / 2) / temp;
	else
		f = 0.5;
	q[0] = cos(temp / 2);
	q[1] = f*rv[0];
	q[2] = f*rv[1];
	q[3] = f*rv[2];
}

void qmuln(double q1[], double q2[], double q[])
{
	q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	q[2] = q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3];
	q[3] = q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1];
}
void qdelphi(double qpb[], double phi[])
{
	double q0[4] = { 0.0 };
	double qnb[4] = { 0.0 };
	rv2q(phi, q0);
	qmuln(q0, qpb, qnb);
	Mat_equal(qnb, 4, 1, qpb);
}

void ecef2enu(const double *pos, const double *r, double *e)
{
	double E[9];

	xyz2enu(pos, E);
	matmul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}

void xyz2enu(const double *pos, double *E)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

	E[0] = -sinl;      E[3] = cosl;       E[6] = 0.0;
	E[1] = -sinp*cosl; E[4] = -sinp*sinl; E[7] = cosp;
	E[2] = cosp*cosl;  E[5] = cosp*sinl;  E[8] = sinp;
}

void matmul(const char *tr, int n, int k, int m, double alpha,
	const double *A, const double *B, double beta, double *C)
{
	double d;
	int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

	for (i = 0;i<n;i++) for (j = 0;j<k;j++) {
		d = 0.0;
		switch (f) {
		case 1: for (x = 0;x<m;x++) d += A[i + x*n] * B[x + j*m]; break;
		case 2: for (x = 0;x<m;x++) d += A[i + x*n] * B[j + x*k]; break;
		case 3: for (x = 0;x<m;x++) d += A[x + i*m] * B[x + j*m]; break;
		case 4: for (x = 0;x<m;x++) d += A[x + i*m] * B[j + x*k]; break;
		}
		if (beta == 0.0) C[i + j*n] = alpha*d; else C[i + j*n] = alpha*d + beta*C[i + j*n];
	}
}

void Var_XYZ2BLH(double xyz[3], double Pecef[3], double Penu[3])
{
	double ENU[9], blh[3];
#if 0 // POS_XYZ
	ecef2pos(xyz, blh);
#endif 
#if 1 // POS_BLH_deg
	blh[0] = xyz[0] * PI / 180.0;
	blh[1] = xyz[1] * PI / 180.0;
	blh[2] = xyz[2];
#endif 
	xyz2enu(blh, ENU);

	//Penu[0]=sqrt(pow(ENU[0],2)*pow(Pecef[0],2)+pow(ENU[1],2)*pow(Pecef[1],2));
	//Penu[1]=sqrt(pow(ENU[3],2)*pow(Pecef[0],2)+pow(ENU[4],2)*pow(Pecef[1],2)+pow(ENU[5],2)*pow(Pecef[2],2));
	//Penu[2]=sqrt(pow(ENU[6],2)*pow(Pecef[0],2)+pow(ENU[7],2)*pow(Pecef[1],2)+pow(ENU[8],2)*pow(Pecef[2],2));

	double Decef[3 * 3] = { 0.0 }, temp1[9] = { 0.0 }, temp2[9] = { 0.0 };
	for (int i = 0;i<3;i++)
	{
		Decef[i * 3 + i] = pow(Pecef[i], 2);
	}

	matmul("TN", 3, 3, 3, 1.0, ENU, Decef, 0.0, temp1);
	matmul("NN", 3, 3, 3, 1.0, temp1, ENU, 0.0, temp2);

	for (int i = 0;i<3;i++)
	{
		Penu[i] = sqrt(temp2[i * 3 + i]);
	}
}

void pos2ecef(const double *pos, double *r)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
	double e2 = FE_WGS84*(2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2*sinp*sinp);

	r[0] = (v + pos[2])*cosp*cosl;
	r[1] = (v + pos[2])*cosp*sinl;
	r[2] = (v*(1.0 - e2) + pos[2])*sinp;
}

void diffpos(double blhpre[3], double blhcur[3], double denu[3])
{
	double rr0[3] = { 0.0 }, rr1[3] = { 0.0 }, drr[3] = { 0.0 };
	pos2ecef(blhpre, rr0);
	pos2ecef(blhcur, rr1);
	Mat_min(rr1, rr0, drr, 3, 1);
	ecef2enu(blhpre, drr, denu);
}

void difpos_b(double pospre[3], double poscur[3], double att[3], double dpos_b[3])
{
	double rr0[3] = { 0.0 }, rr1[3] = { 0.0 }, drr[3] = { 0.0 }, denu[3] = { 0 };
	pos2ecef(pospre, rr0);
	pos2ecef(poscur, rr1);
	Mat_min(rr1, rr0, drr, 3, 1);
	ecef2enu(pospre, drr, denu);

	double Cnb[9] = { 0 }, Cbn[9];
	a2mat(att, Cnb);
	Mat_tran(Cnb, 3, 3, Cbn);
	Mat_mul(Cbn, denu, 3, 3, 1, dpos_b);

}