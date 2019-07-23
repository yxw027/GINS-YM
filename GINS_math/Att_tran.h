#ifndef _ATT_TRAN_H
#define _ATT_TRAN_H
#include <math.h>
#include "GINS_math.h"
#include "GINS_process.h"
void xyz2enu(const double *pos, double *E);
void pos2ecef(const double *pos, double *r); //blh2xyz
void matmul(const char *tr, int n, int k, int m, double alpha,
	const double *A, const double *B, double beta, double *C);
void Var_XYZ2BLH(double xyz[3], double Pecef[3], double Penu[3]);
void ecef2enu(const double *pos, const double *r, double *e); //xyz坐标系向量r转化到enu
void m2qua_ned(double m[], double q[]);//方向余弦矩阵转四元数ned
void q2mat_ned(double qua[], double m[]);//四元数转方向余弦矩阵ned
void m2att_ned(double m[], double a[]);//方向余弦矩阵转欧拉角ned
void a2mat_ned(double att[], double m[]);//欧拉角转方向余弦矩阵ned
void a2mat(double att[], double m[]);//欧拉角2方向余弦矩阵enu
void a2qua(double att[], double qua[]);//欧拉角2四元数enu
void m2att(double mat[], double att[]);//方向余弦矩阵2欧拉角enu
void m2qua(double mat[], double qua[]);//方向余弦矩阵2四元数enu
void q2att(double qua[], double att[]);//四元数2欧拉角
void q2mat(double qua[], double mat[]);//四元数2方向余弦矩阵
void askew(double v[], double m[]);
void qdelphi(double qpb[], double phi[]);
void rv2q(double rv[], double q[]);
void qmuln(double q1[], double q2[], double q[]);
//姿态更新，连乘两次
void qupdt2(double qnb0[], double rv_ib[], double rv_in[], double qnb1[]);


void difpos_b(double pospre[3], double poscur[3], double att[3], double dpos_b[3]);
void diffpos(double blhpre[3], double blhcur[3], double denu[3]);
#endif
