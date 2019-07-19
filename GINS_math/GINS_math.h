#ifndef _GINS_MATH_H
#define _GINS_MATH_H

#include <math.h>
#include <string>
#include <vector>
#include <stdarg.h>
#include <algorithm> 
#include <time.h>
/*mat mul*/
void Mat_mul(double *a, double *b, int m, int n, int k, double *c);//c[m][k]=a[m][n]*b[n][k]
void Mat_mulb(double *a, int m, int n, double b, double *c);//a[m][n]=a[m][n]*b
/*mat add min*/
void Mat_add(double *a, double *b, double *c, int m, int n);//c[m][n]=a[m][n]+b[m][n]
void Mat_min(double *a, double *b, double *c, int m, int n); //c[m][n] = a[m][n] - b[m][n]
/*mat tran*/
void Mat_tran(double *a, int m, int n, double *b);//c[m][m]=a[m][m]T
/*mat inv*/
double Mat_inv(double a[], int n, double *b); //b[m][n] = inv(a[m][n]);
/*mat a=mat b*/
void Mat_equal(double *a, int m, int n, double *b);//a[m][n]=b[m][n]
void Mat_unit(double* a, int n);
extern void *__ml_zero(int size);
#endif
