#include "GINS_math.h"
void Mat_mul(double *ain, double *bin, int m, int n, int k, double *c)
{
	int i, j, l, u;
	double *a = (double *)malloc(m*n * sizeof(double));
	double *b = (double *)malloc(k*n * sizeof(double));
	for (i = 0; i<m*n; i++)
		a[i] = ain[i];
	for (i = 0; i<k*n; i++)
		b[i] = bin[i];

	for (i = 0; i <= m - 1; i++)
		for (j = 0; j <= k - 1; j++)
		{
			u = i*k + j;
			c[u] = 0.0;
			for (l = 0; l <= n - 1; l++)
				c[u] = c[u] + a[i*n + l] * b[l*k + j];
		}
	free(a);
	free(b);
}

void Mat_mulb(double *a, int m, int n, double b, double *c)
{
	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			*(c + i*n + j) = *(a + i*n + j)*b;
		}
	}
}


void Mat_add(double *a, double *b, double *c, int m, int n)
{
	for (int i = 0; i<m; i++)
		for (int j = 0; j<n; j++)
		{
			*(c + i*n + j) = *(a + i*n + j) + *(b + i*n + j);
		}
}

void Mat_min(double *a, double *b, double *c, int m, int n)
{
	int i, j;

	for (i = 0; i<m; i++)
		for (j = 0; j<n; j++)
		{
			*(c + i*n + j) = *(a + i*n + j) - *(b + i*n + j);
		}
}

void Mat_tran(double *a, int m, int n, double *b)
{
	for (int l = 0; l<n; l++)
		for (int k = 0; k<m; k++)
		{
			b[l*m + k] = a[k*n + l];
		}

}



double Mat_inv(double a[], int n, double *b)
{
	for (int i = 0; i<n; i++)
		for (int j = 0; j<n; j++)
		{
			*(b + i*n + j) = *(a + i*n + j);
		}

	int *is, *js, i, j, k, l, u, v;
	double d, p;
	is = (int *)malloc(n * sizeof(int));
	js = (int *)malloc(n * sizeof(int));
	for (k = 0; k <= n - 1; k++)
	{
		d = 0.0;
		for (i = k; i <= n - 1; i++)
			for (j = k; j <= n - 1; j++)
			{
				l = i*n + j;
				p = fabs(b[l]);
				if (p>d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		if (d == 0)//d+1.0==1.0 lcc
		{
			free(is);
			free(js);
			//log("\nerror:inverse matrix is not exist\n");
			return (0);
		}
		if (is[k] != k)
			for (j = 0; j <= n - 1; j++)
			{
				u = k*n + j; v = is[k] * n + j;
				p = b[u]; b[u] = b[v]; b[v] = p;
			}
		if (js[k] != k)
			for (i = 0; i <= n - 1; i++)
			{
				u = i*n + k;
				v = i*n + js[k];
				p = b[u];
				b[u] = b[v]; b[v] = p;
			}
		l = k*n + k;
		b[l] = 1.0 / b[l];
		for (j = 0; j <= n - 1; j++)
			if (j != k)
			{
				u = k*n + j;
				b[u] = b[u] * b[l];
			}
		for (i = 0; i <= n - 1; i++)
			if (i != k)
				for (j = 0; j <= n - 1; j++)
					if (j != k)
					{
						u = i*n + j;
						b[u] = b[u] - b[i*n + k] * b[k*n + j];
					}
		for (i = 0; i <= n - 1; i++)
			if (i != k)
			{
				u = i*n + k; b[u] = -b[u] * b[l];
			}
	}
	for (k = n - 1; k >= 0; k--)
	{
		if (js[k] != k)
			for (j = 0; j <= n - 1; j++)
			{
				u = k*n + j; v = js[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		if (is[k] != k)
			for (i = 0; i <= n - 1; i++)
			{
				u = i*n + k;
				v = i*n + is[k];
				p = b[u];
				b[u] = b[v]; b[v] = p;
			}
	}

	free(is);
	free(js);
	return(1);
}

void Mat_equal(double *a, int m, int n, double *b)
{
	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			*(a + i*n + j) = *(b + i*n + j);
		}
	}
}

extern void *__ml_zero(int size)
{
	void *p = malloc(size);
	if (p == NULL) {
		//gilc_log("not malloc for val\n");
	}
	memset(p, 0, size);
	return p;
}

void Mat_unit(double* a, int n)
{
	for (int i = 0; i<n; i++)
	{
		for (int j = 0; j<n; j++)
		{
			a[i*n + j] = 0.0;
		}
	}
	for (int i = 0; i<n; i++)
	{
		a[i*n + i] = 1.0;
	}
}