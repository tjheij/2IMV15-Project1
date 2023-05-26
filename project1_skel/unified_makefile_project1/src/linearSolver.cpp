#include "linearSolver.h"

// vector helper functions

std::size_t getIndex(std::size_t i, std::size_t j, std::size_t n) {
    return i * n + j;
}

double SparseMatrix::get(std::size_t i, std::size_t j) {
    std::size_t index = getIndex(i, j, m_N);

    if (m_Elements.count(index)) {
        return m_Elements[index];
    } else {
        return 0.0;
    }
}

void SparseMatrix::set(std::size_t i, std::size_t j, double value) {
    std::size_t index = getIndex(i, j, m_N);

    if (value == 0.0) {
        m_Elements.erase(index);
        m_PerRow_ColumnIndices[i].erase(j);
        m_PerColumn_RowIndices[j].erase(i);
    } else {
        m_Elements[index] = value;
        m_PerRow_ColumnIndices[i].insert(j);
        m_PerColumn_RowIndices[j].insert(i);
    }
}

void SparseMatrix::add(std::size_t i, std::size_t j, double value) {
    set(i, j, get(i, j) + value);
}

SparseMatrix::SparseMatrix(std::size_t N) : m_N(N) {
    for (std::size_t i = 0; i < N; i++) {
        m_PerRow_ColumnIndices.push_back(std::set<std::size_t>());
        m_PerColumn_RowIndices.push_back(std::set<std::size_t>());
    }
}

void SparseMatrix::matVecMult(double x[], double r[]) {
    for (std::size_t i = 0; i < m_N; i++) {
        r[i] = 0;
    }

    for (std::size_t i = 0; i < m_N; i++) {
        for (std::size_t j : m_PerRow_ColumnIndices[i]) {
            std::size_t index = getIndex(i, j, m_N);
            r[i] += m_Elements[index] * x[j];
        }
    }
}

void SparseMatrix::matTransVecMult(double x[], double r[]) {
    for (std::size_t i = 0; i < m_N; i++) {
        r[i] = 0;
    }

    for (std::size_t j = 0; j < m_N; j++) {
        for (std::size_t i : m_PerColumn_RowIndices[j]) {
            std::size_t index = getIndex(j, i, m_N);
            r[i] += m_Elements[index] * x[j];
        }
    }
}

void vecAddEqual(int n, double r[], double v[])
{
  for (int i = 0; i < n; i++)
    r[i] = r[i] + v[i];
}

void vecDiffEqual(int n, double r[], double v[])
{
  for (int i = 0; i < n; i++)
    r[i] = r[i] - v[i];
}

void vecAssign(int n, double v1[], double v2[])
{
  for (int i = 0; i < n; i++)
    v1[i] = v2[i];
}

void vecTimesScalar(int n, double v[], double s)
{
  for (int i = 0; i < n; i++)
    v[i] *= s;
}

double vecDot(int n, double v1[], double v2[])
{
  double dot = 0;
  for (int i = 0; i < n; i++)
    dot += v1[i] * v2[i];
  return dot;
}

double vecSqrLen(int n, double v[])
{
  return vecDot(n, v, v);
}

double ConjGrad(int n, implicitMatrix *A, double x[], double b[], 
		double epsilon,	// how low should we go?
		int    *steps)
{
  int		i, iMax;
  double	alpha, beta, rSqrLen, rSqrLenOld, u;

  double *r = (double *) malloc(sizeof(double) * n);
  double *d = (double *) malloc(sizeof(double) * n);
  double *t = (double *) malloc(sizeof(double) * n);
  double *temp = (double *) malloc(sizeof(double) * n);

  vecAssign(n, x, b);

  vecAssign(n, r, b);
  A->matVecMult(x, temp);
  vecDiffEqual(n, r, temp);

  rSqrLen = vecSqrLen(n, r);

  vecAssign(n, d, r);

  i = 0;
  if (*steps)
    iMax = *steps;
  else
    iMax = MAX_STEPS;
		
  if (rSqrLen > epsilon)
    while (i < iMax) {	
      i++;
      A->matVecMult(d, t);
      u = vecDot(n, d, t);
      
      if (u == 0) {
	printf("(SolveConjGrad) d'Ad = 0\n");
	break;
      }
      
      // How far should we go?
      alpha = rSqrLen / u;
      
      // Take a step along direction d
      vecAssign(n, temp, d);
      vecTimesScalar(n, temp, alpha);
      vecAddEqual(n, x, temp);
      
      if (i & 0x3F) {
	vecAssign(n, temp, t);
	vecTimesScalar(n, temp, alpha);
	vecDiffEqual(n, r, temp);
      } else {
	// For stability, correct r every 64th iteration
	vecAssign(n, r, b);
	A->matVecMult(x, temp);
	vecDiffEqual(n, r, temp);
      }
      
      rSqrLenOld = rSqrLen;
      rSqrLen = vecSqrLen(n, r);
      
      // Converged! Let's get out of here
      if (rSqrLen <= epsilon)
	break;			    
      
      // Change direction: d = r + beta * d
      beta = rSqrLen/rSqrLenOld;
      vecTimesScalar(n, d, beta);
      vecAddEqual(n, d, r);
    }
  
  // free memory

  free(r);
  free(d);
  free(t);
  free(temp);
		
  *steps = i;
  return(rSqrLen);
}


