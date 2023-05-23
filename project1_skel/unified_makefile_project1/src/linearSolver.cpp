#include "linearSolver.h"

// vector helper functions

std::size_t getIndex(std::size_t i, std::size_t j, std::size_t n) {
    return i * n + j;
}

double SparseMatrix::get(std::size_t i, std::size_t j) {
    std::size_t index = getIndex(i, j, m_num_columns);

    if (m_Elements.count(index)) {
        return m_Elements[index];
    } else {
        return 0.0;
    }
}

void SparseMatrix::set(std::size_t i, std::size_t j, double value) {
    std::size_t index = getIndex(i, j, m_num_columns);

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

SparseMatrix::SparseMatrix(std::size_t N) : m_num_rows(N), m_num_columns(N) {
    for (std::size_t i = 0; i < N; i++) {
        m_PerRow_ColumnIndices.push_back(std::set<std::size_t>());
        m_PerColumn_RowIndices.push_back(std::set<std::size_t>());
    }
}

SparseMatrix::SparseMatrix(std::size_t num_rows, std::size_t num_columns) : m_num_rows(num_rows), m_num_columns(num_columns) {
    for (std::size_t i = 0; i < num_rows; i++) {
        m_PerRow_ColumnIndices.push_back(std::set<std::size_t>());
    }
    for (std::size_t i = 0; i < num_columns; i++) {
        m_PerColumn_RowIndices.push_back(std::set<std::size_t>());
    }
}

void SparseMatrix::matVecMult(double x[], double r[]) {
    for (std::size_t i = 0; i < m_num_rows; i++) {
        r[i] = 0;
    }

    for (std::size_t i = 0; i < m_num_rows; i++) {
        for (std::size_t j : m_PerRow_ColumnIndices[i]) {
            std::size_t index = getIndex(i, j, m_num_columns);
            r[i] += m_Elements[index] * x[j];
        }
    }
}

void SparseMatrix::matTransVecMult(double x[], double r[]) {
    for (std::size_t i = 0; i < m_num_columns; i++) {
        r[i] = 0;
    }

    for (std::size_t j = 0; j < m_num_columns; j++) {
        for (std::size_t i : m_PerColumn_RowIndices[j]) {
            std::size_t index = getIndex(i, j, m_num_columns);
            r[j] += m_Elements[index] * x[i];
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

void vecTimesElementWise(int n, double v1[], double v2[])
{
  for (int i = 0; i < n; i++)
    v1[i] *= v2[i];
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

double VectorConjGrad(int m, int n, implicitMatrixWithTrans *J, double W[], double x[], double b[], 
		double epsilon,	// how low should we go?
		int    *steps)
{
  int		i, iMax;
  double	alpha, beta, rSqrLen, rSqrLenOld, u;

  double *r = (double *) malloc(sizeof(double) * m);
  double *d = (double *) malloc(sizeof(double) * m);
  double *t = (double *) malloc(sizeof(double) * m);
  double *temp = (double *) malloc(sizeof(double) * m);
  double *tempn = (double *) malloc(sizeof(double) * n);

  vecAssign(m, x, b);

  vecAssign(m, r, b);
  //JWJ^T
  J->matTransVecMult(x, tempn);
  vecTimesElementWise(n, tempn, W);
  J->matVecMult(tempn, temp);
  //A->matVecMult(x, temp);
  vecDiffEqual(m, r, temp);

  rSqrLen = vecSqrLen(m, r);

  vecAssign(m, d, r);

  i = 0;
  if (*steps)
    iMax = *steps;
  else
    iMax = MAX_STEPS;
		
  if (rSqrLen > epsilon)
    while (i < iMax) {	
      i++;
      //JWJ^T
      J->matTransVecMult(d, tempn);
      vecTimesElementWise(n, tempn, W);
      J->matVecMult(tempn, t);
      //A->matVecMult(d, t);
      u = vecDot(m, d, t);
      
      if (u == 0) {
        printf("(SolveConjGrad) d'Ad = 0\n");
        break;
      }
      
      // How far should we go?
      alpha = rSqrLen / u;
      
      // Take a step along direction d
      vecAssign(m, temp, d);
      vecTimesScalar(m, temp, alpha);
      vecAddEqual(m, x, temp);
      
      if (i & 0x3F) {
        vecAssign(m, temp, t);
        vecTimesScalar(m, temp, alpha);
        vecDiffEqual(m, r, temp);
      } else {
        // For stability, correct r every 64th iteration
        vecAssign(m, r, b);
        //JWJ^T
        J->matTransVecMult(x, tempn);
        vecTimesElementWise(n, tempn, W);
        J->matVecMult(tempn, temp);
        //A->matVecMult(x, temp);
        vecDiffEqual(m, r, temp);
      }
      
      rSqrLenOld = rSqrLen;
      rSqrLen = vecSqrLen(m, r);
      
      // Converged! Let's get out of here
      if (rSqrLen <= epsilon) break;			    
      
      // Change direction: d = r + beta * d
      beta = rSqrLen/rSqrLenOld;
      vecTimesScalar(m, d, beta);
      vecAddEqual(m, d, r);
    }
  
  // free memory

  free(r);
  free(d);
  free(t);
  free(temp);
  free(tempn);
		
  *steps = i;
  return(rSqrLen);
}
