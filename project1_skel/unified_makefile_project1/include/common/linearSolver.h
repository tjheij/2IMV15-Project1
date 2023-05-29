#ifndef LINEAR_SOLVER_H
#define LINEAR_SOLVER_H

#include <math.h> 
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <set>

// Karen's CGD

#define MAX_STEPS 100

// Matrix class the solver will accept
class implicitMatrix
{
 public:
  virtual void matVecMult(double x[], double r[]) = 0;
};

// Matrix class the solver will accept
class implicitMatrixWithTrans : public implicitMatrix
{
 public:
  virtual void matVecMult(double x[], double r[]) = 0;
  virtual void matTransVecMult(double x[], double r[]) = 0;
};



// Solve Ax = b for a symmetric, positive definite matrix A
// A is represented implicitely by the function "matVecMult"
// which performs a matrix vector multiple Av and places result in r
// "n" is the length of the vectors x and b
// "epsilon" is the error tolerance
// "steps", as passed, is the maximum number of steps, or 0 (implying MAX_STEPS)
// Upon completion, "steps" contains the number of iterations taken
double ConjGrad(int n, implicitMatrix *A, double x[], double b[], 
		double epsilon,	// how low should we go?
		int    *steps);

double VectorConjGrad(int m, int n, implicitMatrixWithTrans *J, double W[], double x[], double b[], double epsilon, int *steps);

// Some vector helper functions
void vecAddEqual(int n, double r[], double v[]);
void vecDiffEqual(int n, double r[], double v[]);
void vecAssign(int n, double v1[], double v2[]);
void vecTimesScalar(int n, double v[], double s);
void vecTimesElementWise(int n, double v1[], double v2[]);
double vecDot(int n, double v1[], double v2[]);
double vecSqrLen(int n, double v[]);

class SparseMatrix : public implicitMatrixWithTrans {
public:
    const std::size_t m_num_rows;
    const std::size_t m_num_columns;
    std::map<std::size_t, double> m_Elements;
    std::vector<std::set<std::size_t> > m_PerRow_ColumnIndices;
    std::vector<std::set<std::size_t> > m_PerColumn_RowIndices;

    double get(std::size_t i, std::size_t j);
    void set(std::size_t i, std::size_t j, double value);
    void add(std::size_t i, std::size_t j, double value);

    SparseMatrix(std::size_t N);
    SparseMatrix(std::size_t num_rows, std::size_t num_columns);
    virtual ~SparseMatrix() = default;
    virtual void matVecMult(double x[], double r[]);
    virtual void matTransVecMult(double x[], double r[]);
};

#endif
