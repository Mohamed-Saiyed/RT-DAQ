#ifndef MATRIX_H
#define MATRIX_H

#include "konfig.h"

#define true	1
#define false	0


typedef enum
{
    InitMatWithZero,    /* Initialize matrix with zero data */
    NoInitMatZero
		
} InitZero;
    
	
typedef struct 
{	

	int16_t i16row;
    int16_t i16col;
    float_prec floatData[MATRIX_MAXIMUM_SIZE][MATRIX_MAXIMUM_SIZE];
	
}Matrix;

Matrix MatrixCholeskyDec(Matrix* matrix);
Matrix MatrixTranspose(Matrix* matrix);
uint8_t MatrixNormVector(Matrix* this);
uint8_t MatrixDoesnotEqual(Matrix* this ,const Matrix* _compare);
uint8_t MatrixCompare(Matrix* this , const Matrix* _compare);
Matrix MatrixChangeSign(Matrix* this , const float_prec _scalar);
Matrix MatrixMultiplyMatrix(Matrix* this , const Matrix* _matAdd);
Matrix MatrixSubtractMatrix(Matrix* this , const Matrix* _matAdd);
Matrix MatrixAddMatrix(Matrix* this , const Matrix* _matAdd);
Matrix MatrixDivideFactor(const float_prec _scalar, const Matrix* _mat);
Matrix MatrixMultiplyFactor(const float_prec _scalar, const Matrix* _mat);
Matrix MatrixSubtractFactor(const float_prec _scalar, const Matrix* _mat);
Matrix MatrixAddFactor(const float_prec _scalar, const Matrix* _mat);
uint8_t MatrixIsValid(Matrix* matrix) ;
void MatrixSetHomogen(Matrix* matrix, const float_prec _val);
void MatrixSetMatrixInvalid(Matrix* matrix);
void MatrixSetDiag(Matrix* matrix , const float_prec _val);
void MatrixSetIdentity(Matrix* matrix);
Matrix MatrixInsertSubMatrix(Matrix* matrix, const Matrix* _subMatrix, const int16_t _posRow, const int16_t _posCol);
Matrix MatrixInsertVector(Matrix* matrix, const Matrix* _Vector, const int16_t _posCol);
Matrix MatrixCholeskyDec(Matrix* matrix);
Matrix MatrixInvers(Matrix* matrix);
int16_t Matrixi16getCol(Matrix* matrix);
int16_t Matrixi16getRow(Matrix* matrix);
Matrix* MatrixCopy(Matrix* matrixnew , const Matrix* matrixold);
void MatrixCopyOld(Matrix* matrixnew , const Matrix* matrixold);
void MatrixInitData(Matrix* matrix, const int16_t _i16row, const int16_t _i16col, const float_prec* initData, const InitZero _init) ;
void MatrixInitEmpty(Matrix* matrix, const int16_t _i16row, const int16_t _i16col, const InitZero _init);
void MatrixRoundingElementToZero(Matrix* matrix, const int16_t _i, const int16_t _j);

#endif /*MATRIX_H*/

