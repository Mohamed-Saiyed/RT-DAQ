#include "main.h"

#include <stdio.h>
#include "string.h"
#include "konfig.h"

#include "Matrix.h"

void MatrixInitEmpty(Matrix* matrix, const int16_t _i16row, const int16_t _i16col, const InitZero _init)
{
    matrix->i16row = _i16row;
    matrix->i16col = _i16col;
    
    if (_init == InitMatWithZero)
	{
        MatrixSetHomogen(matrix , 0.0);
    }
}

void MatrixInitData(Matrix* matrix, const int16_t _i16row, const int16_t _i16col, const float_prec* initData, const InitZero _init) 
{
    matrix->i16row = _i16row;
    matrix->i16col = _i16col;
    
    if (_init == InitMatWithZero)
	{
        MatrixSetHomogen(matrix , 0.0);
    }
	
    for (int16_t _i = 0; _i < matrix->i16row; _i++)
	{
        for (int16_t _j = 0; _j < matrix->i16col; _j++)
		{
            matrix->floatData[_i][_j] = *initData;
            initData++;
        }
    }
}

void MatrixCopyOld(Matrix* matrixnew , const Matrix* matrixold)
{
    /* For copy contructor, we only need to copy (_i16row x _i16col) submatrix, there's no need to copy all data */
    matrixnew->i16row = matrixold->i16row;
    matrixnew->i16col = matrixold->i16col;
    
    const float_prec *sourc = matrixold->floatData[0];
    float_prec *desti = matrixnew->floatData[0];

    for (int16_t _i = 0; _i < matrixold->i16row; _i++)
	{
        /* Still valid with invalid matrix ((i16row == -1) or (i16col == -1)) */
        memcpy(desti, sourc, sizeof(float_prec)*sizeof((matrixnew->i16col)));
        sourc += (MATRIX_MAXIMUM_SIZE);
        desti += (MATRIX_MAXIMUM_SIZE);
    }
}

Matrix* MatrixCopy(Matrix* matrixnew , const Matrix* matrixold)
{
    /* For assignment operator, we only need to copy (_i16row x _i16col) submatrix, there's no need to copy all data */
    matrixnew->i16row = matrixold->i16row;
    matrixnew->i16col = matrixold->i16col;
    
    const float_prec *sourc = matrixold->floatData[0];
    float_prec *desti = matrixnew->floatData[0];

    for (int16_t _i = 0; _i < matrixold->i16row; _i++)
	{
        /* Still valid with invalid matrix ((i16row == -1) or (i16col == -1)) */
        memcpy(desti, sourc, sizeof(float_prec)*sizeof((matrixold->i16col)));
        sourc += (MATRIX_MAXIMUM_SIZE);
        desti += (MATRIX_MAXIMUM_SIZE);
    }
    
    return matrixnew;
}

int16_t Matrixi16getRow(Matrix* matrix)
{
	return matrix->i16row; 
}
 
int16_t Matrixi16getCol(Matrix* matrix)
{
	return matrix->i16col; 
}

/* Invers operation using Gauss-Jordan algorithm */
Matrix MatrixInvers(Matrix* matrix)
{
	Matrix _outp;
	Matrix _temp;
	
	MatrixInitEmpty(&_outp, matrix->i16row, matrix->i16col, NoInitMatZero);
	MatrixInitEmpty(&_temp, matrix->i16row, matrix->i16col, NoInitMatZero);
	
    MatrixSetIdentity(&_outp);
    
    /* Gauss Elimination... */
    for (int16_t _j = 0; _j < (_temp.i16row)-1; _j++)
	{
        for (int16_t _i = _j+1; _i < _temp.i16row; _i++)
		{
            if (fabs(_temp.floatData[_j][_j]) < (float_prec)float_prec_ZERO)
			{
                /* Matrix is non-invertible */
                MatrixSetMatrixInvalid(&_outp);
                return _outp;
            }

            float_prec _tempfloat = _temp.floatData[_i][_j] / _temp.floatData[_j][_j];

            for (int16_t _k = 0; _k < _temp.i16col; _k++)
			{
                _temp.floatData[_i][_k] -= (_temp.floatData[_j][_k] * _tempfloat);
                _outp.floatData[_i][_k] -= (_outp.floatData[_j][_k] * _tempfloat);

                MatrixRoundingElementToZero(&_temp,_i, _k);
                MatrixRoundingElementToZero(&_outp,_i, _k);
            }

        }
    }

    #if (1)
        /* Here, the _temp matrix should be an upper triangular matrix.
         * But because of rounding error, it might not.
         */
        for (int16_t _i = 1; _i < _temp.i16row; _i++)
		{
            for (int16_t _j = 0; _j < _i; _j++)
			{
                 _temp.floatData[_i][_j] = 0.0;
            }
        }
    #endif


    /* Jordan... */
    for (int16_t _j = (_temp.i16row)-1; _j > 0; _j--)
	{
        for (int16_t _i = _j-1; _i >= 0; _i--)
		{
            if (fabs(_temp.floatData[_j][_j]) < (float_prec)float_prec_ZERO)
			{
                /* Matrix is non-invertible */
                MatrixSetMatrixInvalid(&_outp);
                return _outp;
            }

            float_prec _tempfloat = _temp.floatData[_i][_j] / _temp.floatData[_j][_j];
           _temp.floatData[_i][_j] -= (_temp.floatData[_j][_j] * _tempfloat);
            MatrixRoundingElementToZero(&_temp,_i, _j);

            for (int16_t _k = (_temp.i16row - 1); _k >= 0; _k--) 
			{
                _outp.floatData[_i][_k] -= (_outp.floatData[_j][_k] * _tempfloat);
                MatrixRoundingElementToZero(&_outp,_i, _k);
            }
        }
    }


    /* Normalization */
    for (int16_t _i = 0; _i < _temp.i16row; _i++) 
	{
        if (fabs(_temp.floatData[_i][_i]) < (float_prec)float_prec_ZERO)
		{
            /* Matrix is non-invertible */
            MatrixSetMatrixInvalid(&_outp);
            return _outp;
        }

        float_prec _tempfloat = _temp.floatData[_i][_i];
        _temp.floatData[_i][_i] = 1.0;

        for (int16_t _j = 0; _j < _temp.i16row; _j++) 
		{
            _outp.floatData[_i][_j] /= _tempfloat;
        }
    }
    return _outp;
}

Matrix MatrixCholeskyDec(Matrix* matrix)
{
	
    float_prec _tempFloat;

    /* Note that _outp need to be initialized as zero matrix */
	Matrix _outp;
    MatrixInitEmpty(&_outp, matrix->i16row, matrix->i16col, NoInitMatZero);
    
    if (matrix->i16row != matrix->i16col)
	{
        MatrixSetMatrixInvalid(&_outp);
        return _outp;
    }
    for (int16_t _j = 0; _j < matrix->i16col; _j++)
	{
        for (int16_t _i = _j; _i < matrix->i16row; _i++)
		{
            _tempFloat = matrix->floatData[_i][_j];
            if (_i == _j)
			{
                for (int16_t _k = 0; _k < _j; _k++)
				{
                    _tempFloat = _tempFloat - (_outp.floatData[_i][_k] *_outp.floatData[_i][_k]);
                }
                if (_tempFloat < -(float_prec)float_prec_ZERO)
				{
                    /* Matrix is not positif (semi)definit */
                    MatrixSetMatrixInvalid(&_outp);
                    return _outp;
                }
                /* Rounding to zero to avoid case where sqrt(0-) */
                if (fabs(_tempFloat) < (float_prec)float_prec_ZERO)
				{
                    _tempFloat = 0.0;
                }
                _outp.floatData[_i][_i]  = sqrt(_tempFloat);
            } 
			else
			{
                for (int16_t _k = 0; _k < _j; _k++)
				{
                    _tempFloat = _tempFloat - (_outp.floatData[_i][_k] * _outp.floatData[_j][_k]);
                }
                if (fabs(_outp.floatData[_j][_j]) < (float_prec)float_prec_ZERO)
				{
                    /* Matrix is not positif definit */
                    MatrixSetMatrixInvalid(&_outp);
                    return _outp;
                }
                _outp.floatData[_i][_j] = _tempFloat / _outp.floatData[_j][_j];
            }
        }
    }
    return _outp;
}

Matrix MatrixInsertVector(Matrix* matrix, const Matrix* _Vector, const int16_t _posCol)
{
	Matrix _outp;
	MatrixCopyOld(&_outp , matrix);
	
    if ((_Vector->i16row > matrix->i16row) || (_Vector->i16col + _posCol > matrix->i16col))
	{
        /* Return false */
        MatrixSetMatrixInvalid(&_outp);
        return _outp;
    }
    for (int16_t _i = 0; _i < _Vector->i16row; _i++)
	{
		_outp.floatData[_i][_posCol] = _Vector->floatData[_i][0];
    }
	
    return _outp;
}

Matrix MatrixInsertSubMatrix(Matrix* matrix, const Matrix* _subMatrix, const int16_t _posRow, const int16_t _posCol)
{
	Matrix _outp;
    MatrixCopyOld(&_outp , matrix);
   
    if (((_subMatrix->i16row + _posRow) > matrix->i16row) || ((_subMatrix->i16col + _posCol) > matrix->i16col)) {
        /* Return false */
        MatrixSetMatrixInvalid(&_outp);
        return _outp;
    }
    for (int16_t _i = 0; _i < _subMatrix->i16row; _i++)
	{
        for (int16_t _j = 0; _j < _subMatrix->i16col; _j++)
		{
			_outp.floatData[_i + _posRow][_j + _posCol] = _subMatrix->floatData[_i][_j];
        }
    }
    return _outp;
}

void MatrixSetIdentity(Matrix* matrix)
{
	MatrixSetDiag(matrix , 1.0);
}

void MatrixSetDiag(Matrix* matrix , const float_prec _val)
{
    for (int16_t _i = 0; _i < matrix->i16row; _i++)
	{
        for (int16_t _j = 0; _j < matrix->i16col; _j++)
		{
            if (_i == _j)
			{
                 matrix->floatData[_i][_j] = _val;
            } else
			{
                 matrix->floatData[_i][_j] = 0.0;
            }
        }
    }
}

void MatrixSetMatrixInvalid(Matrix* matrix)
{
    matrix->i16row = -1;
    matrix->i16col = -1;
}

void MatrixSetHomogen(Matrix* matrix, const float_prec _val)
{
    for (int16_t _i = 0; _i < matrix->i16row; _i++)
	{
        for (int16_t _j = 0; _j < matrix->i16col; _j++)
		{
            matrix->floatData[_i][_j] = _val;
        }
    }
}

void MatrixSetToZero(Matrix* matrix)
{
    MatrixSetHomogen(matrix , 0.0);
}

uint8_t MatrixIsValid(Matrix* matrix) 
{
	  uint8_t retval = 0 ;
    /* Check whether the matrix is valid or not */
    if ((matrix->i16row > 0) && (matrix->i16row <= MATRIX_MAXIMUM_SIZE) &&
        (matrix->i16col > 0) && (matrix->i16col <= MATRIX_MAXIMUM_SIZE))
    {
        retval =  true ;
    }
	else
	{
        retval = false ;
    }
	return retval;
}

Matrix MatrixAddFactor(const float_prec _scalar, const Matrix* _mat)
{
	Matrix _outp;
    MatrixInitEmpty(&_outp, _mat->i16row, _mat->i16col, NoInitMatZero);
 
    for (int16_t _i = 0; _i < _mat->i16row ; _i++)
	{
        for (int16_t _j = 0; _j < _mat->i16col; _j++)
		{
            _outp.floatData[_i][_j] = _scalar + _mat->floatData[_i][_j];
        }
    }
	
    return _outp;
}

Matrix MatrixSubtractFactor(const float_prec _scalar, const Matrix* _mat)
{
	Matrix _outp;
    MatrixInitEmpty(&_outp, _mat->i16row, _mat->i16col, NoInitMatZero);
 
    for (int16_t _i = 0; _i < _mat->i16row ; _i++)
	{
        for (int16_t _j = 0; _j < _mat->i16col; _j++)
		{
            _outp.floatData[_i][_j] = _mat->floatData[_i][_j] - _scalar;
        }
    }
	
    return _outp;
}

Matrix MatrixMultiplyFactor(const float_prec _scalar, const Matrix* _mat)
{
	Matrix _outp;
    MatrixInitEmpty(&_outp, _mat->i16row, _mat->i16col, NoInitMatZero);
 
    for (int16_t _i = 0; _i < _mat->i16row ; _i++)
	{
        for (int16_t _j = 0; _j < _mat->i16col; _j++)
		{
            _outp.floatData[_i][_j] = _scalar * _mat->floatData[_i][_j];
        }
    }
	
    return _outp;
}

Matrix MatrixDivideFactor(const float_prec _scalar, const Matrix* _mat)
{
	Matrix _outp;
    MatrixInitEmpty(&_outp, _mat->i16row, _mat->i16col, NoInitMatZero);
 
    for (int16_t _i = 0; _i < _mat->i16row ; _i++)
	{
        for (int16_t _j = 0; _j < _mat->i16col; _j++)
		{
            _outp.floatData[_i][_j] = _mat->floatData[_i][_j] / _scalar;
        }
    }
	
    return _outp;
}

Matrix MatrixAddMatrix(Matrix* this , const Matrix* _matAdd)
{
    Matrix _outp;
    MatrixInitEmpty(&_outp, _matAdd->i16row, _matAdd->i16col, NoInitMatZero);
 
    if ((this->i16row != _matAdd->i16row) || (this->i16col != _matAdd->i16col))
	{
        MatrixSetMatrixInvalid(&_outp);
        return _outp;
    }

    for (int16_t _i = 0; _i < this->i16row; _i++) 
	{
        for (int16_t _j = 0; _j < this->i16col; _j++)
		{
            _outp.floatData[_i][_j] = this->floatData[_i][_j] + _matAdd->floatData[_i][_j];
        }
    }
    return _outp;
}

Matrix MatrixSubtractMatrix(Matrix* this , const Matrix* _mat)
{
    Matrix _outp;
    MatrixInitEmpty(&_outp, _mat->i16row, _mat->i16col, NoInitMatZero);
 
    if ((this->i16row != _mat->i16row) || (this->i16col != _mat->i16col))
	{
        MatrixSetMatrixInvalid(&_outp);
        return _outp;
    }

    for (int16_t _i = 0; _i < this->i16row; _i++) 
	{
        for (int16_t _j = 0; _j < this->i16col; _j++)
		{
            _outp.floatData[_i][_j] = this->floatData[_i][_j] - _mat->floatData[_i][_j];
        }
    }
    return _outp;
}

Matrix MatrixMultiplyMatrix(Matrix* this , const Matrix* _mat)
{
    Matrix _outp;
    MatrixInitEmpty(&_outp, _mat->i16row, _mat->i16col, NoInitMatZero);
 
    if ((this->i16col != _mat->i16row))
	{
        MatrixSetMatrixInvalid(&_outp);
        return _outp;
    }

    for (int16_t _i = 0; _i < this->i16row; _i++)
	{
        for (int16_t _j = 0; _j < _mat->i16col; _j++)
		{
             _outp.floatData[_i][_j] = 0.0;
            for (int16_t _k = 0; _k < this->i16col; _k++)
			{
                 _outp.floatData[_i][_j] += this->floatData[_i][_k] * _mat->floatData[_k][_j];
            }
        }
    }
    return _outp;
}

Matrix MatrixChangeSign(Matrix* this , const float_prec _scalar)
{
    Matrix _outp;
    MatrixInitEmpty(&_outp, this->i16row, this->i16col, NoInitMatZero);

   for (int16_t _i = 0; _i < this->i16row; _i++)
   {
        for (int16_t _j = 0; _j < this->i16col; _j++)
		{
            _outp.floatData[_i][_j] = -this->floatData[_i][_j];
        }
    }
    return _outp;
}

uint8_t MatrixCompare(Matrix* this , const Matrix* _compare)
{
    if ((this->i16row != _compare->i16row) || (this->i16col != _compare->i16col)) 
	{
        return false;
    }

    for (int16_t _i = 0; _i < this->i16row; _i++) 
	{
        for (int16_t _j = 0; _j < this->i16col; _j++)
		{
            if (fabs(this->floatData[_i][_j]) - _compare->floatData[_i][_j] > (float_prec)float_prec_ZERO_ECO)
			{
                return false;
            }
        }
    }
    return true;
}

uint8_t MatrixDoesnotEqual(Matrix* this ,const Matrix* _compare)
{
    return (!(MatrixCompare(this,_compare)));
}

uint8_t MatrixNormVector(Matrix* this)
{
    float_prec _normM = 0.0;
	
    for (int16_t _i = 0; _i < this->i16row; _i++)
	{
        for (int16_t _j = 0; _j < this->i16col; _j++)
		{
            _normM = _normM + (this->floatData[_i][_j] * this->floatData[_i][_j]);
        }
    }

    /* Rounding to zero to avoid case where sqrt(0-), and _normM always positive */
    if (_normM < (float_prec)float_prec_ZERO)
	{
        return false;
    }
    _normM = sqrt(_normM);
    for (int16_t _i = 0; _i < this->i16row; _i++)
	{
        for (int16_t _j = 0; _j < this->i16col; _j++)
		{
            this->floatData[_i][_j] /= _normM;
        }
    }
    return true;
}

Matrix MatrixTranspose(Matrix* matrix)
{
	Matrix _outp;
    MatrixInitEmpty(&_outp, matrix->i16row, matrix->i16col, NoInitMatZero);
	
    for (int16_t _i = 0; _i < matrix->i16row; _i++) 
	{
        for (int16_t _j = 0; _j < matrix->i16col; _j++) 
		{
            _outp.floatData[_j][_i] = matrix->floatData[_i][_j];
        }
    }
    return _outp;
}


void MatrixRoundingElementToZero(Matrix* matrix, const int16_t _i, const int16_t _j)
{
    if (fabs(matrix->floatData[_i][_j]) < (float_prec)float_prec_ZERO_ECO)
	  {
        matrix->floatData[_i][_j]= 0.0;
    }
}
