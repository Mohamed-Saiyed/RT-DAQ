#include "main.h" 
#include "Matrix.h"
#include "konfig.h"

#include "UKF.h"

typedef struct
{
	Matrix X_Est;
	Matrix X_Sigma;
	
	Matrix Y_Est;
	Matrix Y_Sigma;
	
	Matrix P;
	Matrix P_Chol;
	
	Matrix DX;
	Matrix DY;
	
	Matrix Py;
	Matrix Pxy;
	
	Matrix Wm;
	Matrix Wc;
	
	Matrix Rv;
	Matrix Rn;
	
	Matrix Err;
	Matrix Gain;
		
	uint8_t  (*bNonlinearUpdateX) (Matrix* X_dot, const Matrix* X, const Matrix* U);
	uint8_t  (*bNonlinearUpdateY) (Matrix* Y_Est, const Matrix* X, const Matrix* U);
	
}UKF;

float_prec Gamma;

UKF	UKFInit;

void UKF(UKF* this, const Matrix* XInit, const Matrix* PInit, const Matrix* Rv, const Matrix* Rn,
        uint8_t (*bNonlinearUpdateX)(Matrix* , const Matrix* , const Matrix* ),
        uint8_t (*bNonlinearUpdateY)(Matrix* , const Matrix* , const Matrix* ))
{
	/* Initialization:
     *  P(k=0|k=0) = Identity matrix * covariant(P(k=0)), typically initialized with some big number.
     *  x(k=0|k=0) = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
     *  Rv, Rn     = Covariance matrices of process & measurement. As this implementation 
     *                the noise as AWGN (and same value for every variable), this is set
     *                to Rv=diag(RvInit,...,RvInit) and Rn=diag(RnInit,...,RnInit).
     */
    this->X_Est = XInit;
    this->P     = PInit;
    this->Rv    = Rv;
    this->Rn    = Rn;
    this->bNonlinearUpdateX = bNonlinearUpdateX;
    this->bNonlinearUpdateY = bNonlinearUpdateY;
    
    
    /* Van der. Merwe, .. (2004). Sigma-Point Kalman Filters for Probabilistic Inference in Dynamic State-Space Models 
     * (Ph.D. thesis). Oregon Health & Science University. Page 6:
     * 
     * where λ = α2(L+κ)−L is a scaling parameter. α determines the spread of the sigma points around ̄x and is usually
     * set to a small positive value (e.g. 1e−2 ≤ α ≤ 1). κ is a secondary scaling parameter which is usually set to
     * either 0 or 3−L (see [45] for details), and β is an extra degree of freedom scalar parameter used to 
     * incorporate any extra prior knowledge of the distribution of x (for Gaussian distributions, β = 2 is optimal).
     */
    float_prec _alpha   = 1e-2;
    float_prec _k       = 0.0;
    float_prec _beta    = 2.0;
    
    /* lambda = (alpha^2)*(N+kappa)-N,         gamma = sqrt(N+alpha)            ...{UKF_1} */
    float_prec _lambda  = (_alpha*_alpha)*(SS_X_LEN+_k) - SS_X_LEN;
    Gamma = sqrt((SS_X_LEN + _lambda));


    /* Wm = [lambda/(N+lambda)         1/(2(N+lambda)) ... 1/(2(N+lambda))]     ...{UKF_2} */
    Wm[0][0] = _lambda/(SS_X_LEN + _lambda);
    for (int16_t _i = 1; _i < Wm.i16getCol(); _i++)
	{
        Wm[0][_i] = 0.5/(SS_X_LEN + _lambda);
    }
    
    /* Wc = [Wm(0)+(1-alpha(^2)+beta)  1/(2(N+lambda)) ... 1/(2(N+lambda))]     ...{UKF_3} */
    Wc = Wm;
    Wc[0][0] = Wc[0][0] + (1.0-(_alpha*_alpha)+_beta);
	
}



void UKF_InitMatrices(void)
{
	MatrixInitEmpty(&UKFInit.X_Est ,SS_X_LEN , 1 , InitMatWithZero);
	MatrixInitEmpty(&UKFInit.X_Sigma ,SS_X_LEN , (2*SS_X_LEN + 1) , InitMatWithZero);
	
	MatrixInitEmpty(&UKFInit.Y_Est ,SS_Z_LEN , 1 , InitMatWithZero);
	MatrixInitEmpty(&UKFInit.Y_Sigma ,SS_Z_LEN , (2*SS_X_LEN + 1) , InitMatWithZero);
	
	MatrixInitEmpty(&UKFInit.P ,SS_X_LEN , SS_X_LEN , InitMatWithZero);
	MatrixInitEmpty(&UKFInit.P_Chol ,SS_X_LEN , SS_X_LEN , InitMatWithZero);
	
	MatrixInitEmpty(&UKFInit.DX ,SS_X_LEN , (2*SS_X_LEN + 1) , InitMatWithZero);
	MatrixInitEmpty(&UKFInit.DY ,SS_Z_LEN , (2*SS_X_LEN + 1) , InitMatWithZero);
	
	MatrixInitEmpty(&UKFInit.Py ,SS_Z_LEN , SS_Z_LEN , InitMatWithZero);
	MatrixInitEmpty(&UKFInit.Pxy ,SS_X_LEN , SS_Z_LEN , InitMatWithZero);
	
	MatrixInitEmpty(&UKFInit.Wm ,1 , (2*SS_X_LEN + 1) , InitMatWithZero);
	MatrixInitEmpty(&UKFInit.Wc ,1 , (2*SS_X_LEN + 1) , InitMatWithZero);
	
	MatrixInitEmpty(&UKFInit.Rv ,SS_X_LEN , SS_X_LEN , InitMatWithZero);
	MatrixInitEmpty(&UKFInit.Rn ,SS_Z_LEN , SS_Z_LEN , InitMatWithZero);
	
	MatrixInitEmpty(&UKFInit.Err ,SS_Z_LEN , 1 , InitMatWithZero);
	MatrixInitEmpty(&UKFInit.Gain ,SS_X_LEN , SS_Z_LEN , InitMatWithZero);
}
















