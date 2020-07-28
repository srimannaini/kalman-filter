/**
 *  ***********************************************************************
 * @file    BallPlateKFlib.c
 * @author  B. Mysliwetz
 * @version V1.10
 * @brief   C function templates for XXX model simulation &Kalman Filter
 *          estimation, usable via MATLAB Shared Library Calling (CALLLIB).
 *
 *IMPORT C-functions from 'BallPlateKFlib.mexw64' into MATLAB program via :
 *
 *          if not(libisloaded('BallPlateKFlib'))
 *           %addpath(fullfile(matlabroot,'extern','examples','shrlib'));
 *            addpath('C:\Users\srima\OneDrive\Desktop\3rd sem\matlab\test1');
 *            loadlibrary('BallPlateKFlib');
 *          end
 *          libfunctions BallPlateKFlib -full    % print lib function properties
 *
 *
 *Changes:
 *09.10.2013 mys          Initial version
 *14.12.2016 mys          Adapted for BallPlate
 *12.12.2019 mys          Adapted for R2019
 *24.05.2020 KF ballplate Team#1 Completed the blank C functions
 *                    
 * @date    24/05/2020
 **************************************************************************
 */
#define MATLAB_USE
#include <math.h>

# ifdef MATLAB_USE
//#ifndef  STM32_VERSION
#define EXPORT_FCNS
#include "shrhelp.h"
#include <mex.h >  /*only needed because of mexFunction below and mexPrintf */

# endif
#include "BallPlateKFlib.h"


/**
 *  ***********************************************************************
 * @brief  Helper functions used in this C Code! 
 *         Computes Matrix*Vector for four dimensions
 * @param  Mat:  4x4 Matrix 
 * @param  Vec:  4x1 or 1x4 Vector respectively
 * @param  res:  vector to store result 
  
 **************************************************************************
 */
void MATxVEC(float mat[][NDIM], float vec[NDIM], float res[NDIM])
{
  int i, k;
 	// NDIM = 4;
  for (i = 0; i < NDIM; i++)
  {
    res[i] = 0;
    for (k = 0; k < NDIM; k++)
      res[i] += mat[i][k] *vec[k];
  }
}
/**
 *  ***********************************************************************
 * @brief  Helper functions used in this C Code! 
 *         Computes Vector*Matrix for four dimensions 
 * @param  Vec:  1x4 Vector
 * @param  Mat:  4x4 Matrix 
 * @param  res:  vector to store result
 
 **************************************************************************
 */
void VECxMAT(float vec[NDIM], float mat[][NDIM], float res[NDIM])
{
  int i, k;
 	//NDIM = 4;
  for (i = 0; i < NDIM; i++)
  {
    res[i] = 0;
    for (k = 0; k < NDIM; k++)
      res[i] += mat[k][i] *vec[k];
  }
}

/**
 *  ***********************************************************************
 * @brief  Helper functions used in this C Code! 
 *         Computes vector (only used for easy computing)
 * @param  L:   Matrix or vector (order should satisfy matrix rules)
 * @param  k:   Matrix or vector
 * @param  m:   scalar input
 * @nk: stores result  
 **************************************************************************
 */

void compute_vec(float *L, float *k, float m, float *nk)
//NDIM = 4;
{
  for (int i = 0; i < NDIM; i++)
  {
    nk[i] = L[i] + k[i]*m;
  }
}
/**
 *  ***********************************************************************
 * @brief  Used just in C Code! 
 *         Computes Matrix*Matrix for four dimensions
 * @param  mat1:  1st 4x4 Matrix 
 * @param  mat2:  2nd 4x4 Matrix
 * @param  res :  4x4 Matrix to store result
 * @retval none 
 **************************************************************************
 */

void MATxMAT(float mat1[][NDIM], float mat2[][NDIM], float res[][NDIM])
{
  int i, j, k;
 	//NDIM = 4;
  for (i = 0; i < NDIM; i++)
  {
    for (j = 0; j < NDIM; j++)
    {
      res[i][j] = 0;
      for (k = 0; k < NDIM; k++)
        res[i][j] += mat1[i][k] *mat2[k][j];
    }
  }
}

void TransMAT(float mat[][NDIM], float res[][NDIM])
{
  int i, j;
  for (i = 0; i < NDIM; i++)
    for (j = 0; j < NDIM; j++)
      res[i][j] = mat[j][i];
}


/**
 *  ***********************************************************************
 * @brief  KFcomputeFMat computes transition matrix of time discrete system 
 *       Parameter T scaled in [s], theta4 scaled in [rad]
 * FMat =[1 T 0 0;0 1 kga*cos(theta4)*T kga*cos(theta4)*T; 0 0 1-as*T 0;0 0 0 1];
 * @param FMat : 4x4 transition matrix / 4th order ballplate system model 
 * @param  G   : 4x1 Control input gains of time discrete system vector 
 * @param  FParams : 3x1 parameter vector[ T*Kga, zk,zk1 ]
 * @retval none
 **************************************************************************
 */

EXPORTED_FUNCTION void KFcomputeFMat(float FMat[][NDIM], float FMatTrans[][NDIM], float FParams[])
{
 	// FParams=[T*kga;zk(k);zk1(k)];
                //kga=servo angle to acceleration constant [pxl/s2]
  float Tkga = FParams[0];	//Tkga= T*kga      
    
  float TkgacosTheta = (float)(Tkga* cos(FParams[1] + FParams[2]));   //declared this parameters in matlab
	 
  FMatTrans[1][2] = TkgacosTheta;
  FMatTrans[1][3] = TkgacosTheta;
  TransMAT(FMatTrans, FMat);
}	// end function KFcomputeFMat()

/**
 *  ***********************************************************************
 * @brief  Computes(G) Control input gains of time discrete system 
 *             
 * @param  G   : Control input gains of time discrete system 
 * @param  FParams : 3x1 parameter vector[ T*Kga, zk,zk1 ]
 * @param  T   : sample period scaled in [s](T22=T*T/2)
 * @retval none
 **************************************************************************
 */

EXPORTED_FUNCTION void KFcomputeG(float G[], float FParams[], float as, float T, float T22)
{
	// FParams=[T*kga;zk(k);zk1(k)];
  float kga = FParams[0] / T;     // kga=servo angle to acceleration constant [pxl/s2]
  float theta4 = FParams[1] + FParams[2];   //theta4=servo angle scaled in [rad]
  G[0] = 0.0;
  G[1] = kga* cos(theta4) *T22 * as;  
  G[2] = as *T - as *as * T22;
  G[3] = 0.0;
 	//end function 
}	// gn4k=[0;kga*cos(theta4)*T22*as;as*T-as*as*T22;0];

/**
 *  ***********************************************************************
 * @brief  Computes time extrapolated state xk1 = FMat*xk + G*uk.
 * @param  xk   : 4x1 state vector in at[k] -> out predicted at[k+1]
 * @param  FMat : 4x4 system transition matrix 
 * @param  G    : 4x1 Control input gains of time discrete system vector
 * @param  uk   : 1x1 control input at[k]
 * @retval none
 **************************************************************************
 */
EXPORTED_FUNCTION void KFpredictState(float xk1[], float xk[], float F[][NDIM], float g[], float uk)
{
  float Fxk[NDIM];
  MATxVEC(F, xk, Fxk);
  compute_vec(Fxk, g, uk, xk1);
 	// xState4KFk1=F4*xState4KFk+g4*uk;
}	// end function KFpredictState()

/**
 *  ***********************************************************************
 * @brief  Computes time updated covariance matrix P1 = FMat*P*FMat'+QMat.
 * @param  P    : 4x4  estim. error cov matrix in at sample[k] 
 * @param  FMat : 4x4 system transition matrix  
 * @param  QMat : 4x1 process noise cov matrix diagonal elements
 * @param  P1   : 4x4 estimated error covarience matrix,predicted
 * @retval none
 **************************************************************************
 */
EXPORTED_FUNCTION void KF_PTUPD(float P1[][NDIM], float F[][NDIM], float P[][NDIM], float Q[])
{
  float FTrans[NDIM][NDIM];   //Transpose of transition matrix
  float F_P[NDIM][NDIM];
  float F_P_FTrans[NDIM][NDIM];
  TransMAT(F, FTrans);
  MATxMAT(F, P, F_P);
  MATxMAT(F_P, FTrans, F_P_FTrans);
 	//P1 = F*P*F'; 
  P1[0][0] = F_P_FTrans[0][0] + Q[0];
  P1[1][1] = F_P_FTrans[1][1] + Q[1];
  P1[2][2] = F_P_FTrans[2][2] + Q[2];
  P1[3][3] = F_P_FTrans[3][3] + Q[3];

  P1[0][1] = F_P_FTrans[0][1];
  P1[0][2] = F_P_FTrans[0][2];
  P1[0][3] = F_P_FTrans[0][3];

  P1[1][0] = F_P_FTrans[1][0];
  P1[1][2] = F_P_FTrans[1][2];
  P1[1][3] = F_P_FTrans[1][3];

  P1[2][1] = F_P_FTrans[2][1];
  P1[2][0] = F_P_FTrans[2][0];
  P1[2][3] = F_P_FTrans[2][3];

  P1[3][0] = F_P_FTrans[3][0];
  P1[3][1] = F_P_FTrans[3][1];
  P1[3][2] = F_P_FTrans[3][2];

 	// P4k1=F4*P4k*F4'+Q4k;
}	// end function KF_PTUPD()

// For results crosschecking with the above optimized version KF_PTUPD()
EXPORTED_FUNCTION void KF_PTUPDref(float PMat[][NDIM], float Phi[][NDIM], float Q[])
{
 	// ...
}	// end function KF_PTUPDref()

/**
 *  ***********************************************************************
 * @brief  Computes KF gain matrix     KKF_k = PMat_k1*CMat' *CPCRinv.
 * @param  KMat  : out 4x1 KF gain matrix
 * @param  PMat  : in  4x4 estim. error cov matrix, extrapolated for[k+1] 
 * @param  Rdiag  : measurement noise cov matrix IN OUR IMPLEMENTATION ONLY SCALAR
 * @retval none
 **************************************************************************
 */

EXPORTED_FUNCTION void KFcomputeKMat(float KMat[], float PMat[][NDIM], float Rdiag)
{
  float PC[NDIM];
  float cpcrinv;

  for (int i = 0; i < NDIM; i++)
  {
    PC[i] = PMat[i][0];
  }

  cpcrinv = 1 / (PMat[0][0] + Rdiag);
  for (int k = 0; k < NDIM; k++)
  {
    KMat[k] = PC[k] *cpcrinv;
  }

 	// kKF4=P4k1*c4/(c4'*P4k1*c4+rVar4);
}	// end function KFcomputeKMat()

/**
 *  ***********************************************************************
 * @brief  Computes inverse of C*P_k1*C' + R).
 * @param  CPCRinv: out 1x1 matrix, inverse of (C*P_k1*C' + R) 
 * @param  PMat   : in  4x4 estim. error cov matrix, extrapolated for[k+1] 
 * @param  Rdiag   :in measurement noise cov matrix(scalar)
 * @retval none
 **************************************************************************
 */

EXPORTED_FUNCTION void KFcomputeCPCRinv(float CPCRinv[][MDIM], float PMat[][NDIM], float Rdiag[])
{
  for (int k = 0; k < 4; k++)
  {
    CPCRinv[k][0] = 1 / (PMat[k][k] + Rdiag[k]);
  }
}	// end function KFcomputeCPCRinv()

/**
 *  ***********************************************************************
 * @brief  Computes KF gain matrix     KKF_k = PMat_k1*CMat' *CPCRinv.
 *         needs intermediate result from KFcomputeCPCRinv()
 * @param  KMat   : out 4x1 KF gain matrix
 * @param  PMat   : in  4x4 estim. error cov matrix, extrapolated for[k+1] 
 * @param  CPCRinv: 1x1 input matrix, inverse of (C*P*C' + R) 
 * @retval none
 **************************************************************************
 */

EXPORTED_FUNCTION void KFcomputeKMat0(float KMat[][MDIM], float PMat[][NDIM], float CPCRinv[][MDIM])
{
  for (int k = 0; k < NDIM; k++)
  {
    KMat[k][0] = PMat[k][k] *CPCRinv[k][0];
  }

 	// kKF4=P4k1*c4 * CPCRinv;
}	// end function KFcomputeKMat0()

/**
 *  ***********************************************************************
 * @brief  Computes PMat measurement update P_k = P_k1 - KMat*CMat*P_k1.
 * @param  Pkout  : out 4x4 estim. error cov matrix, measured
 * @param  KMat   : in  4x1 KF gain matrix
 * @param  Pk1in  : in  4x4 estim. error cov matrix, extrapolated for[k+1] 
 * @retval none
 **************************************************************************
 */

EXPORTED_FUNCTION void KF_PMUPD(float Pkout[][NDIM], float KMat[], float Pk1in[][NDIM])
{
  float eye[NDIM][NDIM] = {
		{ 1, 0, 0, 0 }, /* initializers for row indexed by 0 */
    { 0, 1, 0, 0 }, /* initializers for row indexed by 1 */
    { 0, 0, 1, 0 }, /* initializers for row indexed by 2 */
    { 0, 0, 0, 1 }
  };

  for (int i = 0; i < NDIM; i++)
  {
    eye[i][0] = eye[i][0] - KMat[i]; //(I4-knKF4*c4')
  }

  MATxMAT(eye, Pk1in, Pkout);
 	// P4k=(I4-knKF4*c4')*P4nk1;
}	// end function KF_PMUPD()

/**
 *  ***********************************************************************
 * @brief  Computes xKF measurement update x_k = x_k1 + KMat*(ymeas-ypred).
 * @param  xKF     : out4x1 state vector; out updated 
 * @param  xKF1    : in 4x1 state vector; in predicted 
 * @param  KMat    : in 4x1 KF gain matrix
 * @param  yMeas   : in 1x1 Measurement vector 
 * @retval none
 **************************************************************************
 */

EXPORTED_FUNCTION void KF_xMUPD(float xKF[NDIM], float xKF1[NDIM], float KMat[], float yMeas)
{
  float dy;
  dy = yMeas - xKF1[0];   // yk-c4'*xState4KFk1
  compute_vec(xKF1, KMat, dy, xKF);
 	// xState4KFk=xState4KFk1+kKF4*(yk-c4'*xState4KFk1);
}	// end function KF_xMUPD()

# ifdef MATLAB_USE

EXPORTED_FUNCTION void print44Mat(float mat44[][NDIM])
{
  int i, j;
  mexPrintf("\n");
  for (i = 0; i < NDIM; ++i)
  {
    for (j = 0; j < NDIM; ++j)
    {
      mexPrintf("%10.6f", mat44[i][j]);
    }

    mexPrintf("\n");
  }

  mexPrintf("\n");
}

EXPORTED_FUNCTION void myPrintMat(char matName[], int nRows, int nCols, float mat[])
{
  int i, j, k;
  mexPrintf("%s =\n", matName);
  for (i = 0; i < nRows; ++i)
  {
    for (j = 0; j < nCols; ++j)
    {
      k = i *nCols + j;
     	//k = j*nRows + i;
      mexPrintf("%10.6f", mat[k]);
    }

    mexPrintf("\n");
  }

  mexPrintf("\n");
}

/*this function exists so that mex may be used to compile the library
it is not otherwise needed*/

void mexFunction(int nlhs, mxArray *plhs[],
  int nrhs, const mxArray *prhs[]) {}

# endif

/**
 * ************************************************************************
 *End of module BallPlateKFlib_Template.c
 **************************************************************************
 */