/**
*  ***********************************************************************
*  @file    BallPlateKFlib.h
*  @author  B. Mysliwetz
*  @version V1.10
*  @brief   C function prototypes and defines for BallPlate simulation & 
*           Kalman Filter based estimation, usable via MATLAB 
*           Shared Library Calling (CALLLIB).
*
*  IMPORT C-functions from 'BallPlateKFlib.mexw64 into MATLAB program via :
*
*  if not(libisloaded('BallPlateKFlib'))
*   %addpath( fullfile(matlabroot,'extern','examples','shrlib') );
*    addpath( 'C:\myMatLab\BallPlatSim\' );
*    loadlibrary('BallPlateKFlib');
*  end
*  libfunctions BallPlateKFlib -full    % print lib function properties
*
*
*  USAGE of C-functions in MATLAB program via 'calllib( )' e.g.:
*
*  FParams = single( [ LLTest,  ThetaElKF_k, OmegaElKF_k ] );
*  FMat32 = ( calllib( 'BallPlateKFlib', 'KFcomputeFMat', FMat32', FParams ) )';
*  calllib( 'BallPlateKFlib', 'print44Mat', FMat32' );
*
*
*  Changes:
*  09.10.2013 mys     Initial version
*  14.12.2016 mys     Adapted for BallPlate                     
*  23.05.2020 mys     Adapted for R2019
*                      
*  @date    23/05/2020
**************************************************************************
*/

#ifndef BALLPLATE_KFLIB_H            // avoid multiple inclusion
#define BALLPLATE_KFLIB_H

#include "shrhelp.h"

// system order
#define  NDIM     4

// number of measurements
#define  MDIM     1


// time update related functions

EXPORTED_FUNCTION void KFcomputeFMat(  float FMat[][NDIM],float FMatTrans[][NDIM], float FParams[] );

EXPORTED_FUNCTION void KFcomputeG(  float G[NDIM], float FParams[], float as, float T , float T22);

EXPORTED_FUNCTION void KFpredictState( float xk1[],float xk[], float F[][NDIM], float g[], float uk );

EXPORTED_FUNCTION void KF_PTUPD( float P1[][NDIM],float F[][NDIM],float P[][NDIM], float Q[] );



// measurement update related functions

EXPORTED_FUNCTION void KFcomputeKMat( float KMat[],float PMat[][NDIM], float Rdiag );

EXPORTED_FUNCTION void KF_PMUPD( float Pkout[][NDIM],float KMat[], float Pk1in[][NDIM] );

EXPORTED_FUNCTION void KF_xMUPD( float xKF[NDIM],float xKF1[NDIM], float KMat[], float yMeas );



// for test & debugging

EXPORTED_FUNCTION void KFcomputeCPCRinv( float CPCRinv[][MDIM], float PMat[][NDIM], float Rdiag[] );

EXPORTED_FUNCTION void KFcomputeKMat0( float KMat[][MDIM], float PMat[][NDIM], float CPCRinv[][MDIM] );


EXPORTED_FUNCTION void myPrintMat( char matName[], int nRows, int nCols, float mat[]);

EXPORTED_FUNCTION void print44Mat( float mat44[][NDIM] );

#endif  // #ifndef BALLPLATE_KFLIB_H

/**
* ************************************************************************
*  End of module BallPlateKFlib.h
**************************************************************************
*/