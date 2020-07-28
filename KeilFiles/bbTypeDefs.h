/**
*  ***********************************************************************
*  @file    bbTypeDefs.h
*  @author  B. Mysliwetz (based on WS2014 Master's Project by I. Lienhardt)
*  @version V1.00
*  @brief   State space model and state vector estimation related data
*           structures.
*
*  @date    06/12/2016
            24/05/2020 KF ballplate Team#1 (defined State space parameters for KF)
**************************************************************************
*/
#ifndef __BB_TYPDEFS__
#define __BB_TYPDEFS__


#include "..\basicIO\mctDefs.h" // useful constants, macros, type shorthands 

#include "bbSSParams.h"

//#define PI         ((float) 3.1415926)
//#define CRAD2DEG   ((float) 180.0 / PI)
//#define CDEG2RAD   (PI / (float) 180.0)

/** Struct for state space parameters.
*
* The struct SS_PAR is a container for parameters of a state space model
* of the type:
*
*  x[k+1]   = (A - K^T *B ) * x[k] + B *V *r[k] + L *(y_meas[k] - y_est[k]
*
*  y_est[k] = C' * x[k] 
*/

typedef struct SS_PAR // State space parameters for Observer
{
  const float F[ORDER][ORDER];    //!< Transition Matrix
  const float G[ORDER];           //!< Input Gain Matrix 
  const float C[ORDER];           //!< Measurement Matrix
  const float K[ORDER];           //!< Controller gain vector
  const float V;                  //!< Overall gain
  const float L[ORDER];           //!< Observer/Kalman gain vector
} SS_PAR;


typedef struct SS_PAR_KF // State space parameters for KF( kalman filter )
{
  float F[ORDER][ORDER];     //!< Transition Matrix
  float G[ORDER];           //!< Input Gain Matrix 
  float C[ORDER];           //!< Measurement Matrix
  float K[ORDER];           //!< Controller gain vector
  float V;                  //!< Overall gain
  float L[ORDER];           //!< Kalman gain vector
  float P[ORDER][ORDER];    //!< Estimation error covariance Matrix(measurement update)  
  float P1[ORDER][ORDER];   //!< Estimation error covariance Matrix(extrapolated)
	float Q[ORDER][ORDER];    //!< Process noise covariance Matrix
	float R;                  //!< system noise covariance Matrix
} SS_PAR_KF;



/** Struct for state space variables.
*
* The struct SS_VAR is a container for variables of the balance board model.
*
* Only variables of the state space model, full state feedback and 
* observer/kalman filter should be members of this struct.
*/

typedef struct SS_VAR // State space Variables
{
  UINT16 posMax;         //!< manually measured max image position of ball [pxl]
  UINT16 posMin;         //!< manually measured min image position of ball [pxl]
  
 /** Current [k] state vector.
   *  index 0: position of the ball in [m]
   *  index 1: speed of the ball in [m/s]
   *  index 2: servo arm angle in [rad] (ONLY FOR ORDER == 3)
  */
  float  ssvEst_k[ORDER];  

 /** Predicted [k+1] state vector.
   *  index 0: position of the ball in [m]
   *  index 1: speed of the ball in [m/s]
   *  index 2: servo arm angle in [rad] (ONLY FOR ORDER == 3)
  */  
  float  ssvEst_k1[ORDER];
   
  // ball positions in [pxl]
  short  posMeasPxl;          //!< current measured position [pxl]
  short  posRefPxl;           //!< reference position        [pxl]
  short  posOldPxl;           //!< last measured position    [pxl]
  
  // ball positions in [m]
  float  posMeas;             //!< current measured position [m]
  float  posRef;              //!< reference position        [m]
  float  posEst;              //!< estimated position by observer [m]
  float  speedCur;            //!< current ball speed in     [m/s]
  
  float  alphaPlateRad;           //!< ball plate angle in [rad]
	float  cmdServoAngleRad;        //!< servo angle command in [rad]

  INT16  calServoAngleOffsetPWM;  //!< servo angle horizontal calibration offset [us]  
  // PWM signal duration in [us]
  UINT16 cmdServoAnglePWM;        //!< servo angle command as pulse width in [us]
  
} SS_VAR;

#endif
