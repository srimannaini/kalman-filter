/**
*  ***********************************************************************
*  @file    Controller.c
*  @author  B. Mysliwetz (based on WS2014 Master's Project by I. Lienhardt) 
*  @version V1.00
*  @brief   Full state feedback controller,observer and Kalman filter for ballplate using
*           2nd/3rd/4th order model. 
*
*  Modified
*  161109   mys   new comp. flags INVERT_SERVO_XY to +/- servo turning sense
*  161221   mys   jerks -> XY.speedCur INIT. CORRECTLY; T, MAX_SERVO_ANGLE_RAD
*  200524   KF ballplate Team#1(defined KF) 
*  @date    21/12/2016
            24/05/2020 KF ballplate Team#1
**************************************************************************
*/

#include "BallPlate.h"           // application specific defs & types,
//#include "bbSSParams.h"        // plus model, observer & controller params
//#include "bbTypeDefs.h"        // ALREADY INCLUDED via BallPlate.h ...


#ifdef OBSERVER
void bbSSControlAndObserver( SS_VAR *xAxis, SS_VAR *yAxis, const SS_PAR *SS )
/**
 * @brief Compute new estimated state and control variable.
 *
 *  The state space model:
 *
 *    x[k+1]   = (F - K`*G) * x[k] + G*V* r[k] + L(y_meas[k] - y_est[k])
 *  with
 *    y_est[k] = C' * x[k]
 *
 * @param[in] xAxis  Pointer to state variables struct for X-Axis
 * @param[in] yAxis  Pointer to state variables struct for Y-Axis
 * @param[in] SS     Pointer to struct with state space model parameters 
 *
 */
{
  float xEstError;
  float yEstError;

  // save current state
#if ORDER == 2
  xAxis->ssvEst_k[0] = xAxis->ssvEst_k1[0];
  yAxis->ssvEst_k[0] = yAxis->ssvEst_k1[0];

  xAxis->ssvEst_k[1] = xAxis->ssvEst_k1[1];
  yAxis->ssvEst_k[1] = yAxis->ssvEst_k1[1];
#elif ORDER > 2
  xAxis->ssvEst_k[2] = xAxis->ssvEst_k1[2];
  yAxis->ssvEst_k[2] = yAxis->ssvEst_k1[2];
#elif ORDER > 3
  #error "Illegal value for system ORDER ! Valid (as yet):  2 or 3"
#endif


/*************************************************************************\
  Full state feedback controller
	ThetaServoCmd = posRef * V - K' * xEst
\*************************************************************************/
 
  xAxis->cmdServoAngleRad = xAxis->posRef * SS->V
                     - SS->K[0] * xAxis->posMeas
                     - SS->K[1] * xAxis->ssvEst_k[1];

  yAxis->cmdServoAngleRad = yAxis->posRef * SS->V
                     - SS->K[0] * yAxis->posMeas
                     - SS->K[1] * yAxis->ssvEst_k[1];
										 
#if ORDER == 3
  xAxis->cmdServoAngleRad = xAxis->cmdServoAngleRad - SS->K[2] * xAxis->ssvEst_k[2];
  yAxis->cmdServoAngleRad = yAxis->cmdServoAngleRad - SS->K[2] * yAxis->ssvEst_k[2];
#endif

#ifdef INVERT_SERVO_X
  xAxis->cmdServoAngleRad = - xAxis->cmdServoAngleRad;
#endif
#ifdef INVERT_SERVO_Y
  yAxis->cmdServoAngleRad = - yAxis->cmdServoAngleRad;
#endif


  // compute ballplate angle in [rad]  !! SIMPLIFIED/LINEARIZED EQN !!
  xAxis->alphaPlateRad = (xAxis->cmdServoAngleRad * SERVO_ARM_LENGTH ) / HALF_BOARD_LENGTH ;
  yAxis->alphaPlateRad = (yAxis->cmdServoAngleRad * SERVO_ARM_LENGTH ) / HALF_BOARD_LENGTH ;

  // Limit servo angle  |cmdServoAngleRad| <= MAX_SERVO_ANGLE_RAD
  if ( xAxis->cmdServoAngleRad > MAX_SERVO_ANGLE_RAD )
  { 
    xAxis->cmdServoAngleRad =    MAX_SERVO_ANGLE_RAD;
  }

  if( xAxis->cmdServoAngleRad < -MAX_SERVO_ANGLE_RAD )
  {
    xAxis->cmdServoAngleRad =   -MAX_SERVO_ANGLE_RAD;
  }

  if ( yAxis->cmdServoAngleRad > MAX_SERVO_ANGLE_RAD ) 
  {
    yAxis->cmdServoAngleRad =    MAX_SERVO_ANGLE_RAD;
  }
	
  if( yAxis->cmdServoAngleRad < -MAX_SERVO_ANGLE_RAD ) 
  {
    yAxis->cmdServoAngleRad =   -MAX_SERVO_ANGLE_RAD;
  }

/*************************************************************************\
  Observer Calculations
	x[k+1] = A*x[k] + B*Theta + L*(y_meas[k] - y_est[k])
\*************************************************************************/

  xEstError = xAxis->posMeas - xAxis->ssvEst_k[0];
  yEstError = yAxis->posMeas - yAxis->ssvEst_k[0];

#if ORDER == 2
  xAxis->ssvEst_k1[0]  = SS->F[0][0] * xAxis->ssvEst_k[0]
                       + SS->F[0][1] * xAxis->ssvEst_k[1]
                       + SS->G[0]    * xAxis->cmdServoAngleRad
                       + SS->L[0]    * xEstError;

  xAxis->ssvEst_k1[1]  = SS->F[1][0] * xAxis->ssvEst_k[0]
                       + SS->F[1][1] * xAxis->ssvEst_k[1]
                       + SS->G[1]    * xAxis->cmdServoAngleRad
                       + SS->L[1]    * xEstError;

  yAxis->ssvEst_k1[0]  = SS->F[0][0] * yAxis->ssvEst_k[0]
                       + SS->F[0][1] * yAxis->ssvEst_k[1]
                       + SS->G[0]    * yAxis->cmdServoAngleRad
                       + SS->L[0]    * yEstError;

  yAxis->ssvEst_k1[1]  = SS->F[1][0] * yAxis->ssvEst_k[0]
                       + SS->F[1][1] * yAxis->ssvEst_k[1]
                       + SS->G[1]    * yAxis->cmdServoAngleRad
                       + SS->L[1]    * yEstError;

#elif ORDER == 3
  xAxis->ssvEst_k1[0] += SS->F[0][2] * xAxis->ssvEst_k[2];
  xAxis->ssvEst_k1[1] += SS->F[1][2] * xAxis->ssvEst_k[2];

  xAxis->ssvEst_k1[2]  = SS->F[2][0] * xAxis->ssvEst_k[0]
                       + SS->F[2][1] * xAxis->ssvEst_k[1]
                       + SS->F[2][2] * xAxis->ssvEst_k[2]
                       + SS->G[2]    * xAxis->cmdServoAngleRad
                       + SS->L[2]    * xEstError;

  yAxis->ssvEst_k1[0] += SS->F[0][2] * yAxis->ssvEst_k[2];
  yAxis->ssvEst_k1[1] += SS->F[1][2] * yAxis->ssvEst_k[2];

  yAxis->ssvEst_k1[2]  = SS->F[2][0] * yAxis->ssvEst_k[0]
                       + SS->F[2][1] * yAxis->ssvEst_k[1]
                       + SS->F[2][2] * yAxis->ssvEst_k[2]
                       + SS->G[2]    * yAxis->cmdServoAngleRad
                       + SS->L[2]    * yEstError;
#elif ORDER ==4
  xAxis->ssvEst_k1[0] += SS1->F[0][3] * xAxis->ssvEst_k[3];
  xAxis->ssvEst_k1[1] += SS1->F[1][3] * xAxis->ssvEst_k[3];
  xAxis->ssvEst_k1[2] += SS1->F[2][3] * xAxis->ssvEst_k[3];
	
	xAxis->ssvEst_k1[3]  = SS1->F[3][0] * xAxis->ssvEst_k[0]
                       + SS1->F[3][1] * xAxis->ssvEst_k[1]
                       + SS1->F[3][2] * xAxis->ssvEst_k[2]
											 + SS1->F[3][3] * xAxis->ssvEst_k[3]
                       + SS1->G[3]    * xAxis->cmdServoAngleRad
                       + SS1->L[3]    * xEstError;

  yAxis->ssvEst_k1[0] += SS1->F[0][3] * yAxis->ssvEst_k[3];
  yAxis->ssvEst_k1[1] += SS1->F[1][3] * yAxis->ssvEst_k[3];
  yAxis->ssvEst_k1[2] += SS1->F[2][3] * yAxis->ssvEst_k[3];

  yAxis->ssvEst_k1[3]  = SS1->F[3][0] * yAxis->ssvEst_k[0]
                       + SS1->F[3][1] * yAxis->ssvEst_k[1]
                       + SS1->F[3][2] * yAxis->ssvEst_k[2]
											 + SS1->F[3][3] * yAxis->ssvEst_k[3]
                       + SS1->G[3]    * yAxis->cmdServoAngleRad
                       + SS1->L[3]    * yEstError;
		
#else
	
  #error "Illegal value for system ORDER ! Valid values: 2 or 3"

#endif

} // END of bbSSControlAndObserver( )

  

#elif defined KF
void bbSSControlAndKF( SS_VAR *xAxis, SS_VAR *yAxis, SS_PAR_KF *SSx, SS_PAR_KF *SSy )
/**
 * @brief Compute new estimated state and control variable.
 *
 *  The state space model:
 *
 *    x[k+1]   = (F - K`*G) * x[k] + G*V* r[k] + L(y_meas[k] - y_est[k])
 *  with
 *    y_est[k] = C' * x[k]
 *
 * @param[in] xAxis  Pointer to state variables struct for X-Axis
 * @param[in] yAxis  Pointer to state variables struct for Y-Axis
 * @param[in] SS_VAR State space Variables
 * @param[in] SSx    Pointer to struct with state space model parameters in x-axis
 * @param[in] SSy    Pointer to struct with state space model parameters in y-axis
 */
{
	float FParams[3];
	float taus = 0.3;   //time constant
  float as = 1/taus;
	//float xEstError;
  //float yEstError;
	float zk = DEG2RAD(xAxis->ssvEst_k1[2]);  // estimated servo angle[rad]
  float zk1= DEG2RAD(xAxis->ssvEst_k1[3]);  // estimated delta servo offset[rad]
	float Tkga= -6.3795;  //T*kga

	FParams[0]= (float) Tkga;
	FParams[1]= (float) zk;
  FParams[2]= (float) zk1;
  // save current state
#if ORDER == 2
  xAxis->ssvEst_k[0] = xAxis->ssvEst_k1[0];
  yAxis->ssvEst_k[0] = yAxis->ssvEst_k1[0];

  xAxis->ssvEst_k[1] = xAxis->ssvEst_k1[1];
  yAxis->ssvEst_k[1] = yAxis->ssvEst_k1[1];
#elif ORDER > 2
  xAxis->ssvEst_k[2] = xAxis->ssvEst_k1[2];
  yAxis->ssvEst_k[2] = yAxis->ssvEst_k1[2];
#elif ORDER > 3
	xAxis->ssvEst_k[3] = xAxis->ssvEst_k1[3];
  yAxis->ssvEst_k[3] = yAxis->ssvEst_k1[3];
#elif ORDER > 4
  #error "Illegal value for system ORDER ! Valid (as yet):  2 or 3 or 4"
#endif


/*************************************************************************\
  Full state feedback controller
	ThetaServoCmd = posRef * V - K' * xEst
\*************************************************************************/
 
  xAxis->cmdServoAngleRad = xAxis->posRef * SSx->V
                     - SSx->K[0] * xAxis->posMeas
                     - SSx->K[1] * xAxis->ssvEst_k[1];

  yAxis->cmdServoAngleRad = yAxis->posRef * SSy->V
                     - SSy->K[0] * yAxis->posMeas
                     - SSy->K[1] * yAxis->ssvEst_k[1];
										 
#if ORDER == 3
  xAxis->cmdServoAngleRad = xAxis->cmdServoAngleRad - SS->K[2] * xAxis->ssvEst_k[2];
  yAxis->cmdServoAngleRad = yAxis->cmdServoAngleRad - SS->K[2] * yAxis->ssvEst_k[2];
#endif
#if ORDER == 4
  xAxis->cmdServoAngleRad = xAxis->cmdServoAngleRad - SSx->K[2] * xAxis->ssvEst_k[2];
  yAxis->cmdServoAngleRad = yAxis->cmdServoAngleRad - SSx->K[2] * yAxis->ssvEst_k[2];
#endif

#ifdef INVERT_SERVO_X
  xAxis->cmdServoAngleRad = - xAxis->cmdServoAngleRad;
#endif
#ifdef INVERT_SERVO_Y
  yAxis->cmdServoAngleRad = - yAxis->cmdServoAngleRad;
#endif


  // compute ballplate angle in [rad]  !! SIMPLIFIED/LINEARIZED EQN !!
  xAxis->alphaPlateRad = (xAxis->cmdServoAngleRad * SERVO_ARM_LENGTH ) / HALF_BOARD_LENGTH ;
  yAxis->alphaPlateRad = (yAxis->cmdServoAngleRad * SERVO_ARM_LENGTH ) / HALF_BOARD_LENGTH ;

  // Limit servo angle  |cmdServoAngleRad| <= MAX_SERVO_ANGLE_RAD
  if ( xAxis->cmdServoAngleRad > MAX_SERVO_ANGLE_RAD )
  { 
    xAxis->cmdServoAngleRad =    MAX_SERVO_ANGLE_RAD;
  }

  if( xAxis->cmdServoAngleRad < -MAX_SERVO_ANGLE_RAD )
  {
    xAxis->cmdServoAngleRad =   -MAX_SERVO_ANGLE_RAD;
  }

  if ( yAxis->cmdServoAngleRad > MAX_SERVO_ANGLE_RAD ) 
  {
    yAxis->cmdServoAngleRad =    MAX_SERVO_ANGLE_RAD;
  }
	
  if( yAxis->cmdServoAngleRad < -MAX_SERVO_ANGLE_RAD ) 
  {
    yAxis->cmdServoAngleRad =   -MAX_SERVO_ANGLE_RAD;
  }

/*************************************************************************\
  Kalman Filter Calculations
\*************************************************************************/
 
	 		
//for X-axis		
		KFcomputeFMat(SSx->F,FParams);  //computes F matrix
		KFcomputeG(SSx->G,FParams,as);  //computes G vector
	//prediction
		KF_PTUPD( SSx->P,SSx->F,SSx->P1,SSx->Q );                                         //extrapolated estim. error covarience matrix
	  KFpredictState(xAxis->ssvEst_k,xAxis->ssvEst_k1,SSx->F,SSx->G,xAxis->posMeas );  // state vector in predicted
	//measurement update
	  //xEstError = xAxis->posMeas - xAxis->ssvEst_k[0];
    //yEstError = yAxis->posMeas - yAxis->ssvEst_k[0];
		
	 KFcomputeKMat( SSx->L,SSx->P,SSx->R );  // KF gain vector
   KF_PMUPD( SSx->P1,SSx->L,SSx->P );      // measurement estim. error covarience matrix
   KF_xMUPD(xAxis->ssvEst_k1,xAxis->ssvEst_k,SSx->L,xAxis->posMeas ); // state vector updated

//for Y-axis
		KFcomputeFMat(SSy->F,FParams);
		KFcomputeG(SSy->G,FParams,as);
	//prediction
		KF_PTUPD( SSy->P,SSy->F,SSy->P1,SSy->Q );
	  KFpredictState(yAxis->ssvEst_k,yAxis->ssvEst_k1,SSy->F,SSy->G,yAxis->posMeas );
	//measurement update
	  //xEstError = xAxis->posMeas - xAxis->ssvEst_k[0];
    //yEstError = yAxis->posMeas - yAxis->ssvEst_k[0];
		
	 KFcomputeKMat( SSy->L,SSy->P,SSy->R );
   KF_PMUPD( SSy->P1,SSy->L,SSy->P );
   KF_xMUPD(yAxis->ssvEst_k1,yAxis->ssvEst_k,SSy->L,yAxis->posMeas );
#else

  #error "Illegal value for system ORDER ! Valid values: 2 or 3 or 4"

#endif

} // END of bbSSControlAndKF( )

/************************************************************************\
*    END OF MODULE Controller.c
\************************************************************************/
