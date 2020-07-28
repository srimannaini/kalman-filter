/**
*  ***********************************************************************
*  @file    BallPlate.h
*  @author  B. Mysliwetz (based on WS2014 Master's Project by I. Lienhardt)
*  @version V1.0
*  @brief   Application specific symbolic consts/config params and typedefs.
*
*  @date    06/12/2016
*           24/05/2020 KF ballplate Team#1(defined KF)
**************************************************************************/


#ifndef   BALLPLATE_H
#define   BALLPLATE_H

/// activate debug messages to console
//#define   DEBUG_MSG_ON

/// define one of "_SYS_2ND_ORDER_OBS_", "_SYS_3RD_ORDER_OBS_" or 
//  "_SYS_3RD_ORDER_KF_" to run the specific model

//#define _SYS_2ND_ORDER_OBS_ 
//#define _SYS_3RD_ORDER_OBS_ 
//#define _SYS_3RD_ORDER_KF_
#define _SYS_4TH_ORDER_KF_

//define either KF or Observer Execution
#ifdef _SYS_2ND_ORDER_OBS_
 #define OBSERVER
#elif defined _SYS_3RD_ORDER_OBS_ 
 #define OBSERVER
#elif defined _SYS_3RD_ORDER_KF_ 
 #define KF
#elif defined _SYS_4TH_ORDER_KF_ 
 #define KF
#else
 #error "Illegal value for system ORDER ! Valid values: _SYS_2ND_ORDER_OBS_ or _SYS_3RD_ORDER_OBS_ or _SYS_3RD_ORDER_KF_ or _SYS_4TH_ORDER_KF_"
#endif

#include  "..\basicIO\mctDefs.h" // useful constants, macros, type shorthands 

#include  "bbTypeDefs.h"         // application specific typdefs
#include  "bbSSParams.h"         // state space model parameters
#include  "BallPlateKFlib.h"

// various execution- and test modes
#define   OPMODE_CLOSED_LOOP_CONTROL   0x01       //!< closed loop control mode
//#define OPMODE_TEST_STEP_SEQUENCE    0x02       //!< test step sequence mode
//#define OPMODE_CALIBRATE_XAXIS       0x04       //!< calibrate x-axis mode
//#define OPMODE_CALIBRATE_YAXIS       0x08       //!< calibrate y-axis mode
//#define INVERT_SERVO_X   //!< invert servo turning sense, eg. for HITEC-645 servo 
//#define INVERT_SERVO_Y   //!< invert servo turning sense, eg. for HITEC-645 servo 



// sample time in seconds - here: 50 ms equal to 20 FPS
#define   T                      (float) (1.0/20)      //!< sample period in [s]
#define   T22                    (float) (T*T/2.0)     //!< term used in F & G
#define   CGEARTH                (float)  9.81;        //!< gravity constant in [m/s2] 
#define   CM2PXL                 (float)  1380;        //!< camera & lens scaling factor in [pxl] per [m]

// conversion factor (1 meter == 1385 pxl)  ??? FOR WHICH CAMERA vgl. ZiSo ???
#define   PIXEL_PER_METER        (float) 1385.0        //!< conversion factor meter > pxl

#define   PLATE_XMIN_PXL          80         //!< visible ballplate image X min 
#define   PLATE_XMAX_PXL         680         //!< visible ballplate image X max

#define   PLATE_YMIN_PXL          30         //!< visible ballplate image Y min 
#define   PLATE_YMAX_PXL         570         //!< visible ballplate image Y max

// servo related PWM/Timer constants
#define   CENTER_PULSE_WIDTH     1500        //!< servo zero position pulse width in [us]
#define   MIN_PULSE_WIDTH        1000        //!< min pulse width in [us]
#define   MAX_PULSE_WIDTH        2000        //!< max pulse width in [us]

// servo angle offsets for calibrating ballplate horizontal position
#define   PWM_SERVO_X_OFFSET      -20        //!< in PWM [us] units -50 seems OK
#define   PWM_SERVO_Y_OFFSET        0        //!< in PWM [us] units   0



// PWM pulse repetition period (default 20000 = 20ms OK, 15000 = 15ms OK)
#define     DEFAULT_PULSE_PERIOD   15000     //!< PWM pulse repeat period in [us]
//#define   DEFAULT_PULSE_PERIOD   13000     //!< NOK -> STRONG JITTER W VS-11AMB Modelcraft Servo
//#define   DEFAULT_PULSE_PERIOD   10000     //!< NOK -> STRONG JITTER W VS-11AMB Modelcraft Servo

// mechanical dimensions/limits
// max servo angle in radians ( -50 deg <= servo angle <= 50 deg )
#define   MAX_SERVO_ANGLE_DEG    50          //!< max servo angle in [o]
#define   MAX_SERVO_ANGLE_RAD    DEG2RAD(MAX_SERVO_ANGLE_DEG)  //!< max servo angle in [rad]
#define   SERVO_ARM_LENGTH       0.03        //!< servo arm length in [m]
#define   HALF_BOARD_LENGTH      0.20        //!< plate axis to servo arm attachment point in [m]

// measurement noise & process noises
#ifdef KF
  #define   SIGMA_POS_PXL          (float) 1.0        //!< standard deviation of measurement noise > pxl
  #define   Q_POS_VAR              (float) 5.0        //!< standard deviation of position > [pxl]^2
	#define   Q_DOT_VAR              (float) 400.0      //!< standard deviation of speed > [pxl/s]^2
	#define   Q_THETA_VAR            (float) 5.0        //!< standard deviation of servo angle> degrees
	#define   Q_DELTATHETA_VAR       (float) 0.1        //!< standard deviation of delta  > degrees
#endif

//UINT16 setPWMSignalAngleX(SS_VAR*);
//UINT16 setPWMSignalAngleY(SS_VAR*);



void bbSSControlAndObserver( SS_VAR *X,  SS_VAR *Y,  const SS_PAR *SS ); // compute new estimated state for observer
void bbSSControlAndKF( SS_VAR *xAxis, SS_VAR *yAxis, SS_PAR_KF *SSx, SS_PAR_KF *SSy); // compute new estimated state for KF

/// Function for clearing console screen (works for putty)
#define CLEAR_SCREEN  DB_print_string("\x1B[2J");

/// Function for newline over RS232 interface
#define NEWLINE       DB_putchar(ASC_CR); DB_putchar(ASC_LF);

#endif  /* BALLPLATE_H */

/************************************************************************\
*    END OF MODULE BallPlate.h
\************************************************************************/
