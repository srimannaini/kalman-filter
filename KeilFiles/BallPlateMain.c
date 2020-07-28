/**
*  ***********************************************************************
*  @file    BallPlateMain.c
*  @author  B. Mysliwetz (based on WS2014 Master's Project by I. Lienhardt) 
*  @brief   Main module of BallPlate Controller Project.
*
*  @version V0.10  2nd/3rd order full state feedback & observer and 4th order kalman filter
*
* Peripherals & operating settings used:
*
*  Target   MCBSTM32           MCBSTM32C       STM32F4_DISCOVERY
*  MCU      STM32F103RB        STM32F107VC     STM32F407VG
*
*  Console-SIO for user-/debug-/status-messages to terminal:
*  Console  USART1             USART2          USART6
*           115200Bd 8N1       115200Bd 8N1
*           TxD - PA9          TxD - PD5
*           RxD - PA10         RxD - PD6
*
*  Aux-SIO for control related communication w image-processing PC:
*  AuxSIO   USART2             USART3          ???
*           115200Bd 8N1       115200Bd 8N1
*           TxD - PA2          TxD - PB10
*           RxD - PA3          RxD - PB11
*
*  Timers to generate PWM signals for servo actuator control:
*  X-Servo  TIM3_CH1 - PA6     TIM4_CH3 - PB8
*  Y-Servo  TIM3_CH2 - PA7     TIM4_CH4 - PB9
*
*  Timer-Settings: 
*           - PWM-mode         1 
*           - Pulse period     10ms -> 15ms mys 190502 !
*           - Pulse width      1ms ... 2ms
*           - Pre-scaler:      72d    -> 1 MHz count freq
*           - Auto-reload:     15000d -> 15 ms 
*           - Output           active low, edge-aligned
*           - Up-counting
*
*  LEDs for indicating/measuring activity/timing of certain functions:
*  LED1     - ...
*  LED2     - ... see auxSIOcomm.h
*  LED3     - ...
*  LED4     - ...
*
*  Modifications
*  160901   mys   cmd computation corrected (UINT16) to (INT16)
*                 use ZiSo controller gains: Kp = -1.0, Kd = -1.5
*  160905   mys   use ZiSo controller gains: Kp = -1.5, Kd = -1.5
*                 use 10ms PulsPeriod, -50us/-50us horiz. offsets 
*  160909   mys   use OBS3 & ZiSo controller gains: Kp = -1.5, Kd = -1.5
*  160909c  mys   use KF3  & ZiSo controller gains: Kp = -1.5, Kd = -1.5
*  160912a  mys   use OBS2 & ZiSo con/obs gains:    Kp = -1.5, Kd = -1.5
*                                                   h1 =  1.0  h2 =  5.6
*                                                 -50us/ 0us horiz. offsets
*  161109   mys   inverted X servo turning sense for Graupner DES 806 in 
*                 Controller.c
*  161111   mys   streamlined var & const names, plate balanced in x
*  161111   mys   use      controller gains: Kp = -1.5, Kd = -1.0
*                 for HITEC645-Servo on X-axis     Kd = -1.5 oszillates !
*  161206   mys   variable names streamlining, cmdServoAngleCalOffsetPWM
*  161207   mys   _aux_putch() & _aux_getch() via AUX_UART, removed L's legacy
*                 sh.. functions DB_XXX()
*  161208   mys   command interpreter, set servo angle horiz. cal offsets
*  161221   mys   T, MAX_SERVO_ANGLE_RAD; init ALL struct vars 
*                 init XY.speedCur !!CORRECTLY!! to avoid servo jerks 
*  170126   mys   Keep control loop static at init condition for first 3 sycles 
*                 if ( kSampleIndex < 3 )    // keep everything static
*  170206   mys   a if ( kSampleIndex < 10 ) // keep everything static
*                 b xDotEstk1 = 2000 after/in 1st cycle -> sh.. in est-formula
*  170227   mys   c if ( kSampleIndex < 5 )  // keep everything static
*  170312   mys   Stack size set to 0x300 in STM32F10x.s to avoid stack overflow
*  170313   mys   Added HardFault_Handler() & test illegal adress via *err_ptr
*  171108   mys   Chuong's controller gains: Kp = -2.2, Kd = -0.7  almost instable !!
*  171115   mys   Rescale cmdServoAngleRad to [o] in sendData() for serial output !!
*                 Rescale alphaPlatRad to [o] in sendData() Kp=-1.5 Kd=-1.1
*  171115   mys   Removed tabs, updated comments
*  190111   mys   compute X->cmdServoAnglePWM = CENTER_PULSE_WIDTH + X->calServoAngleOffsetPWM - 
*   (INT16) (X->cmdServoAngleRad * (MAX_PULSE_WIDTH-MIN_PULSE_WIDTH) / 2*MAX_SERVO_ANGLE_RAD);  // in [us]
*  190205   mys   changed PWM repeat period from 10ms to 20ms to avoid
*                 servo jitter; DEFAULT_PULSE_PERIOD = 20000 in BallPlate.h 
*  190205   mys   controller gains: Kp = -5.0, Kd = -3.0 for Modelcraft VS-11AMB servo
*  190206   mys   changed PWM repeat period to 15ms to avoid servo jitter via
*                 DEFAULT_PULSE_PERIOD = 15000 in BallPlate.h 
*  190206   mys   Update of doxygen compatible comments, (re-)added HardFault_Handler()
*  200524   KF ballplate Team#1  included kalman filter
*  @date    01/12/2019
            24/05/2020
**************************************************************************
*/

#define     SW_VERSION        "V200312_team1"    //!< version string


/************************************************************************\
*    HEADERFILES
\************************************************************************/

#include  <stdio.h>              // C-StdLib sprintf(), scanf() prototypes


#include  "stm32f10x.h"          // STM32F10x Peripherals Definitions

#include  "platform_cfg.h"       // target specific peripherals config
#include  "bsp.h"                // low level IO, clock & int defs for uC/OS
#include  "..\basicIO\basicIO.h" // console text IO & LED control

/************************************************************************\
*    DEFINITIONS application related
\************************************************************************/

#include  "BallPlate.h"
#include  "auxSIOcomm.h"               //!< control related IO with PC via AuxSIO
#include "BallPlateKFlib.h"
UINT32 	  get_pc_from_intstack(void);  //!< used in HardFault_Handler( ) to read PC from int stack

/************************************************************************\
*    GLOBAL VARIABLES application related
\************************************************************************/

/************************************************************************\
*    LOCAL FUNCTION PROTOTYPES
\************************************************************************/

UINT16  setPWMSignalAngleX( SS_VAR *X );
UINT16  setPWMSignalAngleY( SS_VAR *Y );

void    print_float( float var );                      // L's legacy stuff
void    print_TwoFloat( const float, const float );


/************************************************************************\
*    MAIN
\************************************************************************/

int  main (void)
/**
*  ***********************************************************************
*  @brief  Initializes MCU peripherals, adjust/calibrate ballplate horizontal
*          position, run controller & estimation loop.
*  @retval none
**************************************************************************
*/
{
  UINT8  c;                        //!< user keyboard input char 
  UINT8  OpMode;                   //!< control flags for various run modes 
//UINT8  bFirstSampleLoop = TRUE;  //!< flag to init xEstCurr in 1st loop
  UINT32 kSampleIndex     = 0;     //!< sample index after controller activation
  
  char   cmdLineBuf[16];           //!< for user console input
  UINT8  auxSIORxBuf[16];          //!< for communication with PC
  
  INT8   calXServoOffsetDeg;       //!< xAxis servo horizontal cal offset in [o] 
  INT16  calXServoOffsetPWM;       //!< xAxis servo horizontal cal offset in [us] 
  UINT16 cmdXServoPWM;             //!< xAxis servo PWM cmd in [us] 

  #ifdef OBSERVER
  const SS_PAR SS = { BB_SS_F, BB_SS_G, BB_SS_C, BB_SS_K, BB_SS_V, BB_SS_L };
  /* uses model/observer/controller parameters from "BallPlate.h"     */
  #elif defined KF
  SS_PAR_KF SSx = { BB_SS_F, BB_SS_G, BB_SS_C, BB_SS_K, BB_SS_V, BB_SS_L, BB_SS_P, BB_SS_P1, BB_SS_Q, BB_SS_R};
  SS_PAR_KF SSy = { BB_SS_F, BB_SS_G, BB_SS_C, BB_SS_K, BB_SS_V, BB_SS_L, BB_SS_P, BB_SS_P1, BB_SS_Q, BB_SS_R};
  /* uses model/Kalman Filter/controller parameters from "BallPlate.h"     */
  #else
    #error "Illegal value for system ORDER ! Valid values: _SYS_2ND_ORDER_OBS_ or _SYS_3RD_ORDER_OBS_ or _SYS_3RD_ORDER_KF_ or _SYS_4TH_ORDER_KF_"
  #endif
  SS_VAR xAxis = { PLATE_XMAX_PXL, PLATE_XMIN_PXL,  0.0, 0.0,  0.0, 0.0,  0, 0, 0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 1500 }; 
  // manually measured max/min xAxis-pos. of ball
  // DEF_PULSE_WIDTH = 0.0 init -> ssvEst_k[0]
  
  SS_VAR yAxis = { PLATE_YMAX_PXL, PLATE_YMIN_PXL,  0.0, 0.0,  0.0, 0.0,  0, 0, 0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 1500 };
  // manually measured max/min yAxis-position of ball

  // init servo command horizontal calibration offsets
  xAxis.calServoAngleOffsetPWM = PWM_SERVO_X_OFFSET;
  yAxis.calServoAngleOffsetPWM = PWM_SERVO_Y_OFFSET;
  
  OpMode = OPMODE_CLOSED_LOOP_CONTROL;


/************************************************************************\
*    initialize peripherals & 'clear' screen & print hello message 
\************************************************************************/
  
  BSP_Init( );          // peripherals initialization
  BSP_SetLED( 0x55 );   // set initial LED on/off pattern
  
// print hello message
  print_string( "\n\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\r" );
  print_string( "BallPlate Controller using " );

#ifdef        _SYS_2ND_ORDER_OBS_
  print_string( " 2nd Order Observer\n\n\r" );
#elif defined _SYS_3RD_ORDER_OBS_
  print_string( " 3rd Order Observer\n\n\r" );
#elif defined _SYS_3RD_ORDER_KF_
  print_string( " 3rd Order Kalman Filter\n\n\r" );
#elif defined _SYS_4TH_ORDER_KF_
  print_string( " 4th Order Kalman Filter\n\n\r" );	
#endif

//print_string( "uC/OS, The Real-Time Kernel, Version " );
//print_uint( OSVersion( ) ); 
  print_string( "on " TARGET_BOARD " board with " TARGET_MCU " MCU running at " );
  print_uint( BSP_CPU_ClkFreq() / 1000000UL );  // print CPU clock in MHz
  print_string( " MHz ");
//print_string( " Hz\n\rSystem Timer Tick-Rate ");
//print_uint( OS_TICKS_PER_SEC );  // print current value
//print_string( " Hz\n\n\r" );
  print_string( "\n\n\rProgram BallPlateMain.c " SW_VERSION "\n\n\n\n\r" );

  // init TIMER channels to generate PWM signals for X-/Y-axis servo actuator
  TIM_GPIO->ARR    = DEFAULT_PULSE_PERIOD;     // pulse repeat period 10,000us = 10ms
  TIM_XAXIS_RSTVAL = setPWMSignalAngleX( &xAxis );
  TIM_YAXIS_RSTVAL = setPWMSignalAngleY( &yAxis );

  print_string("c = X-Servo Horiz. Cal. Offset   r = Run Control Loop\n\n\n\r");

  while (TRUE)     // mini command interpreter
  {
    c = _keyhit(); // get char if there was keyboard input
    if ( c != 0 ) 
    {
      switch (c)                  
      {
      case 'r':
        print_string("\n\rr -> Run control loop\n\n\n\r");
        OpMode = OPMODE_CLOSED_LOOP_CONTROL;
        break;
      case '-':
        //dcount = dcount << 1;  // dcount = dcount * 2   ->  turn slower
        break;
      case 'c':                  // enter X servo cal offset in [o]
        print_string("\n\rc -> X-Servo Cal Offset in [o] = ?  ");
        getint(cmdLineBuf);                                    // read number
        calXServoOffsetDeg = asciitoint(cmdLineBuf);           // convert to integer
        print_string("     Input was ");                       // verify
        print_int(calXServoOffsetDeg);
        print_string("\n\r");

        calXServoOffsetPWM = 
          calXServoOffsetDeg * (MAX_PULSE_WIDTH-MIN_PULSE_WIDTH) / (2*MAX_SERVO_ANGLE_DEG); 
        print_string("\n\rX-Servo Cal Offset in [us] = ");     // verify
        print_int(calXServoOffsetPWM);
        print_string("\n\r");
      
        xAxis.calServoAngleOffsetPWM = calXServoOffsetPWM;
        TIM_XAXIS_RSTVAL = cmdXServoPWM = setPWMSignalAngleX( &xAxis );
        print_string("\n\rX-Servo PWM in [us] = ");            // verify
        print_int(DEFAULT_PULSE_PERIOD-cmdXServoPWM);
        print_string("\n\r");
    
        break;
      default:
        print_string("c = X-Servo Horiz. Cal. Offset   r = Run Control Loop\n\n\n\r");
        break;
      }
    }
    if ( c == 'r' ) break;  // leave command interpreter & start control loop
  } // end while


  if ( OpMode == 0 )
  {
     print_string("\n\n\rILLEGAL OpMode == 0 !!\n\n\n\r");
    _getch();
  }    

  // init TIMER channels to generate PWM signals for X-/Y-axis servo actuator
  //TIM_GPIO->ARR    = DEFAULT_PULSE_PERIOD;     // pulse repeat period 10,000us = 10ms
  //TIM_XAXIS_RSTVAL = setPWMSignalAngleX( &xAxis );
  //TIM_YAXIS_RSTVAL = setPWMSignalAngleY( &yAxis );


/************************************************************************\
*    real-time loop: state feedback controller & state estimation 
\************************************************************************/

  while ( TRUE ) // do forever  -  real-time controller & estimation loop
  {

#ifdef DEBUG_MSG_ON
    NEWLINE
#ifdef _SYS_2ND_ORDER_OBS_
    _putch('2');
#elif defined _SYS_3RD_ORDER_OBS_
    _putch('3');
#elif defined _SYS_3RD_ORDER_KF_
    _putch('3');
#elif defined _SYS_4TH_ORDER_KF_
    _putch('4');		
#endif
    _putch(' ');
#endif

    // receive image processing data set, i.e. ball position measurements, from PC
    BSP_TurnOnOffLED( oneCompleteLoop, LED_ON );   // LEDs 12&15 ON ???

    serialReceive( auxSIORxBuf );              

    BSP_TurnOnOffLED( serialReception, LED_OFF );   // LEDs 12&15 ON ???


    if ( checkTransfer( auxSIORxBuf ) ) // checksum verified, received PC message is OK
    {
      // save old position measurement values
      xAxis.posOldPxl = xAxis.posMeasPxl;
      yAxis.posOldPxl = yAxis.posMeasPxl;
 
      // 're-assemble' UINT16 coordinate values from NEW RECEIVED PC message
      xAxis.posMeasPxl = (auxSIORxBuf[1] << 8) + auxSIORxBuf[2];
      yAxis.posMeasPxl = (auxSIORxBuf[3] << 8) + auxSIORxBuf[4];
      xAxis.posRefPxl  = (auxSIORxBuf[5] << 8) + auxSIORxBuf[6];
      yAxis.posRefPxl  = (auxSIORxBuf[7] << 8) + auxSIORxBuf[8];

      // rescale current and reference position from [pxl] to [m]
      xAxis.posMeas = xAxis.posMeasPxl / PIXEL_PER_METER;
      yAxis.posMeas = yAxis.posMeasPxl / PIXEL_PER_METER;
      xAxis.posRef  = xAxis.posRefPxl  / PIXEL_PER_METER;
      yAxis.posRef  = yAxis.posRefPxl  / PIXEL_PER_METER;

      kSampleIndex++;
      
    //if ( bFirstSampleLoop ) // 1st loop -> initialize state vector elements
      if ( kSampleIndex < 5 ) // keep everything static in first 5 controller cycles
      {
       //bFirstSampleLoop = FALSE;
        
        xAxis.posOldPxl    = xAxis.posMeasPxl; // so that 1st x.speedCur == 0
        xAxis.ssvEst_k[0]  = xAxis.posMeas;
        xAxis.ssvEst_k[1]  = 0.0;
        xAxis.ssvEst_k1[0] = xAxis.posMeas;    // ERROR in 1st cycle for [k+1] values !!!!
        xAxis.ssvEst_k1[1] = 0.0;              // see INIT-JERK logfiles 170126 & 170206

        yAxis.posOldPxl    = yAxis.posMeasPxl; // so that 1st y.speedCur == 0
        yAxis.ssvEst_k[0]  = yAxis.posMeas;
        yAxis.ssvEst_k[1]  = 0.0;
        yAxis.ssvEst_k1[0] = yAxis.posMeas;
        yAxis.ssvEst_k1[1] = 0.0;

#if ORDER == 3        
        xAxis.ssvEst_k[2]  = 0.0;
        yAxis.ssvEst_k[2]  = 0.0;
        xAxis.ssvEst_k1[2] = 0.0;
        yAxis.ssvEst_k1[2] = 0.0;

#endif

#if ORDER == 4       
        xAxis.ssvEst_k[3]  = 0.0;
        yAxis.ssvEst_k[3]  = 0.0;
        xAxis.ssvEst_k1[3] = 0.0;
        yAxis.ssvEst_k1[3] = 0.0;

#endif
      }

      // compute ball speed by simple backward difference of position measurements
      xAxis.speedCur = (xAxis.posMeasPxl - xAxis.posOldPxl) / T;    // scaled in pxl/s
      yAxis.speedCur = (yAxis.posMeasPxl - yAxis.posOldPxl) / T;

      
      // controller & observer & KF calculation, compute new servo actuator angle
      BSP_TurnOnOffLED( calcFSFAndObs, LED_ON );
#ifdef OBSERVER
      bbSSControlAndObserver( &xAxis, &yAxis, &SS );
#elif defined KF
			bbSSControlAndKF(&xAxis, &yAxis, &SSx, &SSy );
#endif
      BSP_TurnOnOffLED( calcFSFAndObs, LED_OFF );

      
      // compute PWM pulse duration from setpoint of servo actuator angle
      BSP_TurnOnOffLED( calcPWM, LED_ON );
      TIM_XAXIS_RSTVAL  = setPWMSignalAngleX( &xAxis );
      TIM_YAXIS_RSTVAL  = setPWMSignalAngleY( &yAxis );
      BSP_TurnOnOffLED( calcPWM, LED_OFF );


      // send control related data to PC for logging
      BSP_TurnOnOffLED( serialSend, LED_ON );
      sendData(&xAxis, &yAxis);
      BSP_TurnOnOffLED( serialSend, LED_OFF );

      BSP_TurnOnOffLED( oneCompleteLoop, LED_OFF );


#ifdef DEBUG_MSG_ON
      print_TwoFloat(xAxis.posRef,  yAxis.posRef);
      print_TwoFloat(xAxis.posMeas, yAxis.posMeas);
      print_TwoFloat(xAxis.ssvEst_k[0], yAxis.ssvEst_k[0]);
      print_TwoFloat(xAxis.ssvEst_k[1], yAxis.ssvEst_k[1]);

#if ORDER == 3
      //print_TwoFloat( xAxis.ssvEst_k[2], yAxis.ssvEst_k[2]);
#endif
      //print_TwoFloat(xAxis.posMeas - xAxis.posEst, yAxis.posMeas - yAxis.posEst);
      print_TwoFloat(xAxis.cmdServoAngleRad, yAxis.cmdServoAngleRad);
#endif
    } 
    else // checksum verify FAILED, data from PC are NOT OK
    {
      _putch( '!' );       // indicate auxSIO communication error
      _putch( ASC_BELL );  // with a beep & !
      
#ifdef DEBUG_MSG_ON
      print_TwoFloat(xAxis.posRef, yAxis.posRef);
      print_string("   NaN      NaN   ");
      print_TwoFloat(xAxis.ssvEst_k[0], yAxis.ssvEst_k[0]);
      print_TwoFloat(xAxis.ssvEst_k[1], yAxis.ssvEst_k[1]);

#if ORDER == 3
      //print_TwoFloat( xAxis.ssvEst_k[2], yAxis.ssvEst_k[2]);
#endif
      //print_TwoFloat(xAxis.posMeas - xAxis.posEst, yAxis.posMeas - yAxis.posEst);
      print_TwoFloat(xAxis.cmdServoAngleRad, yAxis.cmdServoAngleRad);
#endif
    } // END if( CheckSumOK ...)

  } // END while( TRUE ) // real-time loop - do forever

} // END main()



/************************************************************************\
*    LOCAL FUNCTION DEFINITIONS
\************************************************************************/


UINT16  setPWMSignalAngleX( SS_VAR *X )
/**
*  ***********************************************************************
*
*  @brief  Compute PWMcmd value in [us] units from servo angle cmd in [rad].
*
*  @param  X : used/updates struct members cmdServoAngleRad and cmdServoAnglePWM 
*  @retval UINT16 CCR value in [us] units for timer
*
*  190111   mys  corrected:  / 2*MAX_SERVO_ANGLE_RAD)
**************************************************************************
*/
{
// for XY servo actuator                                           !!! - SIGN !!! see below
  X->cmdServoAnglePWM = CENTER_PULSE_WIDTH + X->calServoAngleOffsetPWM - 
   (INT16) (X->cmdServoAngleRad * (MAX_PULSE_WIDTH-MIN_PULSE_WIDTH) / 2*MAX_SERVO_ANGLE_RAD);  // in [us]
    // scaling      [rad]            [us]                               [rad]
  
  if ( X->cmdServoAnglePWM > MAX_PULSE_WIDTH ) X->cmdServoAnglePWM = MAX_PULSE_WIDTH;
  if ( X->cmdServoAnglePWM < MIN_PULSE_WIDTH ) X->cmdServoAnglePWM = MIN_PULSE_WIDTH;
    
  return DEFAULT_PULSE_PERIOD - X->cmdServoAnglePWM;  // CCRval = ARRperiod-PWM in [us]
}


UINT16  setPWMSignalAngleY( SS_VAR *Y )
/**
*  ***********************************************************************
*
*  @brief  Compute PWMcmd value in [us] units from servo angle cmd in [rad].
*
*  @param  Y : used/updates struct members cmdServoAngleRad and cmdServoAnglePWM 
*  @retval UINT16 CCR value in [us] units for timer
*
*  190111   mys  corrected:  / 2*MAX_SERVO_ANGLE_RAD)
**************************************************************************
*/
{
// for YZ servo actuator                                           !!! + SIGN !!!
  Y->cmdServoAnglePWM = CENTER_PULSE_WIDTH + Y->calServoAngleOffsetPWM +
   (INT16) (Y->cmdServoAngleRad * (MAX_PULSE_WIDTH-MIN_PULSE_WIDTH) / 2*MAX_SERVO_ANGLE_RAD);  // in [us]
    // scaling      [rad]            [us]                               [rad]

  if ( Y->cmdServoAnglePWM > MAX_PULSE_WIDTH ) Y->cmdServoAnglePWM = MAX_PULSE_WIDTH;
  if ( Y->cmdServoAnglePWM < MIN_PULSE_WIDTH ) Y->cmdServoAnglePWM = MIN_PULSE_WIDTH;

  return DEFAULT_PULSE_PERIOD - Y->cmdServoAnglePWM;  // CCRval = ARRperiod-PWM in [us]
}



void print_float(float var)
/**
*  ***********************************************************************
*  @brief  Print floating point number with 6 decimals to CONSOLE 
*          slightly strange legacy code by I. Lienhardt ... 
*
*  @param  var : floating point variable to print
*  @retval none
**************************************************************************
*/
{
  char str[16];
  
  int intPart;         // integer part (678).
  float f2;            // fractional part (0.01234567).
  int d2;              // fract part converted to integer (123).
  if (var < 0. ){
    var = -var;
    _putch('-');       // print - sign  
  }  
  else _putch('+');    // print + sign  
  
  intPart = var;
  f2 = var - intPart;
  d2 = (f2*100000);    // ie. print 6 decimals Mr. L ...

//  print_int(intPart);
//  _putch('.');
//  print_int(d2);
//  
  
  _sprintf (str, "%d.%05d", intPart, d2);
  print_string(str);
  
}


void print_TwoFloat( const float A, const float B)
/**
*  ***********************************************************************
*  @brief  Output of two floating point numbers on serial interface
*
*  @param  A : first  floating point variable to print
*  @param  B : second floating point variable to print
*  @retval none
**************************************************************************
*/
{
  print_float(A);
  _putch(' ');
  print_float(B);
  _putch(' ');
}



void    delay(UINT32 delaycount)
/**
*  ***********************************************************************
*
*  @brief  Simple active waiting loop,
*          effective delay time depends on CPU type & clock frequency !
*
*  @param  delaycount : 32-Bit delay loop starting value
*  @retval none
**************************************************************************
*/
{
  while ( delaycount-- );    // decrement until 0
}


void HardFault_Handler(void)
/**
*  ***********************************************************************
*  @brief  Prints warning message to terminal and 'halts' in infinite loop.
*          Overwrites original/dummy hardfault handler from STM32F10x.s !
*
*  @retval none
**************************************************************************
*/
{
    UINT32 pc;
    UINT32 *hfsr = (UINT32*) 0xE000ED2C;     // Hard Fault Status Register

    // read PC value from interrupt stack
    pc = get_pc_from_intstack( );
    print_string("\n\n\n\rHARDFAULT occurred @ PC = ");
    print_hex32(pc);
    print_string(" triggered by ");

    if ( *hfsr & (1<<31) )        // case DEBUGEVT
    {
      print_string("debug event");
    }
    else if ( *hfsr & (1<<30) )   // case FORCED
    {
      print_string("unhandled bus/mem/usage fault");
    }
    else if ( *hfsr & (1<<1) )    // case VECTBL
    {
      print_string("failed vector fetch");
    }
    else // illegal case
    {
      print_string("UNKNOWN/ILLEGAL case");
    }

    print_string("\n\r");

    while (1) {}   // 'halt' in infinite loop
}


/************************************************************************\
*    END OF MODULE BallPlateMain.c
\************************************************************************/
