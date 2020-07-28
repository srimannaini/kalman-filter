/**
*  ***********************************************************************
*  @file    auxSIOcomm.h
*  @author  B. Mysliwetz
*  @version V0.01
*  @brief   Functions and defines for control related PC communication.
*
*  @date    2016/08/19
**************************************************************************/


#ifndef AUX_SIO_COMM_H
#define AUX_SIO_COMM_H


/** Enum for setting LEDs.
*   Range of LEDs 8-15.
*/
typedef enum STATUS_LEDS  // enum for setting LEDS
{ serialSend      =  8,   //!< indicate sending process of data over RS232 interface to PC
  calcFSFAndObs   = 10,   //!< indicate controller calculations (FSF and Observer/Kalman Filter)
  calcPWM         = 11,   //!< indicate calculations for the timer/PWM parameters
  serialReception = 12,   //!< indicate reception process of data over RS232 interface
  oneCompleteLoop = 15    //!< indicate runtime of one whole loop
} STATUS_LEDS;

#define   LED_ON    0     // turns LED ON
#define   LED_OFF   1     // turns LED OFF


/* Function Prototypes */

void   sendData( const struct SS_VAR *X, const struct SS_VAR *Y );
void   sendFloat( UINT8, float );
void   sendShort( UINT8, short );
UINT16 serialReceive( UINT8* pReceive );
UINT8  checkTransfer( UINT8* );


/// Function for clearing console screen (works for putty)
#define CLEAR_SCREEN  DB_print_string("\x1B[2J");

/// Function for newline over RS232 interface
#define NEWLINE       DB_putchar(ASC_CR); DB_putchar(ASC_LF);

#endif /* AUX_SIO_COMM_H */

/************************************************************************\
*    END OF MODULE auxSIOcomm.h
\************************************************************************/
