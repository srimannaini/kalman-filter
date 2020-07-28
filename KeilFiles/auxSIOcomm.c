/**
*  ***********************************************************************
*  @file    auxSIOcomm.c
*  @author  B. Mysliwetz
*  @version V1.00
*
*  @brief   Functions and defines for BallPlate control related serial 
*           communication with host PC.
*           Improved version of 'serialFunctions.c' from WS2010 Bachelor's  
*           Thesis by R Zinken-Sommer.
*
*  Changes:
*  07.12.2016 mys   _putch() -> _aux_putch() & _getch() -> _aux_getch()
*  06.02.2017 mys   streamlining of checkTransfer( )
*  27.02.2017 mys   Eliminated state machine restarts via 0x0D payload bytes
*                   if (pMsgRecv[1] == ASC_CR){rxState = XPOS_HI;} in serialReceive()
*
*  @date    27/02/2017
**************************************************************************
*/

#include  <string.h>             // needed for memcpy( )

#include  "..\basicIO\mctDefs.h" // useful constants, macros, type shorthands 
#include  "..\basicIO\basicIO.h" // console text IO & LED control

#include  "bsp.h"                // low level IO, clock & int related defs for uC/OS
#include  "BallPlate.h"          // defines system order

#include  "auxSIOcomm.h"         // control related communication w PC 


typedef enum RECEIVE_STATES      //!< for state machine to handle auxSIO comm. with PC
{
  FRAME_START  =  1,             //!< Start of frame
  XPOS_HI      =  2,             //!< High byte of x position
  XPOS_LO      =  3,             //!< Low  byte of x position
  YPOS_HI      =  4,             //!< High byte of y position
  YPOS_LO      =  5,             //!< Low  byte of y position
  XREF_HI      =  6,             //!< Low  byte of x reference point
  XREF_LO      =  7,             //!< Low  byte of x reference point
  YREF_HI      =  8,             //!< Low  byte of y reference point
  YREF_LO      =  9,             //!< Low  byte of y reference point
  CHECKSUM_HI  = 10,             //!< High byte of checksum
  CHECKSUM_LO  = 11,             //!< Low  byte of checksum
  RX_COMPLETE  = 12              //!< msg receive DONE

} RECEIVE_STATES;


UINT16 serialReceive( UINT8* pMsgRecv )  
/**
*  ***********************************************************************
*  @brief  Reads BallPlate position and reference data via auxSIO from PC;
*          waits until FRAME_START is detected, then tries to read complete
*          frame; if a new start is detected the incomplete one is discarded;
*          at detection of a new frame LEDs 12 and 15 will be turned on.
*
*  @param  pMsgRecv Uchar array with length 11 for received data
*  @retval SHORT number of received characters (with overhead due to
*          asynchronous reception)
**************************************************************************
*/
{
  UINT16 rxByteCount = 0;

  RECEIVE_STATES rxState = FRAME_START;

  do
  {
    switch( rxState )     // execute & propagate state machine states
    {
      case FRAME_START:
        pMsgRecv[0] = _aux_getch();
        rxByteCount++;
        if(pMsgRecv[0] == ASC_CR){ rxState = XPOS_HI ;}
        else { rxState = FRAME_START;}
        break;

      case XPOS_HI:
        BSP_TurnOnOffLED(serialReception, LED_ON);
        pMsgRecv[1] = _aux_getch();
        rxByteCount++;
        //if (pMsgRecv[1] == ASC_CR){ rxState = XPOS_HI ;}
        //else { rxState = XPOS_LO ;}
			  rxState = XPOS_LO ;
        break;

      case XPOS_LO:
        pMsgRecv[2] = _aux_getch();
        rxByteCount++;
        //if( pMsgRecv[2] == ASC_CR){ rxState = XPOS_HI ;}
        //else { rxState = YPOS_HI ;}
			  rxState = YPOS_HI ;
        break;

      case YPOS_HI:
        pMsgRecv[3] = _aux_getch();
        rxByteCount++;
        //if (pMsgRecv[3] == ASC_CR) { rxState = XPOS_HI ;}
        //else { rxState = YPOS_LO ;}
			  rxState = YPOS_LO ;
        break;

      case YPOS_LO:
        pMsgRecv[4] = _aux_getch();
        rxByteCount++;
        //if (pMsgRecv[4] == ASC_CR){ rxState = XPOS_HI ;}
        //else { rxState = XREF_HI ;}
			  rxState = XREF_HI ;
        break;

      case XREF_HI:
        pMsgRecv[5] = _aux_getch();
        rxByteCount++;
        //if (pMsgRecv[5] == ASC_CR) { rxState = XPOS_HI ;}
        //else { rxState = XREF_LO ;}
			  rxState = XREF_LO ;
        break;

      case XREF_LO:
        pMsgRecv[6] = _aux_getch();
        rxByteCount++;
        //if (pMsgRecv[6] == ASC_CR){ rxState = XPOS_HI ;}
        //else { rxState = YREF_HI ;}
			  rxState = YREF_HI ;
        break;

      case YREF_HI:
        pMsgRecv[7] = _aux_getch();
        rxByteCount++;
        //if (pMsgRecv[7] == ASC_CR){ rxState = XPOS_HI ;}
        //else { rxState = YREF_LO ;}
			  rxState = YREF_LO ;
        break;

      case YREF_LO:
        pMsgRecv[8] = _aux_getch();
        rxByteCount++;
        //if (pMsgRecv[8] == ASC_CR){ rxState = XPOS_HI ;}
        //else {rxState = CHECKSUM_HI ;}
			  rxState = CHECKSUM_HI ;
        break;

      case CHECKSUM_HI:
        pMsgRecv[9] = _aux_getch();
        rxByteCount++;
        //if (pMsgRecv[9] == ASC_CR){rxState = XPOS_HI ;}
        //else {rxState = CHECKSUM_LO ;}
			  rxState = CHECKSUM_LO ;
        break;

      case CHECKSUM_LO:
        pMsgRecv[10] = _aux_getch();
        rxByteCount++;
        //if (pMsgRecv[10] == ASC_CR) {rxState = XPOS_HI ;}
        //else { rxState = RX_COMPLETE ;}
			  rxState = RX_COMPLETE ;
        break;

      default:
        break;
     } // end switch( rxstate )
		 
  } while ( rxState != RX_COMPLETE );

  return rxByteCount;
}


UINT8  checkTransfer( UINT8* pPosMsgPC )
/**
*  ***********************************************************************
*  @brief     Check for error free reception of image data packet from PC.
*  @param     pPosMsgPC Hi/Lo byte of ball X/Y-positions, X/Y reference
*             positions plus the checksums
*  @retval    0: checksum ERROR     1: checksum OK.   
**************************************************************************
*/
{
  UINT16 xPos, yPos, xRef, yRef, CheckSumPC, CheckSumChk;

  xPos = (pPosMsgPC[1] << 8) + pPosMsgPC[2];
  //X  = X + pPosMsgPC[2];
  yPos = (pPosMsgPC[3] << 8) + pPosMsgPC[4];
  //Y  = Y + pPosMsgPC[4];
  xRef = (pPosMsgPC[5] << 8) + pPosMsgPC[6];
  //RefX = RefX + pPosMsgPC[6];
  yRef = (pPosMsgPC[7] << 8) + pPosMsgPC[8];
  //RefY = RefY + pPosMsgPC[8];
  //temp = pPosMsgPC[9] << 8;
  //temp = temp + pPosMsgPC[10];

  CheckSumPC  = (pPosMsgPC[9] << 8) + pPosMsgPC[10]; // received checksum value
  CheckSumChk = ~(xPos + yPos + xRef + yRef); // bit-inverted sum of all payload values
  //CheckSumChk = ~CheckSumChk;

  if( (CheckSumChk == CheckSumPC) && (xPos != 0) && (yPos != 0) )
    return 1; // checksums are valid and ball was detected    => NO ERROR

  return 0; // checksums are invalid or ball was not detected => ERROR
}


void sendData( const struct SS_VAR *X, const struct SS_VAR *Y )
/**
*  ***********************************************************************
*  @brief  Send data to the PC.
*  @param  X pointer to struct of type SS_VAR for x-axis
*  @param  Y pointer to struct of type SS_VAR for y-axis
**************************************************************************
*/
{
  sendFloat('a', X->speedCur);                   //11 Byte = 1 identifier + 8 data bytes + 2 checksum bytes
  sendFloat('b', Y->speedCur);                   //11 Byte
  sendFloat('c', X->cmdServoAngleRad*CRAD2DEG);  //11 Byte = 1 identifier + 8 data bytes + 2 checksum bytes
  sendFloat('d', Y->cmdServoAngleRad*CRAD2DEG);  //11 Byte
  sendShort('e', X->posMeasPxl);                 // 7 Byte = 1 identifier + 4 data bytes + 2 checksum bytes
  sendShort('f', Y->posMeasPxl);                 // 7 Byte
  sendShort('g', X->posRefPxl);                  // 7 Byte
  sendShort('h', Y->posRefPxl);                  // 7 Byte
  sendFloat('i', X->alphaPlateRad*CRAD2DEG);     //11 Byte     corr  Mys 171115
  sendFloat('j', Y->alphaPlateRad*CRAD2DEG);     //11 Byte
  sendShort('k', X->cmdServoAnglePWM);           // 7 Byte
  sendShort('l', Y->cmdServoAnglePWM);           // 7 Byte
  sendFloat('m', X->ssvEst_k1[0]*PIXEL_PER_METER); //11 Byte   [m]   in [pxl]
  sendFloat('n', X->ssvEst_k1[1]*PIXEL_PER_METER); //11 Byte   [m/s] in [pxl/s]
  sendFloat('o', Y->ssvEst_k1[0]*PIXEL_PER_METER); //11 Byte         in [pxl]
  sendFloat('p', Y->ssvEst_k1[1]*PIXEL_PER_METER); //11 Byte         in [pxl/s]
                                                 // ---------
                                                 // 130 Byte per processing cycle / image frame
  _aux_putch(ASC_LF); // is ignored by PC
  _aux_putch(ASC_CR); // for a neat display (not HYPERTERMINAL)
}


void sendFloat( UINT8 msgID, float floatWORD )
/**
*  ***********************************************************************
*  @brief  Sends float data via auxSIO to PC.
*
*  Splits a FLOAT value in 6 bytes and generates a checksum
*  Furthermore splits every byte in two upper and lower 4 bits and sends
*  them over the COM interface to the PC.
*
*  This is necessary to ensure that identifiers and data can be distinguished.
*  Data are in the range of 0x30 to 0x39 and 0x41 to 0x46, every other value is
*  reserved for identifiers.
*
*  @param[in] msgID     : ID for float data msg
*  @param[in] floatWORD : Data to convert & xmit
**************************************************************************
*/
{
  UINT8 wbufr[5];
  UINT8 CheckSum;
  UINT8 loByte, hiByte;
  UINT8 i;

  memcpy( wbufr, &floatWORD, sizeof(floatWORD) );
	
  CheckSum = ~( wbufr[0]+wbufr[1]+wbufr[2]+wbufr[3] );
  //CheckSum = ~CheckSum;
  wbufr[4] = CheckSum;

  _aux_putch(msgID);
	
  for (i=0; i<5; i++)  // convert & xmit each binary byte as 2 ascii hex bytes
  {
//  loByte =  wbufr[i] % 16;          // split into high/low 4 bits
//  hiByte = (wbufr[i] / 16) % 16;
		
    loByte =  wbufr[i] & 0x0F;        // split up into high/low nibble
    hiByte = (wbufr[i] >> 4) & 0x0F;

    if( hiByte < 10)
      hiByte = 0x30 + hiByte;
    else
      hiByte = 0x37 + hiByte;
    if( loByte < 10)
      loByte = 0x30 + loByte;
    else
      loByte = 0x37 + loByte;

    _aux_putch(hiByte);
    _aux_putch(loByte);
  }
} // end sendFloat( )


void sendShort( UINT8 msgID, short Word16Bit )
/**
*  ***********************************************************************
*  @brief  Sends variables of type short over the RS232 interface.
*
*  Splits a short value in 4 bytes and generates a checksum
*  Furthermore splits every byte in two upper and lower 4 bits and sends
*  them over the COM interface to the PC.
*
*  This is necessary to ensure that identifiers and data can be distinguished.
*  Data are in the range of 0x30 to 0x39 and 0x41 to 0x46, every other value is
*  reserved for identifiers
*
*  @param[in] msgID     : ID of 16-Bit word data msg
*  @param[in] Word16Bit : 16-Bit word to convert & send
**************************************************************************
*/
{
  UINT8 wbufr[3];
  UINT8 CheckSum;
  UINT8 loByte, hiByte;
  UINT8 i;

  memcpy( wbufr, &Word16Bit, sizeof(Word16Bit) );
  CheckSum = ~( wbufr[0] + wbufr[1] );
  //CheckSum = ~CheckSum;
  wbufr[2] = CheckSum;

  _aux_putch(msgID);

  for (i=0; i<3 ;i++)  // convert & xmit each binary byte as 2 ascii hex bytes
  {
//  loByte = wbufr[i] % 16;         // split in high/low 4 bits
//  hiByte = (wbufr[i]/16)%16;

    loByte =  wbufr[i] & 0xF;       // split in high/low 4 bits
    hiByte = (wbufr[i] >> 4) & 0xF;

    if(hiByte < 10)                 // convert binary value to ASCII-Hex
    {
      hiByte = 0x30 + hiByte;       // ASCII digit  0..9
    } else {
      hiByte = 0x37 + hiByte;       // ASCII letter A..F
    }
		
    if(loByte < 10)
		{
      loByte = 0x30 + loByte;
    } else{
      loByte = 0x37 + loByte;
    }

    _aux_putch(hiByte);
    _aux_putch(loByte);
  }
} // end  sendShort( )


/************************************************************************\
*    END OF MODULE auxSIOcomm.c
\************************************************************************/
