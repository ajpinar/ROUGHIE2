/*
  UM7_BIN.cpp - Library for UM7 AHRS
  Created by Guilherme A. Ribeiro, August 22, 2014.
  Property of NASLAB, Michigan Technological University
*/

#include "UM7_BIN.h"

void UM7_BIN::commonInit()
{ // common initialization for hard and soft
  tail = buff;
  
  updated_p = 0;
  updated_h = 0;
}

void UM7_BIN::begin(SoftwareSerial* s)
{ // configure software UART   
  commonInit();
  
  ssFlag = 1;
  ss = s;
  ss->begin(115200);
}

void UM7_BIN::begin(HardwareSerial* s)
{ // configure hardware UART 
  commonInit();
  
  ssFlag = 0;
  hs = s;
  hs->begin(115200);
}

float UM7_BIN::BtoFloat(uint8_t* B)
{ // convert 4 bytes into a float
  union byte2float_t
  { // convert 4 bytes into a float
    uint8_t B[4]; 
    float f;
  } byte2float;
  
  byte2float.B[0] = B[3];
  byte2float.B[1] = B[2];
  byte2float.B[2] = B[1];
  byte2float.B[3] = B[0];
  
  return byte2float.f;
}

int16_t UM7_BIN::refresh(void)
{ // reads serial buffer, decodes and return status
  // must be called frequently or serial RX overflows
  int16_t ret = 0;
  
  if(ssFlag)
    while (ss->available() > 0 && ret < 3)
      ret = decode(ss->read());
  else
    while (hs->available() > 0 && ret < 3)
      ret = decode(hs->read());
 
  return ret;
}
 

int16_t UM7_BIN::decode(uint8_t c)
{ // routine that actually decodes the input stream 
  // shouldn't return error bigger or equal than 3
  struct packet_t
  {
    uint16_t Checksum;
    uint8_t Data[60]; // 15*4
    uint8_t Address;
    uint8_t PT;
  } packet;
  
  if(tail-buff >= RX_SIZE) { // serial overrun
    tail = buff;
    return 4;
  }
  *(tail++) = c;
  
  uint8_t *head;
  
  for(head = buff; head+2 < tail; head++) // find snp
    if(*head == 's' && *(head+1) == 'n' && *(head+2) == 'p')
      break;
  if(head+2 == tail) // if didn't find any
    return 0;
    
  if(head != buff) { // recover from garbage in the pipe
    memmove(buff, head, tail-head); // recycle buffer
    tail -= head-buff; // update buffer pointer
  }
    
  if(tail-head < MIN_PACKET_SIZE) // enough bytes? if so, it has a PT
    return 1;
    
  if(PACKET_SIZE(head) > (tail-head)) // is this specific packet complete?
    return 2;
    
  // Complete packet structure, check datasheet to understand the offsets
  packet.PT = *(head+3);
  packet.Address = *(head+4);
  int16_t packetSize = PACKET_SIZE(head);
  uint16_t calcChecksum = 's' + 'n' + 'p' + packet.PT + packet.Address;
  for(int16_t n = 0; n < (packetSize-7); n++) {
    packet.Data[n] = *(head+5+n);
    calcChecksum += packet.Data[n];
  }
  packet.Checksum = (*(head+5+packetSize-7) << 8) | (*(head+6+packetSize-7));
  
  memmove(buff, head+packetSize, tail-head-packetSize); // recycle buffer
  tail -= packetSize + buff-head; // update buffer pointer

  if(calcChecksum != packet.Checksum) // check checksum
    return 3;
 
  switch(packet.Address) { // all good, so, update variables
  
  case DREG_EULER_PHI_THETA: // pose state packet
    if(BL(packet.PT) != 9)
      return 5;
    
    updated_p = 1;
    roll   = ((packet.Data[ 0] << 8) | packet.Data[ 1]) / 91.02222;
    pitch  = ((packet.Data[ 2] << 8) | packet.Data[ 3]) / 91.02222;
    yaw    = ((packet.Data[ 4] << 8) | packet.Data[ 5]) / 91.02222;
    rolld  = ((packet.Data[ 8] << 8) | packet.Data[ 9]) / 91.02222;
    pitchd = ((packet.Data[10] << 8) | packet.Data[11]) / 91.02222;
    yawd   = ((packet.Data[12] << 8) | packet.Data[13]) / 91.02222;
    t_p1 = BtoFloat(&packet.Data[16]);
    north = BtoFloat(&packet.Data[20]);
    east = BtoFloat(&packet.Data[24]);
    up = BtoFloat(&packet.Data[28]);
    t_p2 = BtoFloat(&packet.Data[32]);
    
    return -1;
 
  case DREG_HEALTH: // health state packet

    updated_h = 1;
    sats_used = packet.Data[0] >> 2;
    HDOP = (packet.Data[1] | ((packet.Data[0] & 0x3) << 8)) / 10.0;
    sats_in_view = packet.Data[2] >> 2;
    flags = (packet.Data[3] & 0x3F) | ((packet.Data[1] & 0x1) << 6);
      
    return -2; 

  default: // unknown packet
	Serial.println("Address: " + String(packet.Address));
    return 6;
  }
}
