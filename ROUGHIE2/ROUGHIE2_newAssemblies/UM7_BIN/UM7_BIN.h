/*
  UM7_BIN.h - Library for UM7 AHRS
  Created by Guilherme A. Ribeiro, August 22, 2014.
  Property of NASLAB, Michigan Technological University
*/

#ifndef UM7_def
#define UM7_def

#ifdef __AVR__
    #include <Arduino.h>
#endif
#include <SoftwareSerial.h>

#define MIN_PACKET_SIZE   11 // health packet
#define RX_SIZE           50 // buffer size, greater than maximum packet size

#define HAS_DATA(PT)   ((PT>>7)&0x1)
#define IS_BATCH(PT)   ((PT>>6)&0x1)
#define BL(PT)         ((PT>>2)&0x0F)  // batch length
#define HIDDEN(PT)     ((PT>>1)&0x1)   // unused
#define CF(PT)         ((PT)&0x1)

#define PACKET_SIZE(HEAD) (7+4*(HAS_DATA(*(HEAD+3))?(IS_BATCH(*(HEAD+3))?BL(*(HEAD+3)):1):0))

#define DREG_HEALTH          0x55  // address of health register
#define DREG_EULER_PHI_THETA 0x70  // initial address of pose packet

class UM7_BIN
{
  public:
    void begin(SoftwareSerial*);    // using software serial
    void begin(HardwareSerial*);    // using hardware serial
    int16_t refresh(void);          // call as frequent as possible to clear RX
    
    // pose packet outputs
    uint16_t updated_p;
    float t_p1, t_p2, roll, pitch, yaw, rolld, pitchd, yawd, north, east, up;
    
    // health packet outputs
    uint16_t updated_h;
    float HDOP;
    int16_t sats_used, sats_in_view, flags;
    
  private:
    void commonInit(void);   // both soft and hard execute this
    int16_t decode(uint8_t); // platform independent decoding routine
    static float BtoFloat(uint8_t*); // 4 bytes to float
    
    SoftwareSerial *ss; // software serial
    HardwareSerial *hs; // hardware serial
    bool ssFlag;        // true if software serial is in use
    
    uint8_t buff[RX_SIZE]; // stores all incoming data
    uint8_t *tail;         // points to first empty space in buffer
};

#endif
