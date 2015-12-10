#include <SoftwareSerial.h>
#include <UM7_BIN.h>
#include <DynamixelSoftSerial.h>

/*
  Glider motion
  Command wirelesly the motion of the syringes and transfer the sensor data
  
  Connect SPI header of UM7 into Arduino (vin, gnd, rx tx)
  Connect the secundary header of UM7 into the GPS (rx2, tx2)
  
  Caution: SoftwareSerial has conflicts with servo
*/

#define DRY_MODE

#define PIN_solenoid   2    // Wire labeled with '0'
#define PIN_PS         4          // Wire labeled with '2'
#define PIN_pumpDir    8     // Wire labeled with '3'
#define PIN_pumpOnDIO 13  // Wire labeled with '5'
#define PIN_GO         7          // External switch to start go timer

#define PIN_dyna_rx 11      // Need a jumper on shield from pin 11 to RX
#define PIN_dyna_tx 12      // Need a jumper on shield from pin 12 to TX

#define PIN_pressureSensorPin A3
#define PIN_voutPin            3
#define PIN_DWSensor          A2

#define GC_NULL  0
#define GC_RESET 1
#define GC_START 2
#define GC_STOP  3
#define GC_BEGIN 4

#define ME_TRANSCEIVE 0
#define ME_GLIDE_DOWN 1
#define ME_GLIDE_UP   2

struct param_t {
  int DWSensorBackPosition;       // bbak
  int DWSensorCenterPosition;     // bmid
  int DWSensorFrontPosition;      // bfro
  unsigned long int descentTime;  // desct
  unsigned long int riseTime;     // riset
  unsigned long int transTime;    // trant
  int pitch_pos_down;             // pdwn
  int pitch_pos_mid;              // pmid
  int pitch_pos_up;               // pupp
  int pitch_vel;                  // pvel
  int enPID;                      // epid
  float sp;                       // setp
  float kp;                       // kp
  float ki;                       // ki
  float kd;                       // kd
} param;

UM7_BIN um7;

unsigned long int tSensor, dtSensor;
bool enSensor;

unsigned long int tGlider, dtGlider;
bool enGlider;

bool enDebug;
bool enPID;

const int NLOG = 100;
int nLog = 0;
struct dataLog_t {
  float pitch;
  float pressure;
} dataLog[NLOG];

#define HELP "Commands: \n\tss [-e|-d] \n\tss [-s] \n\tgc [-e|-d] \n\tgc [-r] \n\tpr [-bbak|-bmid|-bfro|-desct|-riset|-trant|-pdwn|-pmid|-pupp|-pvel|-epid|-setp|-kp|-ki|-kd] [newValue]\n\tpr [-s]\n"

void setup()
{
  // setup imu
  um7.begin(&Serial3);
  
  // setup communication and send the interface help message
  Serial.begin(9600);
  Serial.setTimeout(100);
  Serial.println(HELP);
  
  // setup synchronous events
  enSensor = 0;
  dtSensor = 1000;
  tSensor = millis();
  
  enGlider = 0;
  dtGlider = 200;
  tGlider = millis() - 50;
  
  // setup glider debug messages
  enDebug = 1;
  
  // initial parameters
  param.DWSensorBackPosition = 980;
  param.DWSensorCenterPosition = 825;
  param.DWSensorFrontPosition = 670;
  param.descentTime = 10000;
  param.riseTime = 10000;
  param.transTime = 10000;
  param.pitch_pos_down = 1;
  param.pitch_pos_mid = 250;
  param.pitch_pos_up = 550;
  param.pitch_vel = 200;
  param.enPID = 0;
  param.sp = 35.0;
  param.kp = 50.0;
  param.ki = 0.0;
  param.kd = 50.0;
  
  // setup glider and make it still
  gliderStateMachine(GC_BEGIN);
  gliderStateMachine(GC_STOP);
}

void loop()
{
  const int BUFF_LEN = 80;
  char buff[BUFF_LEN];
  
  int ret = um7.refresh(); // refresh navigational data
  
  if(Serial.available()) // user command >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  {  
    char arg[3][80];
    int len = Serial.readBytesUntil('\r', buff, BUFF_LEN);
    buff[len] = '\0';
    
    Serial.print("@");
    Serial.print(millis());
    Serial.print(" > ");
    Serial.println(buff); // echo back with timestamp
    
    sscanf(buff, "%s %s %s", arg[0], arg[1], arg[2]); // parsing
    
    if(strcmp(arg[0], "gc") == 0) {
      if(strcmp(arg[1], "-e") == 0) { // enable glider cycle
        gliderStateMachine(GC_START);
        enGlider = 1;
      }
      else {if(strcmp(arg[1], "-d") == 0) { // disable glider cycle
        gliderStateMachine(GC_STOP);
        enGlider = 0;
      }
      else {if(strcmp(arg[1], "-r") == 0) { // reset position for trimming: go to middle
        gliderStateMachine(GC_RESET);
      }
      else Serial.println(HELP); // print help is no match was found
    }}}
    else {if(strcmp(arg[0], "ss") == 0) {
      if(strcmp(arg[1], "-e") == 0) { // enable sensor broadcast
        enSensor = 1;
      }
      else {if(strcmp(arg[1], "-d") == 0) { // disable sensor broadcast
        enSensor = 0;
      }
      else {if(strcmp(arg[1], "-s") == 0) { // disable sensor broadcast
        for(int n = 0; n < NLOG; n++)
          Serial.println(dataLog[n].pitch);
      }
      else Serial.println(HELP); // print help is no match was found
    }}}
    else {if(strcmp(arg[0], "pr") == 0) { 
      if(strcmp(arg[1], "-s") == 0) {        // show all parameter values
        Serial.print(" bbak = "); Serial.println(param.DWSensorBackPosition);
        Serial.print(" bmid = "); Serial.println(param.DWSensorCenterPosition);
        Serial.print(" bfro = "); Serial.println(param.DWSensorFrontPosition);
        Serial.print(" desct = "); Serial.println(param.descentTime);
        Serial.print(" riset = "); Serial.println(param.riseTime);
        Serial.print(" trant = "); Serial.println(param.transTime);
        Serial.print(" pdwn = "); Serial.println(param.pitch_pos_down);
        Serial.print(" pmid = "); Serial.println(param.pitch_pos_mid);
        Serial.print(" pupp = "); Serial.println(param.pitch_pos_up);
        Serial.print(" pvel = "); Serial.println(param.pitch_vel);
        Serial.print(" epid = "); Serial.println(param.enPID);
        Serial.print(" setp = "); Serial.println(param.sp);
        Serial.print(" kp = "); Serial.println(param.kp);
        Serial.print(" ki = "); Serial.println(param.ki);
        Serial.print(" kd = "); Serial.println(param.kd);
        Serial.println();
      }
      else {if(strcmp(arg[1], "-bbak") == 0) {        // update parameter: draw wire sensor
        param.DWSensorBackPosition = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-bmid") == 0) { // update parameter: descent time
        param.DWSensorCenterPosition = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-bfro") == 0) { // update parameter: rise time
        param.DWSensorFrontPosition = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-desct") == 0) { // update parameter: maximum pitch position
        param.descentTime = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-riset") == 0) { // update parameter: trimming pitch position
        param.riseTime = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-trant") == 0) { // update parameter: trimming pitch position
        param.transTime = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-pdwn") == 0) { // update parameter: trimming pitch position
        param.pitch_pos_down = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-pmid") == 0) { // update parameter: trimming pitch position
        param.pitch_pos_mid = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-pupp") == 0) { // update parameter: trimming pitch position
        param.pitch_pos_up = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-pvel") == 0) { // update parameter: trimming pitch position
        param.pitch_vel = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-epid") == 0) { // update parameter: trimming pitch position
        param.enPID = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-setp") == 0) { // update parameter: trimming pitch position
        param.sp = atof(arg[2]);
      }
      else {if(strcmp(arg[1], "-kp") == 0) { // update parameter: trimming pitch position
        param.kp = atof(arg[2]);
      }
      else {if(strcmp(arg[1], "-ki") == 0) { // update parameter: trimming pitch position
        param.ki = atof(arg[2]);
      }
      else {if(strcmp(arg[1], "-kd") == 0) { // update parameter: trimming pitch position
        param.kd = atof(arg[2]);
      }
      else Serial.println(HELP); // print help is no match was found
    }}}}}}}}}}}}}}}}
    else Serial.println(HELP); // print help is no match was found
  }}} // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  
  if(enGlider && tGlider + dtGlider < millis()) {
    tGlider = millis();
    gliderStateMachine(GC_NULL); // manages all actions. just make sure to call it as frequently as possible
  }
  
  if(enSensor && tSensor + dtSensor < millis()) {
    tSensor = millis();
    
    if(ret >= 3) { // unexpected error detected
      Serial.print("<"); Serial.print(ret); Serial.println(">"); 
    }
    
    if(um7.updated_p) { // pose packet
      um7.updated_p = 0;
      Serial.print("t_p1 = "); Serial.print(um7.t_p1); Serial.print("; ");
      Serial.print("roll = "); Serial.print(um7.roll); Serial.print("; ");
      Serial.print("pitch = "); Serial.print(um7.pitch); Serial.print("; ");
      Serial.print("yaw = "); Serial.print(um7.yaw); Serial.print("; ");
      Serial.print("rolld = "); Serial.print(um7.rolld); Serial.print("; ");
      Serial.print("pitchd = "); Serial.print(um7.pitchd); Serial.print("; ");
      Serial.print("yawd = "); Serial.print(um7.yawd); Serial.print("; ");
      Serial.print("north = "); Serial.print(um7.north); Serial.print("; ");
      Serial.print("east = "); Serial.print(um7.east); Serial.print("; ");
      Serial.print("up = "); Serial.print(um7.up); Serial.print("; ");
      Serial.print("t_p2 = "); Serial.print(um7.t_p2); Serial.println("; ");
    }
    if(um7.updated_h) { // health packet
      um7.updated_h = 0;
      Serial.print("sats_in_view = "); Serial.print(um7.sats_in_view); Serial.print("; ");
      Serial.print("sats_used = "); Serial.print(um7.sats_used); Serial.print("; ");
      Serial.print("flags = "); Serial.print(um7.flags); Serial.print("; ");
      Serial.print("HDOP = "); Serial.print(um7.HDOP); Serial.println("; ");
    }
  } 
}


void gliderStateMachine(int cmd) {
  
  static int state;        // machine state
  static bool entry;           // if it's first time executing the current state
  static unsigned long int t0; // initial time of current state
  static int cycleCount;
  
  const int dyna_id = 1;
  const long int dyna_Sbaud = 57600;
  
  float pPos, pVel;
  
  if(cmd == GC_BEGIN) { // execute once at beginning of test =================================
    if(enDebug)
      Serial.println(" glider.GC_BEGIN...");
      
    pinMode(PIN_solenoid, OUTPUT);
    pinMode(PIN_PS, OUTPUT);
    pinMode(PIN_pumpDir, OUTPUT);
    pinMode(PIN_pumpOnDIO, OUTPUT);
    pinMode(PIN_GO, INPUT);
  
    //analogReference(INTERNAL);
    
    Dynamixel.begin(dyna_Sbaud, PIN_dyna_rx, PIN_dyna_tx);
    Dynamixel.reset(dyna_id);
    
    psOn();
  }
  if(cmd == GC_RESET) { // reset to trimming position ==========================================
    if(enDebug)
      Serial.print(" glider.GC_RESET (please wait)...");
      
    Dynamixel.moveSpeed(dyna_id, param.pitch_pos_mid, param.pitch_vel);
    
    #ifdef DRY_MODE
      pump(0);
    #else
      pump(1); // removes histeresis
      while(getFiltAnalog(PIN_DWSensor) < param.DWSensorCenterPosition);
      pump(-1);
      while(getFiltAnalog(PIN_DWSensor) > param.DWSensorCenterPosition);
      pump(0);
    #endif
    
    enGlider = 0;
    
    if(enDebug)
      Serial.println(" done.");
  }
  
  if(cmd == GC_STOP) { // stop pump and stay =====================================================
    if(enDebug)
      Serial.println(" glider.GC_STOP...");
    pump(0);
  }
  
  if(cmd == GC_START) { // begin gliding cycle ===================================================
    if(enDebug)
      Serial.println(" glider.GC_START...");
    cycleCount = 0;
    t0 = millis();
    nLog = 0;
    state = ME_GLIDE_DOWN;
    entry = 1;
  }
  
  if(cmd == GC_NULL) { // continue the normal machine state run ==================================
    
    switch(state) { // select current state
        
      case ME_GLIDE_DOWN: // permanent of gliding down ---------------------------------------------
        if(entry) { // entry
          if(enDebug)
            Serial.println(" glider.ME_GLIDE_DOWN...");
          Dynamixel.moveSpeed(dyna_id, param.pitch_pos_down, param.pitch_vel);
          pump(1);
          pitchPID(0.0, 0, 0); // reset pid states
          entry = 0;
        }
        
        // during
        if(param.enPID) {
          pitchPID(-param.sp-(-um7.roll), &pPos, &pVel);
          Dynamixel.moveSpeed(dyna_id, (int) pPos, param.pitch_vel);
        }
        if(getFiltAnalog(PIN_DWSensor) > param.DWSensorBackPosition)
          pump(0);
        if(millis() > t0 + nLog*1000) {
          dataLog[nLog].pressure = getFiltAnalog(PIN_pressureSensorPin);
          dataLog[nLog].pitch = -um7.roll; // make sure it doesn't overflow
          nLog++;
          if(nLog >= NLOG) nLog = 0;
        }
        
        // exit condition
        if(millis() > t0 + (cycleCount+1)*param.descentTime + cycleCount*param.riseTime) { 
          entry = 1;
          state = ME_GLIDE_UP;
        }
        break;
        
      case ME_GLIDE_UP: // permanent of gliding down ---------------------------------------------------
        if(entry) { // entry
          if(enDebug)
            Serial.println(" glider.ME_GLIDE_UP...");
          Dynamixel.moveSpeed(dyna_id, param.pitch_pos_up, param.pitch_vel);
          pump(-1);
          pitchPID(0.0, 0, 0); // reset pid states
          entry = 0;
        }
        
        // during
        if(param.enPID) {
          pitchPID(param.sp-(-um7.roll), &pPos, &pVel);
          Dynamixel.moveSpeed(dyna_id, (int) pPos, param.pitch_vel);
        }
        if(getFiltAnalog(PIN_DWSensor) < param.DWSensorFrontPosition)
          pump(0);
        if(millis() > t0 + nLog*1000) {
          dataLog[nLog].pressure = getFiltAnalog(PIN_pressureSensorPin);
          dataLog[nLog].pitch = -um7.roll; // make sure it doesn't overflow
          nLog++;
          if(nLog >= NLOG) nLog = 0;
        }
        
        // exit condition
        if(millis() > t0 + (cycleCount+1)*param.descentTime + (cycleCount+1)*param.riseTime) { 
          entry = 1;
          cycleCount++;
          if(cycleCount >= 3)            
            state = ME_TRANSCEIVE;
          else
            state = ME_GLIDE_DOWN;
        }
        break;
        
      case ME_TRANSCEIVE:
        if(entry) { // entry
          if(enDebug)
            Serial.println(" glider.ME_TRANSCEIVE...");
          Dynamixel.moveSpeed(dyna_id, param.pitch_pos_mid, param.pitch_vel);
          for(int n = 0; n < NLOG; n++) {
            Serial.print(dataLog[n].pitch); Serial.print('\t');
            Serial.print(dataLog[n].pressure); Serial.print('\n');
            dataLog[n].pitch = 0.0;
            dataLog[n].pressure = 0.0;
            delay(10);
          }
          Serial.println();
          entry = 0;
        }
        
        // during
        if(getFiltAnalog(PIN_DWSensor) < param.DWSensorFrontPosition)
          pump(0);
        
        // exit condition
        if(millis() > t0 + 3*(param.descentTime+param.riseTime) + param.transTime) {
          t0 = millis();
          nLog = 0;
          cycleCount = 0;
          
          entry = 1;
          state = ME_GLIDE_DOWN;
        }
        break;
    }
    //Serial.print("("); Serial.print(pPos); Serial.print(", "); Serial.print(pVel); Serial.print(") ");
  }
}

void pitchPID(float err, float* out, float* outd) {
  
  static float err1 = 0.0;
  static float out1 = 0.0;
  static float sum = 0.0;

  const float dt = ((float)dtGlider) * 1e-3;
  
  if(out == 0) { // if null pointer, reset internal states
    err1 = 0.0;
    out1 = 0.0;
    sum = 0.0;
    return;
  }
  
  *out = param.kp*err + param.ki*sum + param.kd*(err-err1)/dt + param.pitch_pos_mid;
  if     (*out <   1) *out =   1; // lower saturation
  else if(*out > 550) *out = 550; // upper saturation
  
  if(outd != 0) { // for soft movement
    *outd = abs((*out-out1)/dt);
    if     (*outd > 1000) *outd = 1000; // upper saturation
    //*outd = 200;
  }

  err1 = err; // last input value for derivative calculation
  out1 = *out;
  sum = sum + err*dt; // integral
}

void pump(int flow) {
  
  #ifdef DRY_MODE
    flow = 0;
  #endif
  
  switch(flow) {
    case -1: // turn off and pump out
      pumpOff();
      delay(100);
      solOff();
      
      pumpOut();
      solOn();
      pumpOn();
      break;
      
    case 0: // just turn off
      pumpOff();
      delay(100);
      solOff();
      break;
      
    case 1: // turn off and pump in
      pumpOff();
      delay(100);
      solOff();
      
      pumpIn();
      solOn();
      pumpOn();
      break;
  }
}
  

// Equivalent to Serial.readBytesUntil(), except it trashes the buffer if timeout
// Use for chipkit or for softSerial
int ReadBytesUntil(char term, char* buff, int len)
{
  unsigned long int t0, timeout = 100;
  int n = 0;
  
  t0 = millis();
  while(millis() - t0 < timeout)
  {
    if(Serial.available())
    {
      buff[n] = Serial.read();
      if(buff[n] == term)
        return n;
      n++;
    }
  }
  return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*
solOn(); Turns the solenoid on
 solOff(); Turns the solenoid off
 psOn(); Turns P5V on
 psOff(); Turns P5V off
 pumpIn(); 
 
 */
void solOn()
{
  // Turn solenoid on
  digitalWrite(PIN_solenoid, HIGH);
}
void solOff()
{
  // Turn solenoid off
  digitalWrite(PIN_solenoid, LOW);
}
void psOn()
{
  // Turn P5V on
  digitalWrite(PIN_PS, HIGH);
}
void psOff()
{
  // Turn P5V off
  digitalWrite(PIN_PS, LOW);
}
void pumpIn()
{
  // Set pump to pull water in
  digitalWrite(PIN_pumpDir, LOW);
}
void pumpOut()
{
  // Set pump to push water out
  digitalWrite(PIN_pumpDir, HIGH);
}
void pumpOn()
{
  digitalWrite(PIN_pumpOnDIO, HIGH);
}
void pumpOff()
{
  digitalWrite(PIN_pumpOnDIO, LOW);
}
int getFiltAnalog(int APIN)
{
  int val = 0;
  for(int a=0; a<10; a++) {
    val = val + analogRead(APIN);
  }
  val = val/10;
  return val;
}
