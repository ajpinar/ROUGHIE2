#include <SoftwareSerial.h>
#include <UM7_BIN.h>

UM7_BIN um7;
SoftwareSerial ss(10, 11);

void setup() {
  Serial.begin(115200);
  um7.begin(&ss);
}

void loop() {
  int ret = um7.refresh();
  
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
  
