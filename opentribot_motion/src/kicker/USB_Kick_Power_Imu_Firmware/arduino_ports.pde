// .......................................................................
// Code for reading data from a Wii Motion plus device connected to a Nunchuck
// Links to other sites
// http://www.windmeadow.com/node/42   .... first time i saw arduino nunchuk interface
// http://www.kako.com/neta/2009-017/2009-017.html# .....Japanese site with lots of Wii info
// http://wiibrew.org/wiki/Wiimote/Extension_Controllers#Wii_Motion_Plus    .... the one and only
// http://randomhacksofboredom.blogspot.com/2009/06/wii-motion-plus-arduino-love.html
// ....original motion plus but not passthrough
// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1248889032/35   .... great Kalman filter code
// thanks to duckhead and knuckles904. I will be using that code for sure.
// http://news.jeelabs.org/2009/02/14/ports-library-for-arduino/   ..... This is the ports library
// http://obex.parallax.com/objects/471/   .... ideas for passthrough
// by Krulkip
// Hardware Arduino with ATMega 168
// Connections SCA to PD4 (Digital4) and SCL to AD0 (Analog0)
// uses ports from deelabs see above. I think Wire should be OK with some adaptation.
//........................................................................
#include <string.h>
#include "Ports.h"
#include <stdio.h>
uint8_t outbuf[6];		// array to store arduino output
int cnt = 0;
int ledPin = 13;
int xID;
int loopcounter=0;

PortI2C one (1, PortI2C::KHZ100);
DeviceI2C nunchuk (one, 0x52);
DeviceI2C wmplus (one, 0x53);
void
setup ()
{
  Serial.begin (115200);
  Serial.print ("Finished setup\n");
  Serial.print ("Now detecting WM+\n");
  delay(100);
// reset wm+ by means of digital io only then will we get right xID
  wmplus.send();
  wmplus.write(0xFA);
  wmplus.stop();
  delay (100);
  wmplus.receive ();delay(2);
  cnt = wmplus.read(0);xID = cnt;Serial.print(cnt,HEX);Serial.print(" ");
  cnt = wmplus.read(0);xID += cnt;Serial.print(cnt,HEX);Serial.print(" ");
  cnt = wmplus.read(0);xID += cnt;Serial.print(cnt,HEX);Serial.print(" ");
  cnt = wmplus.read(0);xID += cnt;Serial.print(cnt,HEX);Serial.print(" ");
  cnt = wmplus.read(0);xID += cnt;Serial.print(cnt,HEX);Serial.print(" ");
  cnt = wmplus.read(1);xID += cnt;Serial.print(cnt,HEX);Serial.print(" ");
   Serial.print("Extension controller xID = 0x");
   Serial.print(xID,HEX);
   if (xID == 0xCB) { Serial.print (" = wmplus connected but not activated"); }
   if (xID == 0x5FA) { Serial.print (" = wmplus already activated ? check later"); }
   Serial.print ("\r\n");
// now make Wii Motion plus the active extension
// nunchuk mode = 05, wm+ only = 04, classic controller = 07
   Serial.print ("Making Wii Motion plus the active extension in nunchuk mode = 05 ........");
   wmplus.send();
   wmplus.write(0xFE);
   wmplus.write(0x05);// nunchuk
   wmplus.stop();
   Serial.print (" OK done");
   Serial.print ("\r\n");
   delay (100);
// now innitiate Wii Motion plus
    Serial.print ("Innitialising Wii Motion plus ........");
    wmplus.send();
    wmplus.write(0xF0);
    wmplus.write(0x55);
    wmplus.stop();
    Serial.print (" OK done");
    Serial.print ("\r\n");
    delay (100);
    Serial.print ("Set reading address at 0xFA .......");
    nunchuk.send();
    nunchuk.write(0xFA);
    nunchuk.stop();
   Serial.print(" OK done");
   Serial.print ("\r\n");
   delay (100);
   nunchuk.receive ();
   outbuf[0] = nunchuk.read(0);Serial.print(outbuf[0],HEX);Serial.print(" ");
   outbuf[1] = nunchuk.read(0);Serial.print(outbuf[1],HEX);Serial.print(" ");
   outbuf[2] = nunchuk.read(0);Serial.print(outbuf[2],HEX);Serial.print(" ");
   outbuf[3] = nunchuk.read(0);Serial.print(outbuf[3],HEX);Serial.print(" ");
   outbuf[4] = nunchuk.read(0);Serial.print(outbuf[4],HEX);Serial.print(" ");
   outbuf[5] = nunchuk.read(1);Serial.print(outbuf[5],HEX);Serial.print(" ");
   Serial.print ("\r\n");
   xID= outbuf[0] + outbuf[1] + outbuf[2] + outbuf[3] + outbuf[4] + outbuf[5];
   Serial.print("Extension controller xID = 0x");
   Serial.print(xID,HEX);
   if (xID == 0xCB) { Serial.print (" Wii Motion plus connected but not activated"); }
   if (xID == 0xCE) { Serial.print (" Wii Motion plus connected and activated"); }
   if (xID == 0x5FA) { Serial.print (" Wii Motion plus not connected"); }
   Serial.print ("\r\n");
   delay (100);
   // Now we want to point the read adress to 0xa40008 where the 6 byte data is stored
    Serial.print ("Set reading address at 0x08 .........");
    nunchuk.send();
    nunchuk.write(0x08);
    nunchuk.stop();
    Serial.print(" OK done");
    Serial.print ("\r\n");
}
void
send_zero ()
{
    nunchuk.send();
    nunchuk.write(0x00);
    nunchuk.stop();
}
void loop ()
{
// now follows conversion command instructing extension controller to get
// all sensor data and put them into 6 byte register within the extension controller
  send_zero (); // send the request for next bytes
  nunchuk.receive ();
  outbuf[0] = nunchuk.read(0);
  outbuf[1] = nunchuk.read(0);
  outbuf[2] = nunchuk.read(0);
  outbuf[3] = nunchuk.read(0);
  outbuf[4] = nunchuk.read(0);
  outbuf[5] = nunchuk.read(1);
  print ();
  cnt = 0;
  //delay (10000);
}
void
print ()
{
// check if nunchuk is really connected
  if ((outbuf[4]&0x01)==0x01) {

  }
if ((outbuf[5]&0x03)==0x00) {
 int joy_x_axis = outbuf[0];
 int joy_y_axis = outbuf[1];
 int accel_x_axis = (outbuf[2] << 2) + ((outbuf[5] >> 3) & 2);
 int accel_y_axis = (outbuf[3] << 2) + ((outbuf[5] >> 4) & 2);
 int accel_z_axis = (outbuf[4] << 2) + ((outbuf[5] >> 5) & 6);
 int z_button = (outbuf[5]>>2) & 1;
 int c_button = (outbuf[5]>>3) & 1;
  Serial.print("\r\nDATA ");
  Serial.print (" jx ");
  Serial.print (joy_x_axis, DEC);
  Serial.print (" jy ");
  Serial.print (joy_y_axis, DEC);
  Serial.print (" ax ");
  Serial.print (accel_x_axis, DEC);
  Serial.print (" ay ");
  Serial.print (accel_y_axis, DEC);
  Serial.print (" az ");
  Serial.print (accel_z_axis, DEC);

  if (z_button == 0) { Serial.print (" zb"); }
  if (c_button == 0) { Serial.print (" cb"); }
  Serial.print (" ");
 }
else
if  ((outbuf[5]&0x03)==0x02) {
  int yaw = (((outbuf[5]&0xFC)<<6) + outbuf[0]);
  int pitch = (((outbuf[4]&0xFC)<<6) + outbuf[1]);
  int roll = (((outbuf[3]&0xFC)<<6) + outbuf[2]);
  Serial.print (" y ");
  Serial.print (yaw, DEC);
  Serial.print (" p ");
  Serial.print (pitch, DEC);
  Serial.print (" r ");
  Serial.print (roll, DEC);
  Serial.print (" ");
}
}
 


