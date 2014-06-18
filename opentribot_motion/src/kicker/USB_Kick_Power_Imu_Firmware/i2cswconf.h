#ifndef I2CSWCONF_H
#define I2CSWCONF_H
 
 // clock line port
 #define SCLPORT PORTD   // i2c clock port
 #define SCLDDR  DDRD    // i2c clock port direction
 // data line port
 #define SDAPORT PORTD   // i2c data port
 #define SDADDR  DDRD    // i2c data port direction
 #define SDAPIN  PIND    // i2c data port input
 // pin assignments
#define SCL     PD5     // i2c clock pin
#define SDA     PD4     // i2c data pin
 
#endif

