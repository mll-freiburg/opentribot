#ifndef _I2CMASTER_CONFIG_H
#define _I2CMASTER_CONFIG_H   

#include <avr/io.h>

/** Adapt these SCA and SCL port and pin definition to your target !! */
#define SDA     	4		// SDA Port D, Pin 4   
#define SCL		5		// SCL Port D, Pin 5
#define SDA_PORT        PORTD           // SDA Port D
#define SCL_PORT        PORTD           // SCL Port D         



#endif /*[_I2CMASTER_CONFIG_H]*/
