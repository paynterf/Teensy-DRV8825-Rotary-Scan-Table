// Written by Nick Gammon
// May 2012

#include <Arduino.h>
#include <Wire.h>
//#include "SBWire.H"

template <typename T> unsigned int I2C_writeAnything (const T& value)
  {
    const byte * p = (const byte*) &value;
    unsigned int i;
    for (i = 0; i < sizeof value; i++)
          Wire.write(*p++);

////DEBUG!!
//    for (i = 0; i < sizeof value; i++)
//    {
//        Serial.print(": I2C_writeAnything() sent "); Serial.println(*p);
//        Wire.write(*p++);
//    }
////DEBUG!!

    return i;
  }  // end of I2C_writeAnything

template <typename T> unsigned int I2C_readAnything(T& value)
  {
////DEBUG!!
    //Serial.print("I2C_readAnything: ");
    //Serial.print("Sizeof(value) = "); Serial.println(sizeof(value));
////DEBUG!!

    byte * p = (byte*) &value;
    unsigned int i;
    for (i = 0; i < sizeof value; i++)
          *p++ = Wire.read();

////DEBUG!!
//    for (i = 0; i < sizeof value; i++)
//    {
//          *p = Wire.read();
//          Serial.print("i = "); Serial.print(i);  
//          Serial.print(": I2C_readAnything() received "); Serial.println(*p);
//          p++;
//    }
////DEBUG!!

    return i;
  }  // end of I2C_readAnything


