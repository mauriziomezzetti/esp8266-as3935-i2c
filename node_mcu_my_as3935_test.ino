/*

Copyright (c) 2017, Embedded Adventures
All rights reserved.

Contact us at source [at] embeddedadventures.com
www.embeddedadventures.com

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

- Neither the name of Embedded Adventures nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.

*/

// AS3935 MOD-1016 Lightning Sensor Arduino test sketch
// Written originally by Embedded Adventures


#include <Wire.h>
#include "AS3935.h"

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#define IRQ_PIN 14 // d5
//spi
//#define CS_PIN 10
ICACHE_RAM_ATTR void alert(); 
volatile bool detected = false;
long azzera=300; //dopo 300 secondi azzera noise floor
long ini_az,fin_az,ini_st,fin_st;
long scrive=30;  // ogni 30 secondi scrive 
byte aa;

void printBinary(byte inByte)
{
  for (int b = 7; b >= 0; b--)
  {
    Serial.print(bitRead(inByte, b));
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  

  //I2C
  //sda=d2 =4   scl=d1=5
  Wire.begin(4,5); //sda d2 4 scl d1 5
  Serial.println("init ");
  mod1016.init(IRQ_PIN);
  //SPI
  //SPI.begin();
  //mod1016.init(IRQ_PIN, CS_PIN);


 Serial.println("reset original value");
  mod1016.resetoriginal();


  
  
  
  //void powerdown();
  Serial.println("powerup ");
  mod1016.powerup();
 
  
 
//Tune Caps, Set AFE, Set Noise Floor
Serial.println("set auto tune ");
autoTuneCaps(IRQ_PIN, 0);
//mod1016.setTuneCaps(7);
  
 
 



Serial.println("reset original value");
  mod1016.resetoriginal();

/*
//controlla scritture reg 0 

  Serial.println(" read register 0x00 ");
  aa=mod1016.readRegister(0x00,0xFF);
  printBinary(aa);
  Serial.println("00100100 reg 0x00 default");
  Serial.println("set OUTDOOR ");
  mod1016.setOutdoors(); 
  Serial.println(" read register 0x00 ");
  aa=mod1016.readRegister(0x00,0xFF);
  printBinary(aa);
  Serial.println("00011100 OUTDOOR CORRECT");
  Serial.println("set INDOOR ");
  mod1016.setIndoors(); 
  Serial.println(" read register 0x00");
  aa=mod1016.readRegister(0x00,0xFF);
  printBinary(aa);
  Serial.println("00100100 INDOOR CORRETTA");
  Serial.println("powerup ");
  mod1016.powerup();
  Serial.println(" read register 0x00");
  aa=mod1016.readRegister(0x00,0xFF);
  printBinary(aa);
  Serial.println("00100100 power on corretta");
  Serial.println("powerdown ");
  mod1016.powerdown();
  Serial.println(" read register 0x00");
  aa=mod1016.readRegister(0x00,0xFF);
  printBinary(aa);
  Serial.println("00100101 power down corretta");
  Serial.println("powerup ");
  mod1016.powerup();
  Serial.println(" read register 0x00");
  aa=mod1016.readRegister(0x00,0xFF);
  printBinary(aa);
  Serial.println("00100100 power on corretta");
  Serial.println("set OUTDOOR ");
  mod1016.setOutdoors(); 
  Serial.println(" read register 0x00 ");
  aa=mod1016.readRegister(0x00,0xFF);
  printBinary(aa);
  Serial.println("00011100 OUTDOOR CORRECT");
 
//controlla scritture reg 1 

 
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("00100010 register 0x01 corretto");
  
 Serial.print("get noise floor ");
 Serial.println(mod1016.getNoiseFloor(), HEX);
 Serial.println("set noise floor 0");
 mod1016.setNoiseFloor(0);  // 0 to 7
 Serial.print("get noise floor ");
 Serial.println(mod1016.getNoiseFloor(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("00000010 register 0x01 corretto");
 Serial.println("set noise floor 1");
 mod1016.setNoiseFloor(1);  // 0 to 7
 Serial.print("get noise floor ");
 Serial.println(mod1016.getNoiseFloor(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("00010010 register 0x01 corretto");

 Serial.println("set noise floor 2");
 mod1016.setNoiseFloor(2);  // 0 to 7
 Serial.print("get noise floor ");
 Serial.println(mod1016.getNoiseFloor(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("00100010 register 0x01 corretto");

 Serial.println("set noise floor 3");
 mod1016.setNoiseFloor(3);  // 0 to 7
 Serial.print("get noise floor ");
 Serial.println(mod1016.getNoiseFloor(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("00110010 register 0x01 corretto");

 Serial.println("set noise floor 4");
 mod1016.setNoiseFloor(4);  // 0 to 7
 Serial.print("get noise floor ");
 Serial.println(mod1016.getNoiseFloor(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("01000010 register 0x01 corretto");

 Serial.println("set noise floor 5");
 mod1016.setNoiseFloor(5);  // 0 to 7
 Serial.print("get noise floor ");
 Serial.println(mod1016.getNoiseFloor(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("01010010 register 0x01 corretto");
 
 Serial.println("set noise floor 6");
 mod1016.setNoiseFloor(6);  // 0 to 7
 Serial.print("get noise floor ");
 Serial.println(mod1016.getNoiseFloor(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("01100010 register 0x01 corretto");

 Serial.println("set noise floor 7");
 mod1016.setNoiseFloor(7);  // 0 to 7
 Serial.print("get noise floor ");
 Serial.println(mod1016.getNoiseFloor(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("01110010 register 0x01 corretto");


 Serial.print("get wdog ");
 Serial.println(mod1016.getWatchdogThreshold(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("01110010 register 0x01 corretto");

 
 Serial.println("set dog 1");
 mod1016.setWatchdogThreshold(1);  // 0 to 15
 Serial.print("get dog ");
 Serial.println(mod1016.getWatchdogThreshold(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("01110001 register 0x01 corretto");
 
 Serial.println("set dog 4");
 mod1016.setWatchdogThreshold(4);  // 0 to 15
 Serial.print("get dog ");
 Serial.println(mod1016.getWatchdogThreshold(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("01110100 register 0x01 corretto");

 Serial.println("set dog 8");
 mod1016.setWatchdogThreshold(8);  // 0 to 15
 Serial.print("get dog ");
 Serial.println(mod1016.getWatchdogThreshold(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("01111000 register 0x01 corretto");
 
 Serial.println("set dog 15");
 mod1016.setWatchdogThreshold(15);  // 0 to 15
 Serial.print("get dog ");
 Serial.println(mod1016.getWatchdogThreshold(), HEX);
 Serial.println(" read register 0x01 ");
 aa=mod1016.readRegister(0x01,0xFF);
 printBinary(aa);
 Serial.println("01111111 register 0x01 corretto");



//controlla scritture reg 2 

 
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11000010 register 0x02 corretto");


 Serial.print("get min light ");
 Serial.println(mod1016.getMinimumLightnings(), HEX);
 Serial.println("set min light 1");
 mod1016.setMinimumLightnings(1);  // 0=1 1=5 2=9 3=15
 Serial.print("get min light ");
 Serial.println(mod1016.getMinimumLightnings(), HEX);
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11010010 register 0x02 corretto");
 
 Serial.println("set min light 0");
 mod1016.setMinimumLightnings(0);  // 0=1 1=5 2=9 3=15
 Serial.print("get min light ");
 Serial.println(mod1016.getMinimumLightnings(), HEX);
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11000010 register 0x02 corretto");
 
 Serial.println("set min light 2");
 mod1016.setMinimumLightnings(2);  // 0=1 1=5 2=9 3=15
 Serial.print("get min light ");
 Serial.println(mod1016.getMinimumLightnings(), HEX);
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11100010 register 0x02 corretto");

 Serial.println("set min light 3");
 mod1016.setMinimumLightnings(3);  // 0=1 1=5 2=9 3=15
 Serial.print("get min light ");
 Serial.println(mod1016.getMinimumLightnings(), HEX);
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11110010 register 0x02 corretto");
 
 
 
 Serial.print("get spike ");
 Serial.println(mod1016.getSpikeRejection(), HEX);
 Serial.println("set spike 1");
 mod1016.setSpikeRejection(1);  // 0 to 15
 Serial.print("get spike ");
 Serial.println(mod1016.getSpikeRejection(), HEX);
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11110001 register 0x02 corretto");
 Serial.println("set spike 3");
 mod1016.setSpikeRejection(3);  // 0 to 15
 Serial.print("get spike ");
 Serial.println(mod1016.getSpikeRejection(), HEX);
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11110011 register 0x02 corretto");
 Serial.println("set spike 8");
 mod1016.setSpikeRejection(8);  // 0 to 15
 Serial.print("get spike ");
 Serial.println(mod1016.getSpikeRejection(), HEX);
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11111000 register 0x02 corretto");
 Serial.println("set spike 15");
 mod1016.setSpikeRejection(15);  // 0 to 15
 Serial.print("get spike ");
 Serial.println(mod1016.getSpikeRejection(), HEX);
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11111111 register 0x02 corretto");
 

 Serial.println("clear stat ");
 mod1016.clearStats();
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11111111 register 0x02 corretto");


 
 Serial.println("set spike 0");
 mod1016.setSpikeRejection(0);  // 0 to 15
 Serial.println("set min light 0");
 mod1016.setMinimumLightnings(0);  // 0=1 1=5 2=9 3=15
 Serial.println("clear stat ");
 mod1016.clearStats();
 Serial.println(" read register 0x02 ");
 aa=mod1016.readRegister(0x02,0xFF);
 printBinary(aa);
 Serial.println("11000000 register 0x02 corretto");



//controlla scritture reg 3

 
 Serial.println(" read register 0x03 ");
 aa=mod1016.readRegister(0x03,0xFF);
 printBinary(aa);
 Serial.println("00000000 register 0x03 corretto (ultimi 4 bit irq)");
 Serial.println("disable (mask) disturner");
 mod1016.disableDisturbers();
 Serial.println("read mask disturner");
 Serial.println(mod1016.readmaskDisturbers(),HEX);
 Serial.println(" read register 0x03 ");
 aa=mod1016.readRegister(0x03,0xFF);
 printBinary(aa);
 Serial.println("00100000 register 0x03 corretto (ultimi 4 bit irq)");
 Serial.println("enable disturner");
 mod1016.enableDisturbers();
 Serial.println("read mask disturner");
 Serial.println(mod1016.readmaskDisturbers(),HEX);
 Serial.println(" read register 0x03 ");
 aa=mod1016.readRegister(0x03,0xFF);
 printBinary(aa);
 Serial.println("00000000 register 0x03 corretto (ultimi 4 bit irq)");
 


 
 //Serial.println("set auto tune ");
 //autoTuneCaps(IRQ_PIN);

 Serial.println(" read register 0x04 ");
 aa=mod1016.readRegister(0x04,0xFF);
 printBinary(aa);
 Serial.println(" read register 0x05 ");
 aa=mod1016.readRegister(0x05,0xFF);
 printBinary(aa);
 Serial.println(" read register 0x06 ");
 aa=mod1016.readRegister(0x06,0xFF);
 printBinary(aa);
 Serial.println(" read register 0x07 ");
 aa=mod1016.readRegister(0x07,0xFF);
 printBinary(aa);

 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);

 Serial.println(" performing autotune ");
 autoTuneCaps(IRQ_PIN, 0);
 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);



 Serial.println("set TRCO 1");
 mod1016.setTRCO(1);  // 0 to 1
 Serial.print("get TRCO ");
 Serial.println(mod1016.getTRCO(), HEX);
 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);
 Serial.println("00011111 register 0x08 corretto");

 Serial.println("set TRCO 0");
 mod1016.setTRCO(0);  // 0 to 1
 Serial.print("get TRCO ");
 Serial.println(mod1016.getTRCO(), HEX);
 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);
 Serial.println("00001111 register 0x08 corretto");




 Serial.println("set SRCO 1");
 mod1016.setSRCO(1);  // 0 to 1
 Serial.print("get SRCO ");
 Serial.println(mod1016.getSRCO(), HEX);
 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);
 Serial.println("00101111 register 0x08 corretto");

 Serial.println("set SRCO 0");
 mod1016.setSRCO(0);  // 0 to 1
 Serial.print("get TRCO ");
 Serial.println(mod1016.getSRCO(), HEX);
 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);
 Serial.println("00001111 register 0x08 corretto");

 Serial.println("set LCO 1");
 mod1016.setLCO(1);  // 0 to 1
 Serial.print("get LCO ");
 Serial.println(mod1016.getLCO(), HEX);
 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);
 Serial.println("01001111 register 0x08 corretto");

 Serial.println("set LCO 0");
 mod1016.setLCO(0);  // 0 to 1
 Serial.print("get LCO ");
 Serial.println(mod1016.getLCO(), HEX);
 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);
 Serial.println("00001111 register 0x08 corretto");
  




Serial.println("set TRCO 1");
 mod1016.setTRCO(1);  // 0 to 1
 Serial.print("get TRCO ");
 Serial.println(mod1016.getTRCO(), HEX);
 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);
 Serial.println("00011111 register 0x08 corretto");
  
  
 
 
 
 Serial.println("set SRCO 1");
 mod1016.setSRCO(1);  // 0 to 1
 Serial.print("get SRCO ");
 Serial.println(mod1016.getSRCO(), HEX);
 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);
 Serial.println("00111111 register 0x08 corretto");

 

 Serial.println("set LCO 1");
 mod1016.setLCO(1);  // 0 to 1
 Serial.print("get LCO ");
 Serial.println(mod1016.getLCO(), HEX);
 Serial.println(" read register 0x08 ");
 aa=mod1016.readRegister(0x08,0xFF);
 printBinary(aa);
 Serial.println("01111111 register 0x08 corretto");
 
 
 
  
 
 
 
 Serial.print("get noise floor ");
 Serial.println(mod1016.getNoiseFloor(), HEX);
 Serial.print("get min light ");
 Serial.println(mod1016.getMinimumLightnings(), HEX);
 Serial.print("get spike ");
 Serial.println(mod1016.getSpikeRejection(), HEX);
 Serial.print("get wdog ");
 Serial.println(mod1016.getWatchdogThreshold(), HEX);
  //mod1016.disableDisturbers();
 Serial.println("enable disturner");
 mod1016.enableDisturbers();
  Serial.println("set auto tune ");
  autoTuneCaps(IRQ_PIN);


  
  Serial.println("TUNE\tAFE IN/OUT\tNOISEFLOOR");
  Serial.print(mod1016.getTuneCaps(), HEX);
  Serial.print("\t");
  Serial.print(mod1016.getAFE(), BIN);
  Serial.print("\t");
  Serial.println(mod1016.getNoiseFloor(), HEX);
  Serial.print("\n");
*/

 
 






  Serial.println("reset original value");
  mod1016.resetoriginal();
  autoTuneCaps(IRQ_PIN, 0);
  
  
  Serial.println("TUNE\tAFE IN/OUT\tNOISEFLOOR");
  Serial.print(mod1016.getTuneCaps(), HEX);
  Serial.print("\t");
  Serial.print(mod1016.getAFE(), BIN);
  Serial.print("\t");
  Serial.println(mod1016.getNoiseFloor(), HEX);
  Serial.print("\n");
  
 



  pinMode(IRQ_PIN, INPUT);
  delay(500);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), alert, RISING);
  mod1016.getIRQ();
  Serial.print("after interrupt ");
  Serial.println(mod1016.getIRQ());
  //ini=millis()/1000;
}

void loop() {
  if (detected) {
    translateIRQ(mod1016.getIRQ());
    detected = false;
  }
}

void alert() {
  detected = true;
}

void translateIRQ(uns8 irq) {
  switch(irq) {
      case 1:
        Serial.println("NOISE TOO HIGHT DETECTED");
        break;
      case 4:
        Serial.println("DISTURBER DETECTED");
        break;
      case 8: 
        Serial.println("LIGHTNING DETECTED");
        printDistance();
        break;
    }
}

void printDistance() {
  int distance = mod1016.calculateDistance();
  unsigned int energy=mod1016.getIntensity();
  Serial.println("Lightning intensity ");
  Serial.println(energy);
  if (distance == -1)
    Serial.println("Lightning out of range");
  else if (distance == 1)
    Serial.println("Distance not in table");
  else if (distance == 0)
    Serial.println("Lightning overhead");
  else {
    Serial.print("Lightning ~");
    Serial.print(distance);
    Serial.println("km away\n");  
  }
}
