# esp8266-as3935-i2c
use i2c  icache_ram_ for irq
include AS3935.h and AS3935.cpp with all function
use ICACHE_RAM_ATTR with irq to avoid ISR not in ram error with latest esp8266 core 

WEMOS D1 mini
SDA -> D2
SCL -> D1
IRQ -> D5

POWER 3.3 v !!!!  5.V ONLY IF EN_V -> 5v 
