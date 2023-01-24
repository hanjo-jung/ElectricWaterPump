# ElectricWaterPump

A PWM controller for an electric water pump like Davies-Craig EWP-80.
Also it has features that read water and oil temparatures and RPM from the CAN Bus, and read an oil pressure sensor signal which varies from 0 to 5 volts.

A video link:
https://youtu.be/Gr-7CdJrsGw

Used materials:
- Arduino with CAN interface: http://docs.longan-labs.cc/1030008/
- Motor driver: https://tutorial.cytron.io/2013/07/29/controlling-md10c-with-arduino/
- LCD: https://wiki.seeedstudio.com/Grove-16x2_LCD_Series/
- Oil pressure sensor: https://prosportgauges.com/products/premium-oil-fuel-pressure-sender

Reference links:
- https://github.com/wiretap/brz_cangateway/blob/master/cangateway.ino
- https://github.com/timurrrr/RaceChronoDiyBleDevice
- https://github.com/iDoka/awesome-automotive-can-id
