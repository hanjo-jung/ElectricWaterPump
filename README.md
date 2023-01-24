# ElectricWaterPump

A PWM controller for an electric water pump like Davies-Craig EWP-80.

Features:
- Reads water and oil temparatures and RPM from the CAN Bus.
- Reads an oil pressure sensor signal.
- Sends PWM signal to a motor driver, MD10C.
- Has an emergency switch to drive 100% PWM signal.

Recent pictures:
![image](https://user-images.githubusercontent.com/49624742/214237019-a1efb4db-29e4-42bd-bda2-30e244f3e774.png)


Pictures of the first installation:

![20210501_093318](https://user-images.githubusercontent.com/49624742/214234224-91158e3a-f01c-485d-9975-7dc66803fc86.jpg)
![20210430_234508](https://user-images.githubusercontent.com/49624742/214234273-9969fa10-3be6-4157-8167-ba295df3b0ec.jpg)
![20210501_152035](https://user-images.githubusercontent.com/49624742/214234614-122bd5d2-d726-4e90-89e4-5a5c48a62ddd.jpg)
![20210518_162604](https://user-images.githubusercontent.com/49624742/214234434-791b2ea9-6a26-42f9-9b80-dc839cb38725.jpg)

The first run:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Gr-7CdJrsGw/0.jpg)](https://www.youtube.com/watch?v=Gr-7CdJrsGw)

Used materials:
- Arduino with CAN interface: http://docs.longan-labs.cc/1030008/
- Motor driver: https://tutorial.cytron.io/2013/07/29/controlling-md10c-with-arduino/
- LCD: https://wiki.seeedstudio.com/Grove-16x2_LCD_Series/
- Oil pressure sensor: https://prosportgauges.com/products/premium-oil-fuel-pressure-sender

Reference links:
- https://www.ft86club.com/forums/showthread.php?t=90692
- https://github.com/wiretap/brz_cangateway/blob/master/cangateway.ino
- https://github.com/timurrrr/RaceChronoDiyBleDevice
- https://github.com/iDoka/awesome-automotive-can-id
