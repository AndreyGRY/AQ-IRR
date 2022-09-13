# AQ-IRR
AVR(AtTiny45)-based IR-controlled USB HID Keyboard

Instruction and detailed explanation you can get on https://drive.google.com/file/d/1qV_x94ru8l6GEPR-HmLSDU18ONs_s7jf/view?usp=sharing (in Russian).
In general: 

This thing can accept the remote control signals, recode and transmit it as standard USB Keyboard. Powering/Sleeping implemented through the Power Controls.

You need a simple hand-made or aliexpress device with AVR ATTiny45 (may 85) and IR-receiver (schemes are able on GyverLabs git or analog ones) & usual remote controller with NEC-protocol (project adapted for special one, but you can parse your RC with commented IR-decoder in main).
Also you need an ISP-programmer (optimal may be Arduino as ISP, flash process is described in pdf linked above).

You can freely use/change/save this code. I'd be happy to answer on andregrygorie@gmail.com .
