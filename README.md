# Dendy Joystick with USB HID interface

There is already a number of projects that build USB HID interfaces for controllers. After trying out [FreeJoy](https://github.com/FreeJoy-Team/FreeJoy) I figured it is not currently possible to configure "Turbo A/B" buttons for rapid fire function. This implementation builds onto [libopencm3](https://github.com/libopencm3/libopencm3) library instead.

## TODO

how to clone
pid.codes, get a real PID/VID

## Operations

1. Connect the ST-Link: Connect the ST-Link V2 to the BluePill's 4-pin SWD header. The connections are as follows :

   ST-Link SWCLK -> BluePill SWCLK (PA14)
   ST-Link SWDIO -> BluePill SWDIO (PA13)
   ST-Link GND -> BluePill G (GND)
   ST-Link 3.3V -> BluePill 3.3

2. Make sure BluePill is responsive:

```
$ st-info --probe
Found 1 stlink programmers
  version:    V2J29S7
  serial:     7E82060132124647524B4E00
  flash:      131072 (pagesize: 1024)
  sram:       20480
  chipid:     0x410
  dev-type:   STM32F1xx_MD
```

3. Hardware modifications - fix the USB flaw

Replace R10 resistor with 1.5 KOhm, because 10 KOhm is a design issue. Alternatively, if you only have 1.8K resistor - solder it in parallel with 10K to get `(10*1.8)/(10+1.8)=1.525` K.

TODO: add photo

4. Connect Bluepill to the joystick. TA and TB buttons on my joystick weren't wired to GND, which is not what we need here. Desolder the old chip, modify wiring on TA and TB buttons to follow the usual practice with GND. I am following this wiring:

```
Joystick Function	Bluepill Pin	Firmware #define
DPad Up	            PA0	            BUTTON_UP_PIN
DPad Down	        PA1	            BUTTON_DOWN_PIN
DPad Left	        PA2	            BUTTON_LEFT_PIN
DPad Right	        PA3	            BUTTON_RIGHT_PIN
Select	            PA4	            BUTTON_SELECT_PIN
Start	            PA5	            BUTTON_START_PIN
Button A	        PA6	            BUTTON_A_PIN
Button B	        PA7	            BUTTON_B_PIN
Turbo A	            PB0	            BUTTON_TURBO_A_PIN
Turbo B	            PB1	            BUTTON_TURBO_B_PIN

COMMON GROUND G GND Connect to any GND pin on BluePill
```

5. ...

6. Flashing

Download old firmware. If `st-info --probe` reported 64 KB then use

```
# For a 64KB flash chip (65536 bytes = 0x10000 hex)
st-flash read original_firmware.bin 0x08000000 0x10000
```

For 128KB use

```
st-flash read original_firmware.bin 0x08000000 0x20000
```

Erase the old firmware

```
st-flash erase
```

Flash FreeJoy. Do _NOT_ attempt to flash .bin file - those are in some weird format for live updates. Use .hex:

```
st-flash --format ihex write FreeJoy_v1_7_1b3.hex
```

Reset the device

```
st-flash reset
```

7. Download FreeJoy configurator https://github.com/FreeJoy-Team/FreeJoyConfiguratorQt/releases
