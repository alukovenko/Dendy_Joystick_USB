# Dendy Joystick with USB HID interface

Dendy was a Famiclone (a cartridge-compatible knock-off of an 8-bit NES Famicom) produced in 1990s for ex-USSR market. For many, it has a unique tactile feeling, which is hard to replace. I was unable to source any similarly-looking controllers with a USB HID interface, and I ended up getting one with the original DB-9 (serial) interface which is obviously not compatible with modern PCs and emulators.

There is already a number of projects that build USB HID interfaces for controllers. After trying out [FreeJoy](https://github.com/FreeJoy-Team/FreeJoy) I figured it is not currently possible to configure "Turbo A/B" buttons for rapid fire function, where these buttons should simulate pressing A/B buttons at a high speed (different sources claim 8 Hz, 15 Hz and 30 Hz, in this implementation I am going along with 15 Hz). This implementation builds onto [libopencm3](https://github.com/libopencm3/libopencm3) library instead.

## What you will need

- Soldering iron, basic soldering skills
- STM32 Bluepill (normally comes at $3 price range)
- ST-Link v2 flash programmer (alternatively, you can use a serial adapter with bootloader, but it is not in-scope of this guide, and you are on your own)
- Joystick (gamepad) itself, doesn't have to fully work, because out of all the original internals we will only use the switches and the main PCB
- Some fine wires
- Micro-USB cable

## Operations

1. Hardware modifications - fix the USB flaw

Many, if not most BluePills come with incorrect R10 resistor which causes a variety of USB enumeration issues. R10 should be 1.5 KOhm, and if it measures 4.8 KOhm or 10 KOhm - you'd need to replace. Alternatively, if you only have 1.8K resistor - solder it in parallel with 10K to get `(10*1.8)/(10+1.8)=1.525` KOhm, which is good enough.

In my case I didn't have a proper SMD size, and I ended up with this workable solution:

TODO: add photo

2. Connect ST-Link to the BluePill's 4-pin SWD header. The connections are as follows :

   ST-Link SWCLK -> BluePill SWCLK (PA14)
   ST-Link SWDIO -> BluePill SWDIO (PA13)
   ST-Link GND -> BluePill G (GND)
   ST-Link 3.3V -> BluePill 3.3

3. Make sure BluePill is responsive:

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

Bluepills come with 64/128/256 KB flashes. This firmware is rather small, and will nicely fit into 64 KB.

4. `make`

5. Connect Bluepill to the joystick. TA and TB buttons on my joystick are abnormal - the rest of the buttons are wired to GND and original chip pin, whereas TA/TB are connected both sides to the chip, which is not what we need here. Desolder the old chip, modify wiring on TA and TB buttons to follow the usual practice with GND. I am following this wiring, with 11 wires coming to the PCB:

| Connection | Pin | Description        |
| ---------- | --- | ------------------ |
| DPad Up    | PA0 | BUTTON_UP_PIN      |
| DPad Down  | PA1 | BUTTON_DOWN_PIN    |
| DPad Left  | PA2 | BUTTON_LEFT_PIN    |
| DPad Right | PA3 | BUTTON_RIGHT_PIN   |
| Select     | PA4 | BUTTON_SELECT_PIN  |
| Start      | PA5 | BUTTON_START_PIN   |
| Button A   | PA6 | BUTTON_A_PIN       |
| Button B   | PA7 | BUTTON_B_PIN       |
| Turbo A    | PB0 | BUTTON_TURBO_A_PIN |
| Turbo B    | PB1 | BUTTON_TURBO_B_PIN |
| Ground     | GND | BluePill GND       |

## TODO

Currently using VID/PID from pid.codes for test device, get a real PID/VID
