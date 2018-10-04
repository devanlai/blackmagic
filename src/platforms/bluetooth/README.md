# Blackmagic STM32F103 Minimum System Development Board over Bluetooth serial

This is an experimental platform using a Bluepill board and a bluetooth serial-port-profile module.

## External connections:

|  Function   | PIN   | BLUEPILL    |
| ----------- | ----- | ----------- |
|  JTMS/SWDIO |  PA13 |  P2/2       |
|  JTCK/SWCLK |  PA14 |  P2/3       |
|  JTDI       |  PA15 |  P4/11 (38) |
|  JTDO       |  PB3  |  P4/10 (39) |
|  SRST       |  PB4  |  P4/9  (40) |
|  UART1_TX   |  PB6  |  P4/7  (42) |
|  UART1_RX   |  PB7  |  P4/6  (43) |
|  SWO/RX2    |  PA3  |  P3/7  (13) |


### Force Bootloader Entry:
    Bluepill: Jumper Boot1 to '1' to read PB2 high.

### References:
[Blue Pill Schematics 1
    ](https://jeelabs.org/img/2016/STM32F103C8T6-DEV-BOARD-SCH.pdf) :
    Use first number!

[Blue Pill Schematics 2
    ](https://wiki.stm32duino.com/images/a/ae/Bluepillpinout.gif) :
    Use second number!

## STM32F103 Minimum System Development Board (aka Blue Pill)

This board has the SWD pins of the onboard F103 accessible on one side.
Reuse these pins. There are also jumpers for BOOT0 and BOOT1(PB2). Reuse
Boot1 as "Force Bootloader entry" jumpered high when booting. Boot1
has 100 k Ohm between MCU and header pin and can not be used as output.

All other port pins are have header access with headers not yet soldered.

This platform can be used for any STM32F103x[8|B] board when JTAG/SWD are
accessible, with the LED depending on actual board layout routed to some
wrong pin and force boot not working.

## Other STM32F103x[8|B] boards
If the needed JTAG connections are accessible, you can use this swlink variant.
Depending on board layout, LED and force bootloader entry may be routed to
wrong pins.
