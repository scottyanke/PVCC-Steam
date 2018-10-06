This project is part of a larger monitoring system for air handlers, temperature
and humidity sensors, air conditioners, and a steam boiler.

This specific project is for monitoring a Hurst steam boiler.  It tracks when
the boiler is enabled (controlled by outside air temp), the boiler operator control,
when the main burner is on, and if there is an alarm from the Honeywell controller.
Additionally, it can track steam pressure using a sensor that outputs an analog
voltage.

For this monitoring device, an STM8S103F3 is used.  It connects to a RS485 module
for communication to the main program running on a Raspberry Pi.  This device only
responds to its id being received via RS485.  It will only talk when asked by the
main program.

The boiler monitoring is done using RIBU1Cs that ground the GPIO pins on the
STM8S103F3.  Normally the pins are pulled high using weak (10k) resistors.  The
relays in the RIBs short the GPIO signals to ground.  The steam pressure sensor
is an unbranded device that outputs a value from .5 volts to 4.5 volts depending
on the steam pressure from 0 to 15 psi.

There are two additional GPIO pins that can be used on the STM8S, but currently
nothing is connected.

Eclipse is used on Linux as an IDE and compiled with SDCC for the STM8S.
Programming is done using stm8flash and an ST Link/V2 on the STM8 connector.  It's
been working reliably.

I'm sure the ST Micro supplied programs would work, but I don't use Windows unless
I have no choice.

Anyone can use these programs however they wish.  These are designs that are
being used every day in a nursing home.
