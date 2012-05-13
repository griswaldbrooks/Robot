########################################################################
#
#                             HETdemo.eww
#
#                                                   $Revision: 30366 $
#
########################################################################

DESCRIPTION
===========
This example project shows how to use the IAR Embedded Workbench
for ARM to develop code for the IAR evaluation board based on the
Texas Instruments TMS470R1B1M.
It shows basic use of the HET (High End Timer) unit on the chip.

The application programs the HET to periodically toggle the LED.


COMPATIBILITY
=============
The project is compatible with the IAR evaluation board based on the
Texas Instruments TMS470R1B1M.

The project is by default configured to use the J-Link JTAG interface.


GETTING STARTED
===============
Start the IAR Embedded Workbench for ARM.
Select File->Open->Workspace...
Open the workspace file
  <installation-root>\arm\armexamples\TexasInstruments\TMS470R1B1M\HETdemo\HETdemo.eww


NOTES ON THE HET (High End Timer)
=================================
The control code for the HET is written in the HET assembler language (pwm.het).
Documentation on the HET assembler can be requested from Texas instruments.
The HET assembler executable is located in
  <installation-root>\arm\bin\het470.exe
  
In this example the HET assembler generates two output files pwm.c and pwm.h.

The custom build feature of the IAR Embedded Workbench is used to automatically
invoke the HET assembler as part of the normal project build.
The custom build rule used in the HETDemo project is
  het470 -hc "$FILE_PATH$" "$FILE_BPATH$"


CONFIGURATION
=============
The application is downloaded to flash or RAM depending on the selected configuration.

For the flash configuration make sure that
- the board switches SW13, SW14 and SW14 is configured with the
  flash mapped at 0h, see the evaluation board documentation for details.
  SW13 towards CPU
  SW14 towards CPU
  SW15 opposite CPU

For the RAM configuration make sure that
- the board switches SW13, SW14 and SW14 is configured with the
  RAM mapped at 0h, see the evaluation board documentation for details.
  SW13 opposite CPU
  SW14 towards CPU
  SW15 opposite CPU
