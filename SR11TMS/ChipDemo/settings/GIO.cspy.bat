@REM This batch file has been generated by the IAR Embedded Workbench
@REM C-SPY Debugger, as an aid to preparing a command line for running
@REM the cspybat command line utility using the appropriate settings.
@REM
@REM You can launch cspybat by typing the name of this batch file followed
@REM by the name of the debug file (usually an ELF/DWARF or UBROF file).
@REM Note that this file is generated every time a new debug session
@REM is initialized, so you may want to move or rename the file before
@REM making changes.
@REM 


"C:\Program Files\IAR Systems\Embedded Workbench 6.0 Evaluation\common\bin\cspybat" "C:\Program Files\IAR Systems\Embedded Workbench 6.0 Evaluation\arm\bin\armproc.dll" "C:\Program Files\IAR Systems\Embedded Workbench 6.0 Evaluation\arm\bin\armjlink.dll"  %1 --plugin "C:\Program Files\IAR Systems\Embedded Workbench 6.0 Evaluation\arm\bin\armbat.dll" --macro "C:\Robot\SR11TMS\ChipDemo\..\..\Configuration_Files\B1M_Files\RAMTMS470R1B1M.mac" --backend -B "--endian=big" "--cpu=ARM7TDMI" "--fpu=None" "-p" "C:\Program Files\IAR Systems\Embedded Workbench 6.0 Evaluation\arm\CONFIG\debugger\TexasInstruments\iotms470r1b1m.ddf" "--drv_verify_download" "--semihosting" "--device=TMS470R1B1M" "--drv_communication=USB0" "--jlink_speed=auto" "--jlink_initial_speed=32" 


