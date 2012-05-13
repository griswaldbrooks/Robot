main.o main.d : main.c ../../Source/include/FreeRTOS.h \
  ../../Source/include/projdefs.h FreeRTOSConfig.h \
  ../../Source/include/portable.h \
  ../../Source/include/../portable/GCC/ATMega323/portmacro.h \
  ../../Source/include/mpu_wrappers.h ../../Source/include/task.h \
  ../../Source/include/list.h ../../Source/include/croutine.h SoR_Utils.h \
  global.h avrlibdefs.h avrlibtypes.h uart4.h buffer.h rprintf.h \
  timer640.h pwm.c pwm.h a2d.h i2c.h i2cconf.h spi.h hardware.c
