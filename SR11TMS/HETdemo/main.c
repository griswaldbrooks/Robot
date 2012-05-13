#include <stdio.h>
#include <string.h>
#include <intrinsics.h>
#include <TexasInstruments/iotms470r1b1m.h>
#include "std_het.h"
#include "pwm.h"

__no_init volatile HETPROGRAM0_UN e_HETPROGRAM0_UN @ 0x800000;

#define Het0 0x01
#define Het2 0x04
#define Het4 0x10

void MemCopy32(unsigned long *dst, unsigned long *src, int bytes)
{
  for (int i = 0; i < (bytes + 3) / 4; i++)
    *dst++ = *src++;
}

int main(void)
{
  // Set up peripheral registers.
  // First disable interrupts.
  __disable_interrupt();

  PCR = 0x1E;
  PCR = 0x1F;  // enable peripherals

  HETGCR = 0x00010002;   /* Start HET          */

  /* copy HET instructions to HET ram*/
  MemCopy32((void *) &e_HETPROGRAM0_UN, (void *) HET_INIT0_PST, sizeof(HET_INIT0_PST));

  HETPFR = 0x0000052b;   /* Set PFR register	*/
  HETDIR = 0x00000000;   /* Set pin directions */
  HETDIR |= Het0+Het2+Het4;   /* Set pin directions */

  HETGCR |= 0x00000001;   /* Start HET          */

  for (;;)
  {
  }
}
