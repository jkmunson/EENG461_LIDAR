#include <stdint.h>
#include <stdbool.h>

#include <common/tm4c123gh6pm.h>
#include <driverlib/rom.h>
#include <driverlib/sysctl.h>

#include "main.h"
#include "timers.h"
#include "uart_print.h"
#include "lidar.h"

#define Disable_Interrupts() __asm("CPSID I")
#define Enable_Interrupts() 


int main (void) {
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	
	setup_uart_printer();
	
	__asm("CPSIE I")
	
	uint16_t last_distance = 0; //Last stored distance
	int32_t last_print_time = 0;


	return (0);
}