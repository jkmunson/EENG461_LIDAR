#include <stdint.h>
#include <stdbool.h>

#include <inc/hw_uart.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_memmap.h>
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
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	
	setup_uart_printer();
	printlf("Init Lidar Comms\n");
	setup_lidar_comms();
	printlf("Stopping Lidar scan\n");
	stop_lidar_scan();
	printlf("Flushing Lidar buffer\n");
	ROM_SysCtlDelay(1000000);
	clear_lidar_IO();
	printlf("Starting Scan\n");
	start_lidar_scan();
	printlf("Looping\n");
	while(1) {
		process_packets();
	}
	
	return (0);
}

/*//printlf("%d:%d\n", ROM_UARTCharGet(UART2_BASE), ROM_UARTRxErrorGet(UART2_BASE));
		//ROM_UARTCharPut(UART0_BASE, ROM_UARTCharGet(UART2_BASE));
		printlf("%d\n", ROM_UARTRxErrorGet(UART2_BASE), ROM_UARTCharGet(UART2_BASE));
		//ROM_UARTCharPut(UART0_BASE, ROM_UARTRxErrorGet(UART2_BASE));
		ROM_UARTRxErrorClear(UART2_BASE);*/
